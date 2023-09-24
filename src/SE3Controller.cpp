#include <geometric_controller_ros/SE3Controller.h>

// #include <ros/console.h>
// #include <tf/transform_datatypes.h>

SE3Controller::SE3Controller()
    : mass_(0.5),
      g_(9.81),
      max_pos_int_(0.5),
      max_pos_int_b_(0.5),
      current_orientation_(Eigen::Quaternionf::Identity()),
      cos_max_tilt_angle_(-1.0)
{
}

void SE3Controller::setMass(const float mass)
{
  mass_ = mass;
}

void SE3Controller::setGravity(const float g)
{
  g_ = g;
}

void SE3Controller::setPosition(const Eigen::Vector3f &position)
{
  pos_ = position;
}

void SE3Controller::setVelocity(const Eigen::Vector3f &velocity)
{
  vel_ = velocity;
}

void SE3Controller::setMaxIntegral(const float max_integral)
{
  max_pos_int_ = max_integral;
}

void SE3Controller::setMaxIntegralBody(const float max_integral_b)
{
  max_pos_int_b_ = max_integral_b;
}

void SE3Controller::setCurrentOrientation(const Eigen::Quaternionf &current_orientation)
{
  current_orientation_ = current_orientation;
}

void SE3Controller::setMaxTiltAngle(const float max_tilt_angle)
{
  if(max_tilt_angle > 0.0f && max_tilt_angle <= static_cast<float>(M_PI))
    cos_max_tilt_angle_ = std::cos(max_tilt_angle);
}

void SE3Controller::calculateControl(const Eigen::Vector3f &des_pos, const Eigen::Vector3f &des_vel,
                                  const Eigen::Vector3f &des_acc, const Eigen::Vector3f &des_jerk, const float des_yaw,
                                  const float des_yaw_dot, const Eigen::Vector3f &kx, const Eigen::Vector3f &kv,
                                  const Eigen::Vector3f &ki, const Eigen::Vector3f &ki_b)
{
  const Eigen::Vector3f e_pos = des_pos - pos_;
  const Eigen::Vector3f e_vel = des_vel - vel_;

  for(int i = 0; i < 3; i++)
  {
    if(kx(i) != 0)
      pos_int_(i) += ki(i) * e_pos(i);

    // Limit integral term
    if(pos_int_(i) > max_pos_int_)
      pos_int_(i) = max_pos_int_;
    else if(pos_int_(i) < -max_pos_int_)
      pos_int_(i) = -max_pos_int_;
  }
  // ROS_DEBUG_THROTTLE(2, "Integrated world disturbance compensation [N]: {x: %2.2f, y: %2.2f, z: %2.2f}", pos_int_(0),
  // pos_int_(1), pos_int_(2));

  Eigen::Quaternionf q(current_orientation_);
  const Eigen::Vector3f e_pos_b = q.inverse() * e_pos;
  for(int i = 0; i < 3; i++)
  {
    if(kx(i) != 0)
      pos_int_b_(i) += ki_b(i) * e_pos_b(i);

    // Limit integral term in the body
    if(pos_int_b_(i) > max_pos_int_b_)
      pos_int_b_(i) = max_pos_int_b_;
    else if(pos_int_b_(i) < -max_pos_int_b_)
      pos_int_b_(i) = -max_pos_int_b_;
  }
  // ROS_DEBUG_THROTTLE(2, "Integrated body disturbance compensation [N]: {x: %2.2f, y: %2.2f, z: %2.2f}",
  // pos_int_b_(0), pos_int_b_(1), pos_int_b_(2));

  const Eigen::Vector3f acc_grav = g_ * Eigen::Vector3f::UnitZ();
  const Eigen::Vector3f acc_control = kx.asDiagonal() * e_pos + kv.asDiagonal() * e_vel + pos_int_ + des_acc;
  Eigen::Vector3f acc_total = acc_control + acc_grav;

  // Check and limit tilt angle
  float lambda = 1.0f;
  if(Eigen::Vector3f::UnitZ().dot(acc_total.normalized()) < cos_max_tilt_angle_)
  {
    const float x = acc_control.x(), y = acc_control.y(), z = acc_control.z();
    const float cot_max_tilt_angle = cos_max_tilt_angle_ / std::sqrt(1 - cos_max_tilt_angle_ * cos_max_tilt_angle_);
    lambda = -g_ / (z - std::sqrt(x * x + y * y) * cot_max_tilt_angle);
    if(lambda > 0 && lambda <= 1)
      acc_total = lambda * acc_control + acc_grav;
  }

  force_.noalias() = mass_ * acc_total;

  // std::cout << "Force: " << force_.transpose() << std::endl;

  // Following frame convention in
  // Ref: https://doi.org/10.1109/ICRA.2011.5980409
  Eigen::Vector3f b1c, b2c, b3c;
  const Eigen::Vector3f b1d(std::cos(des_yaw), std::sin(des_yaw), 0);

  if(force_.norm() > 1e-6f)
    b3c.noalias() = force_.normalized();
  else
    b3c.noalias() = Eigen::Vector3f::UnitZ();

  b2c.noalias() = b3c.cross(b1d).normalized();
  b1c.noalias() = b2c.cross(b3c).normalized();

  const Eigen::Vector3f force_dot =
      mass_ * lambda * (kx.asDiagonal() * e_vel + des_jerk);  // Ignoring kv*e_acc and ki*e_pos terms
  const Eigen::Vector3f b3c_dot = b3c.cross(force_dot / force_.norm()).cross(b3c);
  const Eigen::Vector3f b1d_dot(-std::sin(des_yaw) * des_yaw_dot, std::cos(des_yaw) * des_yaw_dot, 0);
  const Eigen::Vector3f b2c_dot = b2c.cross((b3c.cross(b1d_dot) + b3c_dot.cross(b1d)) / (b3c.cross(b1d)).norm()).cross(b2c);
  const Eigen::Vector3f b1c_dot = b2c_dot.cross(b3c) + b2c.cross(b3c_dot);

  Eigen::Matrix3f R;
  R << b1c, b2c, b3c;
  orientation_ = Eigen::Quaternionf(R);

  Eigen::Matrix3f R_dot;
  R_dot << b1c_dot, b2c_dot, b3c_dot;

  const Eigen::Matrix3f omega_hat = R.transpose() * R_dot;
  angular_velocity_ = Eigen::Vector3f(omega_hat(2, 1), omega_hat(0, 2), omega_hat(1, 0));
}

const Eigen::Vector3f &SE3Controller::getComputedForce()
{
  return force_;
}

const Eigen::Quaternionf &SE3Controller::getComputedOrientation()
{
  return orientation_;
}

const Eigen::Vector3f &SE3Controller::getComputedAngularVelocity()
{
  return angular_velocity_;
}

void SE3Controller::resetIntegrals()
{
  pos_int_ = Eigen::Vector3f::Zero();
  pos_int_b_ = Eigen::Vector3f::Zero();
}