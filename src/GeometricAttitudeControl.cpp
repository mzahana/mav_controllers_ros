#include "mav_controllers_ros/GeometricAttitudeControl.h"

GeometricAttitudeControl::GeometricAttitudeControl()
    : mass_(0.5),
      g_(9.81),
      max_pos_int_(0.5),
      current_orientation_q_(Eigen::Quaternionf::Identity()),
      cos_max_tilt_angle_(-1.0),
      max_accel_(10.0),
      pos_int_(Eigen::Vector3f::Zero()),
      velocity_yaw_(false),
      yaw_gain_(0.3),
      saturated_(false),
      rate_ff_enabled_(false)
{
  gravity_vec_ = Eigen::Vector3f(0.0, 0.0, -g_);
}

GeometricAttitudeControl::~GeometricAttitudeControl()
{

}

void GeometricAttitudeControl::setMass(const float mass)
{
  if(mass > 0.0f)
    mass_ = mass;
  else
    std::cerr << "[GeometricAttitudeControl] mass must be > 0; keeping " << mass_ << std::endl;
}

void GeometricAttitudeControl::setGravity(const float g)
{
  g_ = g; // this should be positive
  gravity_vec_ = Eigen::Vector3f(0.0, 0.0, -g_);
}

void GeometricAttitudeControl::setPosition(const Eigen::Vector3f &position)
{
  pos_ = position;
}

void GeometricAttitudeControl::setVelocity(const Eigen::Vector3f &velocity)
{
  vel_ = velocity;
}

void GeometricAttitudeControl::setMaxIntegral(const float max_integral)
{
  max_pos_int_ = max_integral;
}

void GeometricAttitudeControl::setCurrentOrientation(const Eigen::Quaternionf &current_orientation)
{
  current_orientation_q_ = current_orientation;
  current_orientation_vec_(0) = current_orientation_q_.w();
  current_orientation_vec_(1) = current_orientation_q_.x();
  current_orientation_vec_(2) = current_orientation_q_.y();
  current_orientation_vec_(3) = current_orientation_q_.z();
}

void GeometricAttitudeControl::setMaxTiltAngle(const float max_tilt_angle)
{
  if(max_tilt_angle > 0.0f && max_tilt_angle <= static_cast<float>(M_PI))
    cos_max_tilt_angle_ = std::cos(max_tilt_angle);
}

void GeometricAttitudeControl::setMaxAcceleration(const float max_acc)
{
  max_accel_ = max_acc;
  if(max_accel_ <= 0)
  {
    std::cerr << "[GeometricAttitudeControl] max_accel must be > 0. Using the default of 10 m/s/s" << std::endl;
    max_accel_ = 10.0;
  }
}

void GeometricAttitudeControl::setVelocityYaw(const bool vel_yaw)
{
  velocity_yaw_ = vel_yaw;
}

void GeometricAttitudeControl::setYawGain(const float yaw_gain)
{
  yaw_gain_ = yaw_gain;
}

void GeometricAttitudeControl::setRateFeedforward(const bool enable)
{
  rate_ff_enabled_ = enable;
}

const Eigen::Vector3f &GeometricAttitudeControl::getComputedForce()
{
  return force_;
}

const Eigen::Quaternionf &GeometricAttitudeControl::getComputedOrientation()
{
  return orientation_;
}

const Eigen::Vector3f &GeometricAttitudeControl::getComputedAngularVelocity()
{
  return angular_velocity_;
}

const Eigen::Vector3f &GeometricAttitudeControl::getPosError()
{
  return pos_err_;
}

const Eigen::Vector3f &GeometricAttitudeControl::getVelError()
{
  return vel_err_;
}

const Eigen::Vector3f &GeometricAttitudeControl::getAttitudeError()
{
  return att_err_;
}

const Eigen::Vector3f &GeometricAttitudeControl::getPosIntegral()
{
  return pos_int_;
}

bool GeometricAttitudeControl::isSaturated() const
{
  return saturated_;
}

void GeometricAttitudeControl::calculateControl(const Eigen::Vector3f &des_pos, const Eigen::Vector3f &des_vel, const Eigen::Vector3f &des_acc,
                        const Eigen::Vector3f &des_jerk, const float des_yaw, const float des_yaw_dot,
                        const Eigen::Vector3f &kx, const Eigen::Vector3f &kv, const Eigen::Vector3f &ki,
                        const Eigen::Vector3f &kd, const float &attctrl_tau, const float dt)
{
  float yaw_d;
  if (velocity_yaw_) {
    yaw_d = atan2(vel_(1), vel_(0));
  }
  else
    yaw_d = des_yaw;

  // In this implementation, the gains need to be negated, but they will be passed as +ve numbers
  auto Kx = -kx;
  auto Kv = -kv;
  auto Ki = -ki;

  Eigen::Vector3f a_des = controlPosition(des_pos, des_vel, des_acc, yaw_d, Kx, Kv, Ki, kd, dt);
  force_ = mass_ * a_des; // in inertial frame

  // This updates angular_velocity_ and orientation_ (desired angular vel, and desired orientation)
  computeBodyRateCmd(a_des, des_jerk, yaw_d, des_yaw_dot, attctrl_tau);
}

Eigen::Vector3f GeometricAttitudeControl::controlPosition(const Eigen::Vector3f &target_pos, const Eigen::Vector3f &target_vel,
                                  const Eigen::Vector3f &target_acc, const float &des_yaw,
                                  const Eigen::Vector3f &kx, const Eigen::Vector3f &kv,
                                  const Eigen::Vector3f &ki, const Eigen::Vector3f &kd, const float dt)
{
  // Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3f a_ref = target_acc;

  const Eigen::Vector4f q_ref = acc2quaternion(a_ref - gravity_vec_, des_yaw);
  const Eigen::Matrix3f R_ref = quat2RotMatrix(q_ref);

  pos_err_ = pos_ - target_pos;
  vel_err_ = vel_ - target_vel;

  // Position Controller
  const Eigen::Vector3f a_fb = poscontroller(pos_err_, vel_err_, kx, kv, ki, dt);

  // Rotor Drag compensation
  const Eigen::Vector3f a_rd = R_ref * kd.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

  // Reference acceleration
  const Eigen::Vector3f acc_control = a_fb + a_ref - a_rd;
  Eigen::Vector3f a_des = acc_control - gravity_vec_;

  // Limit angle
  float lambda = 1.0f;
  if(Eigen::Vector3f::UnitZ().dot(a_des.normalized()) < cos_max_tilt_angle_)
  {
    const float x = acc_control.x(), y = acc_control.y(), z = acc_control.z();
    const float cot_max_tilt_angle = cos_max_tilt_angle_ / std::sqrt(1 - cos_max_tilt_angle_ * cos_max_tilt_angle_);
    lambda = -g_ / (z - std::sqrt(x * x + y * y) * cot_max_tilt_angle);
    if(lambda > 0 && lambda <= 1)
    {
      a_des = lambda * acc_control + gravity_vec_;
      saturated_ = true;  // tilt limit active -> hold the integrator
    }
  }

  return a_des;
}

void GeometricAttitudeControl::computeBodyRateCmd(const Eigen::Vector3f &a_des, const Eigen::Vector3f &des_jerk,
                                                  const float &des_yaw, const float des_yaw_dot, const float &attctrl_tau)
{
  // Reference attitude
  Eigen::Vector4f q_des = acc2quaternion(a_des, des_yaw);
  orientation_.w() = q_des(0);
  orientation_.x() = q_des(1);
  orientation_.y() = q_des(2);
  orientation_.z() = q_des(3);

  Eigen::Matrix3f rotmat;    // Rotation matrix of current attitude
  Eigen::Matrix3f rotmat_d;  // Rotation matrix of desired attitude

  rotmat = quat2RotMatrix(current_orientation_vec_);
  rotmat_d = quat2RotMatrix(q_des);

  att_err_ = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
  angular_velocity_ = (2.0 / attctrl_tau) * att_err_;

  // Body-rate feedforward from differential flatness (Mellinger/Faessler):
  // for a trajectory with jerk j and total acceleration a (thrust axis
  // z_B = a/|a|), the reference body rates are
  //   w_x = -h_w . y_B,  w_y = h_w . x_B,  w_z = yaw_dot * (e3 . z_B)
  // with h_w = (j - (z_B . j) z_B) / |a|.
  if(rate_ff_enabled_)
  {
    const float a_norm = a_des.norm();
    if(a_norm > 1.0f)  // no meaningful thrust axis near free-fall
    {
      const Eigen::Vector3f x_B = rotmat_d.col(0);
      const Eigen::Vector3f y_B = rotmat_d.col(1);
      const Eigen::Vector3f z_B = rotmat_d.col(2);
      const Eigen::Vector3f h_w = (des_jerk - (z_B.dot(des_jerk)) * z_B) / a_norm;
      Eigen::Vector3f w_ff;
      w_ff(0) = -h_w.dot(y_B);
      w_ff(1) = h_w.dot(x_B);
      w_ff(2) = des_yaw_dot * z_B(2);
      // The feedforward is expressed in the *desired* body frame; for the
      // small attitude errors of normal tracking this matches the current
      // body frame closely. Bound it defensively.
      const float w_ff_max = 3.0f;  // rad/s
      for(int i = 0; i < 3; i++)
        w_ff(i) = std::max(-w_ff_max, std::min(w_ff_max, w_ff(i)));
      angular_velocity_ += w_ff;
    }
  }
}

Eigen::Vector3f GeometricAttitudeControl::poscontroller(const Eigen::Vector3f &pos_error, const Eigen::Vector3f &vel_error,
                                                        const Eigen::Vector3f &kx, const Eigen::Vector3f &kv,
                                                        const Eigen::Vector3f &ki, const float dt)
{
  // Conditional-integration anti-windup: only accumulate when the previous
  // cycle was not saturated (accel clamp / tilt limit), and only with a
  // valid dt. Note ki arrives negated (like kx/kv), and pos_error = pos -
  // target, so the accumulated term already carries the correct sign.
  const bool allow_integration = !saturated_ && dt > 0.0f && dt < 0.5f;
  saturated_ = false;  // recomputed below and by the tilt limit

  for(int i = 0; i < 3; i++)
  {
    if(ki(i) != 0 && allow_integration)
      pos_int_(i) += ki(i) * pos_error(i) * dt;

    // Limit integral term
    if(pos_int_(i) > max_pos_int_)
      pos_int_(i) = max_pos_int_;
    else if(pos_int_(i) < -max_pos_int_)
      pos_int_(i) = -max_pos_int_;
  }

  Eigen::Vector3f a_fb =
      kx.asDiagonal() * pos_error + kv.asDiagonal() * vel_error + pos_int_;  // feedforward term for trajectory error

  // Altitude-priority saturation: when the feedback acceleration exceeds
  // the budget, keep the vertical component (altitude keeps the vehicle
  // alive) and shed horizontal acceleration first.
  const float a_norm = a_fb.norm();
  if(a_norm > max_accel_)
  {
    saturated_ = true;
    float az = std::max(-max_accel_, std::min(max_accel_, a_fb.z()));
    const float xy_budget = std::sqrt(std::max(0.0f, max_accel_ * max_accel_ - az * az));
    const float xy_norm = std::sqrt(a_fb.x() * a_fb.x() + a_fb.y() * a_fb.y());
    if(xy_norm > xy_budget && xy_norm > 1e-6f)
    {
      const float s = xy_budget / xy_norm;
      a_fb.x() *= s;
      a_fb.y() *= s;
    }
    a_fb.z() = az;
  }

  return a_fb;
}

Eigen::Vector4f GeometricAttitudeControl::acc2quaternion(const Eigen::Vector3f &vector_acc, const float &yaw)
{
  Eigen::Vector4f quat;
  Eigen::Vector3f zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3f rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

void GeometricAttitudeControl::resetIntegrals()
{
  pos_int_ = Eigen::Vector3f::Zero();
  saturated_ = false;
}
