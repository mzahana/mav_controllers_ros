#include "geometric_controller_ros/GeometricAttitudeControl.h"

void GeometricAttitudeControl::setMass(const float mass)
{
  mass_ = mass;
}

void GeometricAttitudeControl::setGravity(const float g)
{
  g_ = g; // thhis should be positive
  gravity_vec_ = Eigen::Vector3d(0.0, 0.0, -1.0*g_);
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

void GeometricAttitudeControl::setMaxIntegralBody(const float max_integral_b)
{
  max_pos_int_b_ = max_integral_b;
}

void GeometricAttitudeControl::setCurrentOrientation(const Eigen::Quaternionf &current_orientation)
{
  current_orientation_ = current_orientation;
}

void GeometricAttitudeControl::setMaxTiltAngle(const float max_tilt_angle)
{
  if(max_tilt_angle > 0.0f && max_tilt_angle <= static_cast<float>(M_PI))
    cos_max_tilt_angle_ = std::cos(max_tilt_angle);
}

void GeometricAttitudeControl::setMaxAcceleration(const float max_acc)
{
  max_accel_ = max_acc;
  if(max_accel_ < 0)
  {
    std::cout << "max_accel_ can not be negative. Using the default of 10 m/s/s" << std::endl;
    max_accel_ = 10.0;
  }
}

void GeometricAttitudeControl::setDragCoeff(const Eigen::Vector3f &drag)
{
  D_ = drag;
}

Eigen::Vector3f GeometricAttitudeControl::controlPosition(const Eigen::Vector3f &target_pos, const Eigen::Vector3f &target_vel,
                                  const Eigen::Vector3f &target_acc,
                                  const Eigen::Vector3f &kx, const Eigen::Vector3f &kv)
{
  // Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3f a_ref = target_acc;
  if (velocity_yaw_) {
    mavYaw_ = atan2(vel_(1), vel_(0));
  }

  const Eigen::Vector4f q_ref = acc2quaternion(a_ref - gravity_vec_, mavYaw_);
  const Eigen::Matrix3f R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = pos_ - target_pos;
  const Eigen::Vector3d vel_error = vel_ - target_vel;

  // Position Controller
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error, kx, kv);

  // Rotor Drag compensation
  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

  // Reference acceleration
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_vec_;

  return a_des;
}

Eigen::Vector3f GeometricAttitudeControl::poscontroller(const Eigen::Vector3f &pos_error, const Eigen::Vector3f &vel_error, 
                                const Eigen::Vector3f &kx, const Eigen::Vector3f &kv)
{
  Eigen::Vector3f a_fb =
      kx.asDiagonal() * pos_error + kv.asDiagonal() * vel_error;  // feedforward term for trajectory error

  if (a_fb.norm() > max_accel_)
    a_fb = (max_accel_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  return a_fb;
}

/////////////// Helper functions //////////////////
// Ref: https://github.com/Jaeyoung-Lim/mavros_controllers/blob/master/geometric_controller/include/geometric_controller/common.h

Eigen::Matrix3f GeometricAttitudeControl::quat2RotMatrix(const Eigen::Vector4f &q) {
    Eigen::Matrix3f rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
        2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
        2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
        q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
}

Eigen::Vector4f GeometricAttitudeControl::rot2Quaternion(const Eigen::Matrix3f &R) {
  Eigen::Vector4f quat;
  float tr = R.trace();
  if (tr > 0.0) {
    float S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    float S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    float S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    float S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

