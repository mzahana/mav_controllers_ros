#include "mav_controllers_ros/GeometricAttitudeControl.h"

GeometricAttitudeControl::GeometricAttitudeControl()
    : mass_(0.5),
      g_(9.81),
      max_pos_int_(0.5),
      max_pos_int_b_(0.5),
      current_orientation_q_(Eigen::Quaternionf::Identity()),
      cos_max_tilt_angle_(-1.0),
      yaw_gain_(0.3)
{
}

GeometricAttitudeControl::~GeometricAttitudeControl()
{

}

void GeometricAttitudeControl::setMass(const float mass)
{
  mass_ = mass;
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

void GeometricAttitudeControl::setMaxIntegralBody(const float max_integral_b)
{
  max_pos_int_b_ = max_integral_b;
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
  if(max_accel_ < 0)
  {
    std::cout << "max_accel_ can not be negative. Using the default of 10 m/s/s" << std::endl;
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

void GeometricAttitudeControl::calculateControl(const Eigen::Vector3f &des_pos, const Eigen::Vector3f &des_vel, const Eigen::Vector3f &des_acc,
                        const Eigen::Vector3f &des_jerk, const float des_yaw, const float des_yaw_dot,
                        const Eigen::Vector3f &kx, const Eigen::Vector3f &kv, const Eigen::Vector3f &ki,
                        const Eigen::Vector3f &ki_b,
                        const Eigen::Vector3f &kd, const float &attctrl_tau)
{
  float yaw_d;
  if (velocity_yaw_) {
    yaw_d = atan2(vel_(1), vel_(0));
  }
  else
    yaw_d = des_yaw;

  // In this implementation, the gains need to be negated, but the will be passed as +ve numbers
  auto Kx = -kx;
  auto Kv = -kv;
  auto Ki = -ki;
  auto Kib = -ki_b;

  // Not used ??!  des_yaw_dot, des_jerk !!
  Eigen::Vector3f a_des = controlPosition(des_pos, des_vel, des_acc, yaw_d, Kx, Kv, Ki, Kib, kd);
  force_ = mass_ * a_des; // in inertial frame

  // This updates angular_velocity_ and orientation_ (desired angular vel, and desired orientation)
  computeBodyRateCmd( a_des, yaw_d, attctrl_tau);
  // attcontroller(a_des, yaw_d, attctrl_tau);
  // reducedAttController(a_des, yaw_d, attctrl_tau);
  // mixedAttController(a_des, yaw_d, attctrl_tau);

}

Eigen::Vector3f GeometricAttitudeControl::controlPosition(const Eigen::Vector3f &target_pos, const Eigen::Vector3f &target_vel,
                                  const Eigen::Vector3f &target_acc, const float &des_yaw,
                                  const Eigen::Vector3f &kx, const Eigen::Vector3f &kv, 
                                  const Eigen::Vector3f &ki, const Eigen::Vector3f &kib,
                                  const Eigen::Vector3f &kd)
{
  // Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3f a_ref = target_acc;

  const Eigen::Vector4f q_ref = acc2quaternion(a_ref - gravity_vec_, des_yaw);
  const Eigen::Matrix3f R_ref = quat2RotMatrix(q_ref);

  pos_err_ = pos_ - target_pos;
  vel_err_ = vel_ - target_vel;

  // Position Controller
  const Eigen::Vector3f a_fb = poscontroller(pos_err_, vel_err_, kx, kv, ki, kib);

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
      a_des = lambda * acc_control + gravity_vec_;
  }

  return a_des;
}

void GeometricAttitudeControl::computeBodyRateCmd( const Eigen::Vector3f &a_des, const float &des_yaw, const float &attctrl_tau)
{
  // Reference attitude
  Eigen::Vector4f q_des = acc2quaternion(a_des, des_yaw);
  orientation_.w() = q_des(0);
  orientation_.x() = q_des(1);
  orientation_.y() = q_des(2);
  orientation_.z() = q_des(3);

  Eigen::Vector4f ratecmd;
  Eigen::Matrix3f rotmat;    // Rotation matrix of current attitude
  Eigen::Matrix3f rotmat_d;  // Rotation matrix of desired attitude

  rotmat = quat2RotMatrix(current_orientation_vec_);
  rotmat_d = quat2RotMatrix(q_des);

  att_err_ = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
  angular_velocity_ = (2.0 / attctrl_tau) * att_err_;

  // const Eigen::Vector3d zb = rotmat.col(2);
  // desired_thrust_(0) = 0.0;
  // desired_thrust_(1) = 0.0;
  // desired_thrust_(2) = a_des.dot(zb);

  // bodyrate_cmd.head(3) = controller_->getDesiredRate();
  // double thrust_command = controller_->getDesiredThrust().z();
  // bodyrate_cmd(3) =
  //     std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command +
  //                                     norm_thrust_offset_));  // Calculate thrustcontroller_->getDesiredThrust()(3);
}

void GeometricAttitudeControl::attcontroller( Eigen::Vector3f& ref_acc, const float &des_yaw, const float &attctrl_tau)
{
  Eigen::Vector4f qe, q_inv, inverse;
  inverse << 1.0, -1.0, -1.0, -1.0;
  q_inv = inverse.asDiagonal() * current_orientation_vec_;
  Eigen::Vector4f q_des = acc2quaternion(ref_acc, des_yaw);
  Eigen::Matrix3f rotmat;    // Rotation matrix of current attitude
  Eigen::Matrix3f rotmat_d;  // Rotation matrix of desired attitude

  rotmat = quat2RotMatrix(current_orientation_vec_);
  rotmat_d = quat2RotMatrix(q_des);

  att_err_ = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);

  orientation_.w() = q_des(0);
  orientation_.x() = q_des(1);
  orientation_.y() = q_des(2);
  orientation_.z() = q_des(3);
  qe = quatMultiplication(q_inv, q_des);
  angular_velocity_(0) = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(1);
  angular_velocity_(1) = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(2);
  angular_velocity_(2) = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(3);
}

void GeometricAttitudeControl::reducedAttController( Eigen::Vector3f &ref_acc, const float &des_yaw, const float &attctrl_tau)
{
  Eigen::Vector4f ratecmd;
  Eigen::Vector4f qe, q_inv, inverse, qe_red, qe_red_inv, q_mix;
  Eigen::Matrix3f rotmat, ref_rotmat;
  Eigen::Vector3f zb, ebz, ecmdz, k_vec;
  double alpha, alpha_mix;

  Eigen::Vector4f q_des = acc2quaternion(ref_acc, des_yaw);
  Eigen::Matrix3f rotmat_d;  // Rotation matrix of desired attitude

  rotmat = quat2RotMatrix(current_orientation_vec_);//Current Orientation Rotation Matrix
  rotmat_d = quat2RotMatrix(q_des);

  att_err_ = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);

  orientation_.w() = q_des(0);
  orientation_.x() = q_des(1);
  orientation_.y() = q_des(2);
  orientation_.z() = q_des(3);

  ref_rotmat = quat2RotMatrix(q_des); //Command Orientation Rotation Matrix
  // Full attitude controller  
  inverse << 1.0, -1.0, -1.0, -1.0;
  q_inv = inverse.asDiagonal() * current_orientation_vec_;
  qe = quatMultiplication(q_inv, q_des);

  // Reduced attitude controller
  ebz = rotmat.col(2);
  ecmdz = ref_rotmat.col(2);
  alpha = std::acos(ebz.dot(ecmdz));
  k_vec = ebz.cross(ecmdz);
  k_vec = k_vec.normalized();

  qe_red << cos(alpha), std::sin(alpha) * k_vec(0), std::sin(alpha) * k_vec(1), std::sin(alpha) * k_vec(2); // Reduced error quaternion
  qe_red_inv = inverse.asDiagonal() * qe_red;

  q_mix = quatMultiplication(qe_red_inv, qe);
  alpha_mix = std::acos(2*q_mix(0));
  q_mix << std::cos(.5 * alpha_mix * yaw_gain_), 0, 0, std::sin(.5 * alpha_mix * yaw_gain_);

  qe = quatMultiplication(qe_red, q_mix); //Update error quaternion to mixed reduced quaternion

  // Mixing reduced and full attitude controller

  angular_velocity_(0) = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(1);
  angular_velocity_(1) = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(2);
  angular_velocity_(2) = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(3);

}

void GeometricAttitudeControl::mixedAttController( Eigen::Vector3f &ref_acc, const float &des_yaw, const float &attctrl_tau)
{
  // Using Ref: https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf?sequence=1&isAllowed=y
  
  // commanded z-zxis of body expressed in inertial frame Eq(43)
  Eigen::Vector3f ez_cmd_IB = ref_acc/ref_acc.norm();
  // Z-axis of the current attitude
  Eigen::Vector3f ez_B = quat2RotMatrix(current_orientation_vec_).col(2);
  // the unit vector of the error quaternion of the reduced attitude Eq(45)
  Eigen::Vector3f qe_k = ez_B.cross(ez_cmd_IB);
  qe_k = qe_k.normalized();
  // Alpha angle Eq(46)
  float alpha = std::acos(ez_B.dot(ez_cmd_IB));
  // Reduced error quaternion, Eq(45)
  Eigen::Vector4f qe_red;
  qe_red << std::cos(alpha/2),
            std::sin(alpha/2)*qe_k(0),
            std::sin(alpha/2)*qe_k(1),
            std::sin(alpha/2)*qe_k(2);

  Eigen::Vector4f q_cmd_red = quatMultiplication(current_orientation_vec_, qe_red); // Eq(47)

  Eigen::Matrix3f R_KI; R_KI.setZero();
  R_KI(0,0) = std::cos(des_yaw); R_KI(0,1) = std::sin(des_yaw);
  R_KI(1,0) = -std::sin(des_yaw); R_KI(1,1) = std::cos(des_yaw);
  R_KI(2,2) = 1.0;
  Eigen::Vector3f ez_cmd_KB = R_KI*ez_cmd_IB; // Eq(48)
  float pitch_cmd;
  if(ez_cmd_KB(2) != 0)
    pitch_cmd = std::atan(ez_cmd_KB(0)/ez_cmd_KB(2)); // Eq(49)
  else
    pitch_cmd = 0.0;

  Eigen::Matrix3f R_LK; R_LK.setZero();
  R_LK(0,0) = std::cos(pitch_cmd); R_LK(0,2) = -std::sin(pitch_cmd);
  R_LK(1,1) = 1.0;
  R_LK(2,0) = std::sin(pitch_cmd); R_LK(2,2) = std::cos(pitch_cmd);

  Eigen::Vector3f ez_cmd_LB = R_LK*ez_cmd_KB; // Eq(50)

  float roll_cmd = std::atan2(-ez_cmd_LB(1), ez_cmd_LB(2)); // Eq(51)

  Eigen::Vector4f q_cmd_full = quatFromEulerZYX(des_yaw, pitch_cmd, roll_cmd); // Eq(52)
 
  Eigen::Vector4f q_cmd_red_inv =  inverseQuaternion(q_cmd_red);
  Eigen::Vector4f q_mix = quatMultiplication(q_cmd_red_inv, q_cmd_full);
  float alpha_mix = 2*std::acos(q_mix(0));
  Eigen::Vector4f q_mix_p; q_mix_p.setZero();
  q_mix_p(0) = std::cos(yaw_gain_*alpha_mix/2);
  q_mix_p(3) = std::sin(yaw_gain_*alpha_mix/2);

  Eigen::Vector4f q_cmd = quatMultiplication(q_cmd_red, q_mix_p); // Eq(54)

  orientation_.w() = q_cmd(0);
  orientation_.x() = q_cmd(1);
  orientation_.y() = q_cmd(2);
  orientation_.z() = q_cmd(3);

  // Mixing reduced and full attitude controller
  Eigen::Vector4f q_inv = inverseQuaternion(current_orientation_vec_);
  Eigen::Vector4f qe = quatMultiplication(q_inv, q_cmd);

  // Desired angular velocity Eq(23)
  angular_velocity_(0) = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(1);
  angular_velocity_(1) = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(2);
  angular_velocity_(2) = (2.0 / attctrl_tau) * std::copysign(1.0, qe(0)) * qe(3);
}

Eigen::Vector3f GeometricAttitudeControl::poscontroller(const Eigen::Vector3f &pos_error, const Eigen::Vector3f &vel_error, 
                                                        const Eigen::Vector3f &kx, const Eigen::Vector3f &kv,
                                                        const Eigen::Vector3f &ki, 
                                                        const Eigen::Vector3f &kib)
{
  for(int i = 0; i < 3; i++)
  {
    if(kx(i) != 0)
      pos_int_(i) += ki(i) * pos_error(i);

    // Limit integral term
    if(pos_int_(i) > max_pos_int_)
      pos_int_(i) = max_pos_int_;
    else if(pos_int_(i) < -max_pos_int_)
      pos_int_(i) = -max_pos_int_;
  }
  Eigen::Vector3f a_fb =
      kx.asDiagonal() * pos_error + kv.asDiagonal() * vel_error + pos_int_;  // feedforward term for trajectory error

  if (a_fb.norm() > max_accel_)
    a_fb = (max_accel_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

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
  pos_int_b_ = Eigen::Vector3f::Zero();
}

