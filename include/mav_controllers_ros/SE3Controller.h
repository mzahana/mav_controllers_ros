#ifndef SE3CONTROLLER_H
#define SE3CONTROLLER_H

#include <Eigen/Geometry>
#include <iostream>
// #include <Eigen/Dense>

class SE3Controller
{
 public:
  SE3Controller();

  void setMass(const float mass);
  void setGravity(const float g);
  void setPosition(const Eigen::Vector3f &position);
  void setVelocity(const Eigen::Vector3f &velocity);
  void setMaxIntegral(const float max_integral);
  void setMaxIntegralBody(const float max_integral_b);
  void setMaxAcceleration(const float max_acc);
  void setCurrentOrientation(const Eigen::Quaternionf &current_orientation);
  void resetIntegrals();
  void setMaxTiltAngle(const float max_tilt_angle);

  void calculateControl(const Eigen::Vector3f &des_pos, const Eigen::Vector3f &des_vel, const Eigen::Vector3f &des_acc,
                        const Eigen::Vector3f &des_jerk, const float des_yaw, const float des_yaw_dot,
                        const Eigen::Vector3f &kx, const Eigen::Vector3f &kv, const Eigen::Vector3f &ki,
                        const Eigen::Vector3f &ki_b);

  const Eigen::Vector3f &getComputedForce();
  const Eigen::Quaternionf &getComputedOrientation();
  const Eigen::Vector3f &getComputedAngularVelocity();

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 private:
  // Inputs for the controller
  float mass_;
  float g_;
  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;
  float max_pos_int_;
  float max_pos_int_b_;
  Eigen::Quaternionf current_orientation_;
  float cos_max_tilt_angle_;
  float max_accel_; // Maximum acceleration in m/s^2

  // Outputs of the controller
  Eigen::Vector3f force_; // Total force in inertial frame
  Eigen::Quaternionf orientation_;
  Eigen::Vector3f angular_velocity_;
  Eigen::Vector3f pos_int_;
  Eigen::Vector3f pos_int_b_;
};

#endif