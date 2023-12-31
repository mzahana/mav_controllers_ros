#ifndef GEOMETRICATTITUDECONTROL_H
#define GEOMETRICATTITUDECONTROL_H

#include <Eigen/Dense>
#include <iostream>
#include "mav_controllers_ros/utils.h"
class GeometricAttitudeControl
{
 public:
  GeometricAttitudeControl();
  ~GeometricAttitudeControl();

  void setMass(const float mass);
  void setGravity(const float g);
  void setPosition(const Eigen::Vector3f &position);
  void setVelocity(const Eigen::Vector3f &velocity);
  void setMaxIntegral(const float max_integral);
  void setMaxIntegralBody(const float max_integral_b);
  void setMaxAcceleration(const float max_acc);
  void setCurrentOrientation(const Eigen::Quaternionf &current_orientation);
  void setVelocityYaw(const bool vel_yaw);
  void resetIntegrals();
  void setMaxTiltAngle(const float max_tilt_angle);
  void setYawGain(const float yaw_gain);

  const Eigen::Vector3f &getComputedForce();
  const Eigen::Quaternionf &getComputedOrientation();
  const Eigen::Vector3f &getComputedAngularVelocity();
  const Eigen::Vector3f &getPosError();
  const Eigen::Vector3f &getVelError();
  const Eigen::Vector3f &getAttitudeError();

  void calculateControl(const Eigen::Vector3f &des_pos, const Eigen::Vector3f &des_vel, const Eigen::Vector3f &des_acc,
                        const Eigen::Vector3f &des_jerk, const float des_yaw, const float des_yaw_dot,
                        const Eigen::Vector3f &kx, const Eigen::Vector3f &kv, const Eigen::Vector3f &ki,
                        const Eigen::Vector3f &ki_b,
                        const Eigen::Vector3f &kd, const float &attctrl_tau);

private:

  Eigen::Vector3f controlPosition(const Eigen::Vector3f &target_pos, const Eigen::Vector3f &target_vel,
                                  const Eigen::Vector3f &target_acc, const float &des_yaw,
                                  const Eigen::Vector3f &kx, const Eigen::Vector3f &kv, const Eigen::Vector3f &ki, 
                                  const Eigen::Vector3f &kib, const Eigen::Vector3f &kd);
  void computeBodyRateCmd(const Eigen::Vector3f &a_des, const float &des_yaw, const float &attctrl_tau) ;
  void attcontroller( Eigen::Vector3f& ref_acc, const float &des_yaw, const float &attctrl_tau);
  Eigen::Vector4f acc2quaternion(const Eigen::Vector3f &vector_acc, const float &yaw);
  Eigen::Vector3f poscontroller(const Eigen::Vector3f &pos_error, const Eigen::Vector3f &vel_error, 
                                const Eigen::Vector3f &kx, const Eigen::Vector3f &kv,
                                const Eigen::Vector3f &ki, 
                                  const Eigen::Vector3f &kib);
  void reducedAttController(Eigen::Vector3f &ref_acc, const float &des_yaw, const float &attctrl_tau);
  void mixedAttController( Eigen::Vector3f &ref_acc, const float &des_yaw, const float &attctrl_tau);


  // Inputs for the controller
  float mass_;
  float g_;
  Eigen::Vector3f gravity_vec_; // downward in inertial frame
  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;
  float max_pos_int_;
  float max_pos_int_b_;
  Eigen::Quaternionf current_orientation_q_;
  Eigen::Vector4f current_orientation_vec_;
  float cos_max_tilt_angle_;
  float max_accel_; // Maximum acceleration in m/s^2

  // Outputs of the controller
  Eigen::Vector3f force_; // Total force in inertial frame
  Eigen::Quaternionf orientation_;
  Eigen::Vector3f angular_velocity_;
  Eigen::Vector3f pos_int_;
  Eigen::Vector3f pos_int_b_;

  // Errors
  Eigen::Vector3f pos_err_, vel_err_, att_err_;

  // If true, yaw will be computed internally
  bool velocity_yaw_;

  float yaw_gain_;
};

#endif