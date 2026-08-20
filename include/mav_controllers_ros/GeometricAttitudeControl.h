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
  void setMaxAcceleration(const float max_acc);
  void setCurrentOrientation(const Eigen::Quaternionf &current_orientation);
  void setVelocityYaw(const bool vel_yaw);
  void resetIntegrals();
  void setMaxTiltAngle(const float max_tilt_angle);
  void setYawGain(const float yaw_gain);
  // Enable body-rate feedforward computed from the desired jerk / yaw rate
  // (differential flatness). Improves agile trajectory tracking; has no
  // effect on hover/step behavior when the reference jerk is zero.
  void setRateFeedforward(const bool enable);

  const Eigen::Vector3f &getComputedForce();
  const Eigen::Quaternionf &getComputedOrientation();
  const Eigen::Vector3f &getComputedAngularVelocity();
  const Eigen::Vector3f &getPosError();
  const Eigen::Vector3f &getVelError();
  const Eigen::Vector3f &getAttitudeError();
  const Eigen::Vector3f &getPosIntegral();
  bool isSaturated() const;

  /*
    dt: elapsed time [s] since the previous call; used for the position
    integrator. Pass 0 on the first call (or after a long gap) — the
    integrator simply does not accumulate on that cycle.
  */
  void calculateControl(const Eigen::Vector3f &des_pos, const Eigen::Vector3f &des_vel, const Eigen::Vector3f &des_acc,
                        const Eigen::Vector3f &des_jerk, const float des_yaw, const float des_yaw_dot,
                        const Eigen::Vector3f &kx, const Eigen::Vector3f &kv, const Eigen::Vector3f &ki,
                        const Eigen::Vector3f &kd, const float &attctrl_tau, const float dt);

private:

  Eigen::Vector3f controlPosition(const Eigen::Vector3f &target_pos, const Eigen::Vector3f &target_vel,
                                  const Eigen::Vector3f &target_acc, const float &des_yaw,
                                  const Eigen::Vector3f &kx, const Eigen::Vector3f &kv, const Eigen::Vector3f &ki,
                                  const Eigen::Vector3f &kd, const float dt);
  void computeBodyRateCmd(const Eigen::Vector3f &a_des, const Eigen::Vector3f &des_jerk,
                          const float &des_yaw, const float des_yaw_dot, const float &attctrl_tau);
  Eigen::Vector4f acc2quaternion(const Eigen::Vector3f &vector_acc, const float &yaw);
  Eigen::Vector3f poscontroller(const Eigen::Vector3f &pos_error, const Eigen::Vector3f &vel_error,
                                const Eigen::Vector3f &kx, const Eigen::Vector3f &kv,
                                const Eigen::Vector3f &ki, const float dt);

  // Inputs for the controller
  float mass_;
  float g_;
  Eigen::Vector3f gravity_vec_; // downward in inertial frame
  Eigen::Vector3f pos_;
  Eigen::Vector3f vel_;
  float max_pos_int_;
  Eigen::Quaternionf current_orientation_q_;
  Eigen::Vector4f current_orientation_vec_;
  float cos_max_tilt_angle_;
  float max_accel_; // Maximum acceleration in m/s^2

  // Outputs of the controller
  Eigen::Vector3f force_; // Total force in inertial frame
  Eigen::Quaternionf orientation_;
  Eigen::Vector3f angular_velocity_;
  Eigen::Vector3f pos_int_;

  // Errors
  Eigen::Vector3f pos_err_, vel_err_, att_err_;

  // If true, yaw will be computed internally
  bool velocity_yaw_;

  float yaw_gain_;

  // Saturation state (acceleration clamp or tilt limit active on the last
  // cycle). While saturated, the integrator holds (conditional
  // integration anti-windup).
  bool saturated_;

  bool rate_ff_enabled_;
};

#endif
