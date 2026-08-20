# production-hardening branch — changes vs upstream master

All changes verified by closed-loop simulation against the `geo_tuner`
tuning framework (real controller node + physics sim): identical tracking
convergence to upstream, plus the new behaviors below.

## GeometricAttitudeControl (core)

1. **dt-correct integrator** — `pos_int += ki * e * dt` (was `+= ki * e`
   per call: integral strength silently scaled with the setpoint rate).
   `calculateControl(...)` now takes `dt`; pass 0 on the first call/after
   gaps. Verified: with a 10 % thrust-map error and `ki.z=1.5`, steady z
   error goes from ~0.36 m (PD) to 0.000 m, identically at 25 Hz and
   50 Hz setpoint rates.
2. **Anti-windup by conditional integration** — the integrator holds while
   the acceleration clamp or the tilt limit is saturated (`isSaturated()`
   exposed for logging).
3. **Altitude-priority acceleration saturation** — when `a_fb` exceeds
   `max_accel`, the vertical component is preserved and horizontal
   acceleration is shed first (was: uniform vector scaling, which gave away
   altitude authority exactly when saturated).
4. **Body-rate feedforward from differential flatness** (Mellinger/
   Faessler): reference body rates computed from desired jerk and yaw rate
   are added to the feedback rate command (bounded ±3 rad/s, disabled near
   free-fall). Enable/disable with `enable_rate_feedforward` (default on).
   Zero effect at hover or with jerk-free references. The previously
   unused `des_jerk`/`des_yaw_dot` inputs are now consumed.
5. **kib (body-frame integral) removed** — it was dead code: the
   accumulator was never updated. `gains.kib.*` / `mas_pos_int_b` params
   are still accepted (old configs load) with a deprecation warning if
   nonzero.
6. Removed unused alternate attitude laws (`attcontroller`,
   `reducedAttController`, `mixedAttController`) — dead code kept in git
   history. Parameter validation on `setMass`/`setMaxAcceleration`.

## geometric_attitude_control_node

7. **Odometry staleness watchdog** (`odom_timeout`, default 0.3 s): if
   odometry stops, the node stops publishing commands (letting the
   PX4 offboard-loss failsafe engage) and resets integrals.
8. **Setpoint-gap handling** (`setpoint_timeout`, default 1.0 s): after a
   gap in the setpoint stream the integrals reset and dt restarts cleanly.
9. Integrals also reset while disarmed (was: simply not accumulated).
10. **param_callback completed**: `gains.ki.*`, `drag.kd.*`,
    `max_pos_int`, `enable_rate_feedforward` are now live-settable;
    all gain-like parameters validated (finite, non-negative; mass/tau
    strictly positive) with proper rejection reasons; fixed `yaw_gain`
    printed with `%d`.

## geometric_mavros_node

11. **Command-timeout failsafe fixed** — on SE3 command timeout the node
    used to re-publish the stale command indefinitely, feeding PX4 fresh
    setpoints and *preventing* its offboard-loss failsafe. Now: hold a
    level, zero-rate, hover-thrust setpoint for
    `cmd_timeout_hold_duration` (default 1 s) to bridge brief dropouts,
    then stop publishing so PX4's failsafe takes over.
12. **Online thrust-scale estimator** (`enable_thrust_estimator`, needs
    `mass` param): compares IMU body-z specific force (=T/m) against the
    commanded thrust to estimate the true thrust-map scale (battery sag,
    prop wear, wrong `max_thrust`). Heavily gated (armed, mid-envelope
    throttle, body rates < 1 rad/s, sane accel), slow (first-order,
    `thrust_est_tau` default 15 s), hard-clamped
    (`thrust_est_min/max`, default [0.8, 1.25]). The estimate corrects
    the normalized throttle and is published on
    `geometric_mavros/thrust_scale_estimate` for logging.

## Migration notes

- `calculateControl` signature changed (drop `ki_b`, add `dt`) — external
  callers must update. ROS interfaces are backward compatible.
- Set `mass` in geometric_mavros.yaml to enable the thrust estimator
  (recommended); leave `enable_thrust_estimator: false` for the first
  flights after gain tuning if you want the exact tuned plant.
- `ki` is now safe to use: start with `ki: 0.3–1.5` on z after PD tuning,
  `max_pos_int` sized so `max_pos_int` [m/s²] covers expected disturbance.
