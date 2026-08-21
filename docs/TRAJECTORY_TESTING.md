# Trajectory Testing Guide

Easy steps to test the geometric controller with the `trajectory_test_node`.

## What this node does

The node sends trajectory setpoints to the geometric controller. It can fly:

- **setpoint** — go to one point and stay there.
- **circle** — fly a circle.
- **lemniscate** — fly a figure-8.

It is built to be safe:

- It never sends a "jump" setpoint. It always flies smoothly from where the
  drone is now to the trajectory, using a gentle transition move.
- It checks that the trajectory is flyable **before** starting. If your
  requested speed is too fast for the circle/figure-8 size, it slows down
  automatically and tells you in the log.
- It only flies while the drone is in OFFBOARD mode, armed, and receiving
  odometry. If any of these is lost, it stops and holds position.
- It has a geofence. If the plan (or the drone) goes outside the fence, it
  refuses to start (or stops and holds).
- Nothing happens until **you** call the start service (unless you enable
  `auto_start` for simulation).

## The two commands you will use

Start the trajectory (plan + fly):

```bash
ros2 service call /interceptor/trajectory_test/start std_srvs/srv/Trigger
```

Stop (slow down smoothly, then hold position):

```bash
ros2 service call /interceptor/trajectory_test/stop std_srvs/srv/Trigger
```

(Remove `/interceptor` if you run without a namespace.)

The start call replies with a summary, for example:

```
circle: peak speed 2.00 m/s (requested 2.00), peak accel 1.33 m/s^2, ...
```

If the plan is unsafe (geofence, bad parameters), start is refused and the
reply tells you why.

## Test in SITL (simulation)

All commands run inside the `d2dtracker_cuda` container
(`source /opt/ros/humble/setup.bash`, `source ~/shared_volume/ros2_ws/install/setup.bash`,
`export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`).

1. **Start the simulation with the controller:**

   ```bash
   ros2 launch d2dtracker_sim sitl_bringup.launch.py with_controller:=true
   ```

2. **Start the trajectory node** (in a second terminal). `auto_start:=true`
   means it starts flying by itself once the drone is in OFFBOARD:

   ```bash
   ros2 launch mav_controllers_ros trajectory_test.launch.py \
       controller_ns:=interceptor trajectory_type:=circle auto_start:=true
   ```

3. **Take off and switch to OFFBOARD** the way you normally do in your SITL
   setup (e.g. via QGroundControl or your bringup scripts). As soon as the
   drone is armed + OFFBOARD, the node flies to the circle and starts
   orbiting.

4. **Watch it in RViz.** Add `rviz:=true` to the launch command above and
   RViz opens with a ready-made layout (see "Seeing tracking quality in
   RViz" below). The current phase is also published on
   `/interceptor/trajectory_test/status` (HOLD → GOTO → TRACK → STOPPING).

5. **Stop** with the stop service, or just switch out of OFFBOARD — the node
   aborts to hold automatically.

Try the other shapes by changing one argument:

```bash
trajectory_type:=lemniscate    # figure-8
trajectory_type:=setpoint      # single point
```

## Test in the field (real drone)

Field rule: **keep `auto_start` false** (that is the default). The drone does
nothing until you call the start service.

1. **Before flight**, open `config/trajectory_test.yaml` and check:
   - `trajectory_type` — what you want to fly.
   - `circle.radius` or `lemniscate.width` — size of the shape.
   - `speed` — start small (1–2 m/s) for the first flights.
   - `geofence.max_radius_xy`, `geofence.min_z`, `geofence.max_z` — make the
     fence fit your test area. The fence is measured from the point where you
     call start.
   - `relative_to_start: true` (default) — the shape is placed around the
     drone's position at the moment you call start. This is the safe choice:
     the trajectory is always near the drone.

2. **Launch the controller stack and this node** (with your namespace):

   ```bash
   ros2 launch mav_controllers_ros trajectory_test.launch.py controller_ns:=<ns>
   ```

3. **Take off and hover** at your test altitude, then switch to OFFBOARD.
   The node is streaming "hold here" setpoints, so the drone just holds.

4. **Start the test:**

   ```bash
   ros2 service call /<ns>/trajectory_test/start std_srvs/srv/Trigger
   ```

   Read the reply. It tells you the real peak speed and acceleration it will
   fly. If you don't like it, switch out of OFFBOARD (nothing is latched) or
   call stop.

   The drone will: fly gently to the nearest point of the shape → speed up
   slowly → fly the shape at constant speed.

5. **Stop the test:**

   ```bash
   ros2 service call /<ns>/trajectory_test/stop std_srvs/srv/Trigger
   ```

   The drone slows down smoothly and holds position. You can now start again
   (maybe with a higher `speed`) or land.

6. **If anything looks wrong:** switch out of OFFBOARD (e.g. to Position
   mode). The node immediately stops the trajectory. It will NOT restart by
   itself when you go back to OFFBOARD — it holds until you call start again.

## Making it more agile (step by step)

To push the controller harder, raise `speed` between runs (call stop, change
the parameter, call start again):

```bash
ros2 param set /<ns>/trajectory_test_node speed 4.0
```

The node will never exceed `limits.max_accel` and `limits.max_jerk` — if the
shape is too small for the speed you asked, it derates and warns:

```
Requested speed 6.00 m/s derated to 3.34 m/s by accel/jerk limits ...
Increase radius/width for more speed.
```

So for high speed tests, use a bigger `circle.radius` / `lemniscate.width`,
or (carefully) raise the limits. Keep `limits.max_accel` below the
controller's `max_accel` (10 m/s²).

To judge tracking quality, record and plot:

```bash
ros2 bag record /<ns>/geometric_controller/setpoint \
                /<ns>/mavros/local_position/odom \
                /<ns>/geometric_controller/control_errors
```

## Seeing tracking quality in RViz

Launch with RViz included:

```bash
ros2 launch mav_controllers_ros trajectory_test.launch.py \
    controller_ns:=interceptor trajectory_type:=circle rviz:=true
```

You will see:

- **Green line** — the planned path (published once when you call start).
- **Yellow line** — the reference the controller is being asked to follow,
  drawn as the test runs.
- **Red line** — where the drone actually flew.
- **Blue arrow** — the current setpoint, pointing in the commanded yaw.

Good tracking = the red line sits on top of the yellow line. The gap between
them is your tracking error. Both traces are cleared automatically each time
you call start, so every run gives a clean picture.

The topics (all under your namespace) are `trajectory_test/planned_path`,
`trajectory_test/reference_path`, `trajectory_test/actual_path`, and
`trajectory_test/setpoint_pose` — you can also add them to your own RViz
setup. For numbers instead of pictures, record a bag (see above) or plot
`geometric_controller/control_errors`.

## What happens if the trajectory node dies

If the trajectory node crashes or you Ctrl+C it while flying, the
controller notices the setpoint stream stopped (after `setpoint_timeout`,
1 s) and **holds position at the drone's current pose by itself**. It stays
in that hold until it receives new setpoints — for example when you launch
the trajectory node again (which always starts by streaming "hold here"
setpoints, so the handover is smooth).

This is the controller parameter `hold_on_setpoint_timeout: true` in
`config/geometric_controller.yaml` (on by default). If you turn it off, the
old behavior applies: the controller goes silent after ~1 s and PX4's
offboard-loss failsafe takes over (whatever `COM_OBL_ACT` is set to). Keep
it on for field tests.

As always, the pilot switching out of OFFBOARD overrides everything.

## Yaw options (`yaw_mode` in the yaml)

- `tangent` (default) — nose points where the drone is flying. Most demanding.
- `center` — nose points at the middle of the shape (good with a gimbal target).
- `hold` — keep the heading it had at start.
- `fixed` — use `setpoint.yaw`.

Yaw is always rate-limited by `limits.max_yaw_rate`.

## Quick troubleshooting

| Symptom | Cause / fix |
|---|---|
| Start replies "not in OFFBOARD mode" | Switch to OFFBOARD first, then call start. |
| Start replies "planned path violates geofence" | Shape too big or too far for the fence. Shrink the shape or grow the fence in the yaml. |
| Log says speed was "derated" | Shape too small for the requested speed. Bigger radius/width, or accept the lower speed. |
| Node prints "Waiting for odometry" | It is not receiving `mavros/local_position/odom` — check mavros and the namespace. |
| Drone holds instead of flying the shape | A safety abort happened (mode change, geofence, odom loss). Check the node's log, fix the cause, call start again. |
