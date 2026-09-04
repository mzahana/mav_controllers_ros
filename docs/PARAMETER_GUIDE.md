# Parameter guide — geometric attitude controller

How to set every parameter in [`config/geometric_controller.yaml`](../config/geometric_controller.yaml)
and [`config/geometric_mavros.yaml`](../config/geometric_mavros.yaml), and why.

The shipped defaults are tuned for the **PX4 SITL x500-v2 model (2.0 kg)**.
They are a working starting point, not a universal default — anything with a
different mass or powertrain needs at minimum a new `max_thrust`, and in
practice a new gain set.

For the systematic way to produce that gain set — offline design from one
hover log, plus safe in-flight auto-tuning — use
[geo_tuner](https://github.com/mzahana/geo_tuner). This guide explains what
each parameter *means*; geo_tuner computes the numbers.

---

## 1. The one rule that catches everyone

**`max_thrust` scales every gain in the controller.**

The controller computes a desired force in Newtons. The MAVROS node converts
it to the normalized `[0..1]` throttle PX4 wants by dividing by `max_thrust`:

```
throttle = force_N / max_thrust
```

Halve `max_thrust` and every commanded acceleration doubles. A gain set is
therefore only valid *together with* the `max_thrust` it was tuned at. These
two files are a matched pair — never copy gains from one airframe to another
without carrying `max_thrust` across too, or rescaling.

This is also why the two shipped files cross-reference each other in their
headers.

---

## 2. Order of operations

Tune from the inside out. Each step assumes the ones before it are done.

| # | Step | Why it must come first |
|---|---|---|
| 1 | **PX4 rate + attitude autotune**, in Position mode | The geometric controller commands body rates. If PX4's rate loop is soft or oscillatory, no outer-loop gain will fix it. |
| 2 | **`mass`** (both files) | Feeds the force→throttle map and the thrust estimator. |
| 3 | **`max_thrust`** from a hover log | Sets the scale of all gains. Must precede gain design. |
| 4 | **`attctrl_tau`** | Caps how fast the position loop may be (time-scale separation). |
| 5 | **`gains.pos` / `gains.vel`** | The actual position-loop tuning. |
| 6 | **`gains.ki`** | Only after the PD loop is stable. Trims steady-state offset. |
| 7 | **Limits and watchdogs** | `max_tilt_angle`, `max_accel`, timeouts. |

---

## 3. `max_thrust` — measure it, don't guess

`max_thrust` is the **total** collective thrust of all motors, in Newtons, at
flight voltage. Get it from one steady hover:

```
max_thrust = mass * 9.81 / hover_throttle
```

where `hover_throttle` is the normalized thrust PX4 logged while hovering on a
full flight battery.

Fly a 1–2 minute steady hover in **Position mode**, pull the `.ulg`, then:

```bash
python -m geo_tuner.cli.analyze_hover flight.ulg --mass 2.0 --design
```

This prints the measured hover throttle and `max_thrust`, and with `--design`
writes ready-to-use `geometric_controller.yaml` and `geometric_mavros.yaml`.

Sanity checks:

- Hover throttle **> 0.6** means very little thrust headroom — the vehicle is
  heavy for its powertrain, and aggressive tracking will saturate.
- Hover throttle **< 0.3** means the gains will be very sensitive; double-check
  the mass.
- A bench thrust-stand number is a poor substitute: it ignores voltage sag and
  prop/ESC installation losses. The hover-log number is the one that matters.

Leave the online estimator (§7) on to absorb the residual error.

---

## 4. Position gains — `gains.pos`, `gains.vel`

The position loop is feedback-linearized:

```
a_fb = kx * e_pos + kv * e_vel + integral
```

so each axis is a double integrator under PD control, and the gains are
**physical quantities**, not arbitrary knobs:

```
kx = wn^2         [1/s^2]    gains.pos — stiffness  = natural frequency squared
kv = 2 * zeta * wn [1/s]     gains.vel — damping
```

Design in terms of `(wn, zeta)` and convert, rather than twiddling `kx`/`kv`:

- **`zeta`** (damping): use **0.9–1.0**. Near-critical. Below ~0.7 you get
  visible overshoot; interception work wants no overshoot.
- **`wn`** (bandwidth, rad/s): as high as the two hard caps allow —

  | Cap | Formula | Meaning |
  |---|---|---|
  | Time-scale separation | `wn <= (2/attctrl_tau) / 4` | The position loop must be ≥4× slower than the attitude loop it commands. |
  | Latency margin | `wn <= 0.35 / latency` | Total EKF + MAVROS + offboard round trip, typically 0.05–0.10 s. |

  Take the **smaller** of the two. With `attctrl_tau = 0.3` and 80 ms latency:
  `min(1.67, 4.4) = 1.67 rad/s`, so the attitude loop binds.

- **z may be stiffer than x/y** (roughly `wn_z ≈ 1.25 * wn_xy`): thrust
  authority is direct on the vertical axis, whereas horizontal acceleration
  has to be produced by tilting first.

To compute a set directly:

```bash
python -m geo_tuner.cli.design_gains \
    --mass 2.0 --hover-throttle 0.45 \
    --attctrl-tau 0.3 --zeta 0.95 --latency 0.08 --out-dir cfg/
```

It reports which cap binds. **Raise `wn` gradually** — increase by no more than
~30% per flight and watch for oscillation.

Working backwards from an existing config: `wn = sqrt(kx)`, `zeta = kv / (2*wn)`.
For the shipped x500-v2 defaults that is `wn ≈ 1.06 / 1.33 / 1.66 rad/s` and
`zeta ≈ 0.84 / 0.73 / 0.79` per axis. They are asymmetric in x and y because
they came out of the in-flight auto-tuner, which identifies each axis
separately, rather than the symmetric offline designer.

---

## 5. Integral term — `gains.ki`

```
integral += ki * e_pos * dt          [ki] = 1/s^3
```

Corrects steady-state offset: mass error, thrust-map error, and wind. Note it
is time-correct (multiplied by `dt`), so the same `ki` behaves the same at any
loop rate.

Guidance:

- **Start at zero on all axes.** Tune the PD loop first.
- **z is the axis that usually needs it** — a residual `max_thrust` error shows
  up directly as a constant altitude offset. The shipped default is
  `ki.z = 1.0`, `ki.x = ki.y = 0.0`.
- Keep `ki` well below `kx`. It should trim slow bias, not chase the setpoint.
- If you need large `ki.z` to hold altitude, your `max_thrust` is wrong — fix
  that instead (or let the estimator in §7 do it).

Windup is handled for you: integration is skipped whenever the previous cycle
saturated (tilt or acceleration clamp), and the accumulated term is clamped to
`max_pos_int`.

**`max_pos_int`** clamps the integral contribution in **m/s²** of
acceleration. `1.0` (≈10% of a 1 g budget) is a sane ceiling. Raising it past
~2.0 lets a stuck integrator command significant tilt.

---

## 6. Attitude loop — `attctrl_tau`, `yawctrl_tau`

```
body_rate = (2 / attctrl_tau) * attitude_error
```

**`attctrl_tau`** [s] is the attitude time constant; attitude bandwidth is
`2/tau` rad/s. Smaller = snappier and noisier.

- **0.2–0.4** is the useful range. The shipped default is `0.3`.
- Must be **> 0** — the node rejects zero or negative.
- It sets the ceiling on position bandwidth (§4). Lowering `attctrl_tau` is
  how you earn the right to raise `wn` — but only if PX4's rate loop can
  actually deliver it. Autotune first.

**`yawctrl_tau`** [s] does the same for yaw alone. Yaw authority on a quad is
much weaker than roll/pitch (it comes from motor drag torque, not thrust
differential), so a single `tau` for all three axes either wastes roll/pitch
performance or saturates yaw.

- **`0.0` means "use `attctrl_tau` for yaw too"** — the original coupled
  behavior.
- Set it **larger (slower)** than `attctrl_tau`. The shipped default is
  `1.103`, i.e. yaw is ~3.7× slower than roll/pitch.
- Symptom that you need it: yaw rate command pegged, or yaw oscillation while
  roll/pitch track cleanly.

---

## 7. Thrust-scale estimator (`geometric_mavros.yaml`)

Corrects the thrust map **in flight** — battery sag over a pack, and whatever
`max_thrust` error survived §3. It compares commanded throttle against IMU
body-z specific force and filters the ratio.

| Parameter | Default | Meaning |
|---|---|---|
| `enable_thrust_estimator` | `true` | Master switch. |
| `mass` | `2.0` | **Required.** With `mass <= 0` the estimator disables itself and logs a warning. |
| `thrust_est_tau` | `15.0` | Filter time constant [s]. Long on purpose — this tracks battery sag, not maneuvers. Below ~5 s it starts fighting the controller. |
| `thrust_est_min` | `0.8` | Lower clamp on the scale factor. |
| `thrust_est_max` | `1.25` | Upper clamp. |

The clamps bound how much authority the estimator has. Widening them lets a
bad IMU or a bad `mass` do real damage; keep them near the defaults and fix
`max_thrust` instead. Watch the published estimate — if it sits pinned at a
clamp, `max_thrust` is wrong by more than the estimator is allowed to correct.

`mass` appears in **both** files and must match.

---

## 8. Limits

| Parameter | Default | Units | Notes |
|---|---|---|---|
| `max_tilt_angle` | `0.6853` | rad | Caps commanded tilt (~39°). Bounds horizontal accel to `g*tan(tilt)`. Lower it (0.3–0.5) for early flight tests. Hitting it holds the integrator. |
| `max_accel` | `10.0` | m/s² | Caps the **feedback** acceleration. Saturation is **altitude-priority**: the vertical component is preserved and horizontal is shed first, so a saturated vehicle keeps its height. |
| `max_pos_int` | `1.0` | m/s² | Integral clamp, see §5. |

Set `max_tilt_angle` from what the airframe can survive, then check §4's note
about whether a 1 m step will hit it.

---

## 9. Setpoint and yaw behavior

- **`use_external_yaw`** (`true`): follow the yaw in the setpoint message.
  Set `false` to make yaw track the direction of travel instead.
- **`enable_rate_feedforward`** (`true`): feed reference angular velocity,
  derived from setpoint jerk via differential flatness, into the rate command
  alongside the attitude-error term. Materially improves tracking on
  aggressive trajectories. Turn it **off** if your setpoint source publishes
  noisy or absent jerk — feedforward amplifies noise. The contribution is
  internally bounded to ±3 rad/s.

---

## 10. Watchdogs and failsafes

Safety-relevant. Understand these before flying.

**`geometric_controller.yaml`:**

| Parameter | Default | Behavior |
|---|---|---|
| `odom_timeout` | `0.3` s | Odometry older than this → stop commanding. Set above your worst-case odometry gap, or you will get spurious dropouts. |
| `setpoint_timeout` | `1.0` s | Setpoint stream considered lost after this. |
| `hold_on_setpoint_timeout` | `true` | On setpoint loss, **hold position at the current pose** (closed loop) instead of going silent. This is what saves you when a planner node crashes mid-flight. A new setpoint releases the hold. |

**`geometric_mavros.yaml`:**

| Parameter | Default | Behavior |
|---|---|---|
| `se3_cmd_timeout` | `0.25` s | No SE3 command for this long → command timed out. |
| `cmd_timeout_hold_duration` | `1.0` s | After that timeout, hold **level attitude at hover throttle** for this long, then stop publishing entirely and let the PX4 failsafe take over. Bridges brief controller hiccups without fighting PX4 over a real failure. |

Hover throttle during that hold is computed from `mass` and `max_thrust` — a
third reason to get both right.

---

## 11. Parameters with no effect

Declared so old configs still load, but **ignored** by the control law:

| Parameter | Status |
|---|---|
| `gains.kib.*` | Deprecated. The body-frame integral was never implemented. The node logs a warning if you set it nonzero. |
| `mas_pos_int_b` | Deprecated (note the typo in the name). Paired with `kib`. |
| `yaw_gain` | Stored but never read by `GeometricAttitudeControl`. Yaw bandwidth is controlled by `yawctrl_tau` (§6). |
| `num_props`, `kf`, `lin_cof_a`, `lin_int_b` | Unused remnants of an older thrust model. `max_thrust` is the only thrust parameter that matters. |

Setting any of these will not change how the vehicle flies. If you are chasing
a yaw problem, `yaw_gain` is not the knob — `yawctrl_tau` is.

---

## 12. Live tuning

All gain-like parameters are settable at runtime via `ros2 param set`, with
validation (non-negative and finite; `attctrl_tau` and `mass` must be > 0).
Rejected values are logged and the previous value is kept.

```bash
ros2 param set /geometric_controller_node gains.pos.z 3.0
```

This is what makes in-flight auto-tuning possible.

---

## 13. Symptom → parameter

| Symptom | Look at |
|---|---|
| Slow, sluggish tracking; large lag behind setpoint | `gains.pos` too low — raise `wn` (§4) |
| Overshoot / oscillation at ~1 Hz (position loop) | `gains.vel` too low for the `gains.pos` — raise `zeta` toward 1.0 |
| Fast oscillation, buzzing, hot motors | `attctrl_tau` too small, or PX4 rate loop untuned — autotune first |
| Constant altitude offset | `max_thrust` wrong (§3); then `ki.z` (§5) |
| Altitude offset that drifts over a battery | Enable the thrust estimator (§7) |
| Yaw oscillates but roll/pitch are clean | Raise `yawctrl_tau` (§6) |
| Drifts downwind in steady wind | `ki.x` / `ki.y`, small |
| Loses height in aggressive turns | Expected under `max_accel` saturation — altitude is prioritized; raise `max_accel` only if you have thrust headroom |
| Vehicle stops commanding mid-flight | `odom_timeout` too tight, or odometry actually dropping (§10) |
| Estimator pinned at `thrust_est_min`/`max` | `max_thrust` off by more than the clamp allows (§7) |

---

## See also

- [CHANGES.md](../CHANGES.md) — what the production-hardening work changed
- [docs/TRAJECTORY_TESTING.md](TRAJECTORY_TESTING.md) — flying test trajectories
- [geo_tuner](https://github.com/mzahana/geo_tuner) — gain design and in-flight
  auto-tuning; its `docs/TUNING_GUIDE.md` carries the full derivations, and
  `docs/FIELD_CHECKLIST.md` is the printable field procedure

## Persisting gains changed in flight

A parameter write -- from `ros2 param set`, or from the RViz gain panel on a
laptop -- changes the running node's memory only. It is lost on the next
restart or power cycle. That is deliberate: a bad gain set at 20 m cannot
survive a reboot.

To keep a change, run `gain_saver` **on the vehicle** and call its service:

```bash
ros2 launch mav_controllers_ros gain_saver.launch.py controller_ns:=interceptor
ros2 service call /gain_saver/save std_srvs/srv/SetBool "{data: false}"
```

It reads the live values and writes two override files:

```
<config dir>/geometric_controller.override.yaml
<config dir>/geometric_mavros.override.yaml
```

`<config dir>` is resolved by `launch/config_dir.py`: `$MAV_CONTROLLERS_CONFIG_DIR`
if set, else a shared-volume mount (`~/shared_volume/mav_controllers_config`),
else the mount the workspace itself is installed under, else
`~/.ros/mav_controllers_ros`. On the Jetson the stack runs in a container, so
the default lands on the host-mounted volume -- the container filesystem does
not survive being recreated, and neither the installed package (wiped by
colcon) nor the source tree (must stay clean in git) is an acceptable place to
write.

`geometric_controller.launch.py` and `geometric_to_mavros.launch.py` load those
files **after** the shipped config, so the saved values win. Delete a file to
fall back to the package defaults. Writes are atomic and the previous version is
kept as a timestamped `.bak`, so a save can never leave a config that fails to
parse at the next boot.

`data: true` additionally corrects `max_thrust` by the online thrust-scale
estimate, capturing what the vehicle learned in flight. It is **refused while
armed**: `max_thrust` linearly scales every position gain, so changing it
retunes the whole controller at once.
