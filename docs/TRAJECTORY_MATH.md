# How `trajectory_test_node` Works — Method and Math

This document explains *how* the trajectory test node
([test/trajectory_test_node.cpp](../test/trajectory_test_node.cpp)) builds the
setpoints it sends to the geometric controller, and the math behind each piece.
For *how to run it*, see [TRAJECTORY_TESTING.md](TRAJECTORY_TESTING.md).

---

## 1. The big picture

The geometric controller does not want a position setpoint. It wants a full
**reference state**: position, velocity, acceleration, jerk, yaw, yaw rate
(`mav_controllers_ros/TargetCommand`). Velocity/acceleration are used as
feedforward, so they must be the *true derivatives* of the position it is
being told to fly. If they disagree, the controller fights its own feedforward.

So the node's job is:

> Produce, at 100 Hz, a reference `(p, v, a, j, yaw, yaw_dot)` that is
> (a) mathematically self-consistent (each term is the derivative of the one
> before it), (b) continuous at every phase change, and (c) inside the
> vehicle's speed / acceleration / jerk / yaw-rate limits.

Everything below is a technique for getting one of those three properties.

The design pattern used throughout is **geometry ⊗ timing**:

```
     shape                      timing law                  reference
  p(θ)  (where to fly)   ×   θ(t)  (how fast along it)  =  p(θ(t)) → v, a, j
```

The shape is a parametric curve of an abstract parameter `θ` (not time). The
timing law says how `θ` advances in time. Chain rule glues them together. This
split is what makes the speed limiting a closed-form calculation instead of a
search.

---

## 2. The building block: the C2 smoothstep

Almost every smooth transition in the node uses one polynomial:

```
h(τ) = 10τ³ − 15τ⁴ + 6τ⁵          for τ ∈ [0, 1]
```

Its derivatives (in the code: `smoothstep`, `smoothstepD1/D2/D3`):

```
h′(τ)   =  30τ² −  60τ³ +  30τ⁴
h″(τ)   =  60τ  − 180τ² + 120τ³
h‴(τ)   =  60   − 360τ  + 360τ²
```

and its integral (`smoothstepInt`):

```
H(τ) = ∫₀^τ h = 2.5τ⁴ − 3τ⁵ + τ⁶ ,     H(0) = 0 ,  H(1) = 0.5
```

**Why this polynomial?** It is the unique quintic satisfying six boundary
conditions:

```
h(0)=0   h′(0)=0   h″(0)=0
h(1)=1   h′(1)=0   h″(1)=0
```

That means: start and end at rest, *and* with zero acceleration at both ends.
Applied to position, it gives continuous position, velocity and acceleration
(C2) with only a finite jerk step — nothing in the reference ever jumps.

It is also the **minimum-jerk** rest-to-rest polynomial: it minimises
∫ (d³p/dt³)² dt subject to those boundary conditions. That is why the doc
comments call the point-to-point moves "min-jerk".

Two more facts the code exploits:

**Its peak values are known constants** (used to size the move duration):

| quantity | peak | at |
|---|---|---|
| `h′` | 1.875 | τ = 0.5 |
| `h″` | 5.7735 | τ = (1 − 1/√3)/2 ≈ 0.211 |
| `h‴` | 60 | τ = 0 and τ = 1 |

**It is antisymmetric about τ = 0.5:** `h(1 − τ) = 1 − h(τ)`. This is what
makes the "stop while still ramping up" case exact — see §7.

---

## 3. Phase machine

```
        ~/start (plan accepted)
  HOLD ─────────────────────────► GOTO ──► TRACK ──► STOPPING ──► HOLD
   ▲                               │  (setpoint type,     │         ▲
   │                               │   or stop during     │         │
   └───────────────────────────────┴── goto) ─────────────┴─────────┘
        disengage / geofence breach / odom loss (any phase)
```

- **HOLD** — publish a frozen reference at the vehicle's current pose
  (`v = a = j = 0`). While not engaged, the hold point is re-glued to the live
  odometry every tick, so at the instant OFFBOARD engages the reference equals
  the current pose: **zero initial position error, no lurch**. This also means
  a setpoint stream always exists, which PX4 requires *before* you can enter
  OFFBOARD.
- **GOTO** — a min-jerk move from where you are to the trajectory entry point.
- **TRACK** — ride the periodic curve at the planned angular rate.
- **STOPPING** — ramp the angular rate back down to zero, then hold.

Every arrow into HOLD is a safety abort (loss of OFFBOARD, disarm, motors
disabled, stale odometry, geofence breach) and requires a fresh `~/start`.

---

## 4. Planning: from parameters to a feasible plan

`planFromCurrentState()` runs **once**, at the `~/start` call, and captures the
vehicle's current position as the plan origin. Nothing about the plan changes
afterwards, so the reference is a pure function of time — the vehicle can never
"chase" a setpoint that runs away from it.

### 4.1 The curves

`curve(θ)` returns the position and its first three derivatives **with respect
to θ**.

**Circle** (radius `r`, centre `c`):

```
p(θ)   = c + ( r cos θ ,  r sin θ , 0 )
p′(θ)  =     (−r sin θ ,  r cos θ , 0 )
p″(θ)  =     (−r cos θ , −r sin θ , 0 )
p‴(θ)  =     ( r sin θ , −r cos θ , 0 )
```

so `|p′| = |p″| = |p‴| = r` everywhere — the circle is the easy case.

**Lemniscate of Gerono** (half-width `A`, the figure-8):

```
p(θ)   = c + ( A sin θ  ,  (A/2) sin 2θ , 0 )
p′(θ)  =     ( A cos θ  ,   A cos 2θ    , 0 )
p″(θ)  =     (−A sin θ  , −2A sin 2θ    , 0 )
p‴(θ)  =     (−A cos θ  , −4A cos 2θ    , 0 )
```

x spans `[−A, A]`, y spans `[−A/2, A/2]`. Unlike the circle, the derivative
magnitudes vary strongly around the curve (peak `|p′| = A√2`, `|p″| = 2A`,
`|p‴| = A√17 ≈ 4.12A`), which is why the crossing point of a figure-8 is the
demanding part of the manoeuvre.

### 4.2 Chain rule: curve derivatives → time derivatives

With `θ = θ(t)` (write `θ̇, θ̈, θ⃛`), differentiating `p(θ(t))`:

```
v = p′ θ̇
a = p″ θ̇²  +  p′ θ̈
j = p‴ θ̇³  +  3 p″ θ̇ θ̈  +  p′ θ⃛
```

This is `curveRef()` verbatim. Because the reference is produced analytically
this way, the feedforward terms are exact — no numerical differentiation, no
lag, no noise.

### 4.3 Closed-form speed derating (the key trick)

At a **constant** angular rate `θ̇ = ω` (the cruise part of TRACK), the terms
with `θ̈` and `θ⃛` vanish, so:

```
|v| = ω  |p′|        (linear in ω)
|a| = ω² |p″|        (quadratic)
|j| = ω³ |p‴|        (cubic)
```

The node samples the curve on a grid of 2000 points over one period and takes

```
m₁ = max |p′| ,  m₂ = max |p″| ,  m₃ = max |p‴|
```

Then each limit inverts directly into an upper bound on `ω`:

```
ω ≤ v_des / m₁          from the speed limit
ω ≤ √( a_max / m₂ )     from the accel limit
ω ≤ ∛( j_max / m₃ )     from the jerk limit

ω = min of the three
```

That is the `omega = min( v/|p′|max, √(a_max/|p″|max), ∛(j_max/|p‴|max) )`
line in the code. No iteration, no trial-and-error: the requested `speed` is
**derated automatically** to whatever the geometry can support, and the log
tells you it happened.

*Worked example:* circle, `r = 3 m`, `speed = 2 m/s`, `a_max = 5`, `j_max = 40`.
`m₁=m₂=m₃=3`, so `ω ≤ min(2/3, √(5/3), ∛(40/3)) = min(0.667, 1.29, 2.37)
= 0.667 rad/s`. Speed is the binding limit; achieved accel is
`ω²r = 1.33 m/s²`. Ask for `speed = 6 m/s` on the same circle and the accel
bound `√(5/3) = 1.29 rad/s` binds instead, capping the speed at `3.87 m/s`.

### 4.4 Ramp margin

During the speed ramp `θ̈ ≠ 0`, so the `p′ θ̈` term adds acceleration on top of
the centripetal `p″ ω²`. The peak of `θ̈` over the ramp is `ω h′(0.5)/T_ramp =
1.875 ω / T_ramp`. The node checks the (conservative, worst-case-aligned) sum

```
m₂ ω²  +  m₁ ω · 1.875 / T_ramp   ≤  a_max
```

and if it fails, shrinks `ω` again:
`ω ← √( (a_max − ramp_extra) / m₂ )`. Lengthening `ramp_time` therefore buys
back cruise speed.

### 4.5 Entry point

The same 2000-point scan records the θ that minimises `|p(θ) − origin|`. The
trajectory is entered at that **nearest point on the curve**, `θ₀`, so the GOTO
transit is as short as possible and the vehicle joins the curve tangentially
rather than cutting across it.

### 4.6 Geofence pre-check

Before accepting the plan, the GOTO target and 400 samples of the whole curve
are tested against a cylinder centred on the plan origin:

```
√((x−x₀)² + (y−y₀)²) ≤ max_radius_xy      and      z_min ≤ z ≤ z_max
```

A single violating sample rejects the plan and the service reply says why —
you learn about the problem on the ground, not in the air. A separate runtime
check applies the same test to the *actual* odometry every tick while a
trajectory is active, and aborts to HOLD on breach.

---

## 5. GOTO: minimum-jerk point-to-point

Given start `p₀`, target `p₁`, `D = p₁ − p₀`, and duration `T`, with
`τ = clamp(t/T, 0, 1)`:

```
p(t) = p₀ + D·h(τ)
v(t) =      D·h′(τ)  / T
a(t) =      D·h″(τ)  / T²
j(t) =      D·h‴(τ)  / T³
yaw  = yaw₀ + Δψ·h(τ) ,   yaw_dot = Δψ·h′(τ) / T      (Δψ = wrapPi(ψ₁ − ψ₀))
```

A straight line in space, traversed on a minimum-jerk time profile: starts and
ends at rest with zero acceleration, so it splices cleanly onto HOLD at both
ends.

**Choosing `T`.** Rather than picking a speed and integrating, invert the known
peaks of §2. Peak speed is `1.875‖D‖/T`, peak accel `5.7735‖D‖/T²`, peak jerk
`60‖D‖/T³`. Requiring each ≤ its limit gives a lower bound on `T`, and the node
takes the largest:

```
T = max( 1.0 s ,
         1.875 ‖D‖ / goto_speed ,
         √( 5.7735 ‖D‖ / goto_accel ) ,
         ∛( 60 ‖D‖ / j_max ) ,
         |Δψ| / (0.5 · yaw_rate_max) )
```

The 1 s floor keeps tiny moves from becoming impulsive. The yaw term is
slightly conservative (it implies a peak yaw rate of `1.875/2 ≈ 0.94` of the
limit), leaving headroom for the yaw limiter of §8.

For `trajectory_type: setpoint` the run *is* this move: GOTO to the target,
then HOLD there forever.

---

## 6. TRACK: the θ timing law

TRACK needs `θ` to ramp smoothly from rest up to the cruise rate `ω`, then stay
there. Using `h` as the **rate** profile (not the position profile) and `T =
ramp_time`, `τ = t/T`, with `dir = ±1` for direction of travel:

```
t < T :   θ̇  = dir·ω·h(τ)
          θ   = dir·ω·T·H(τ)          (integral of the rate — hence smoothstepInt)
          θ̈  = dir·ω·h′(τ) / T
          θ⃛ = dir·ω·h″(τ) / T²

t ≥ T :   θ   = dir·ω·(t − T/2) ,   θ̇ = dir·ω ,   θ̈ = θ⃛ = 0
```

The `−T/2` offset is exactly `H(1) = 0.5`: at `t = T` the ramp branch gives
`θ = dir·ω·T·0.5`, and the cruise branch gives `dir·ω·(T − T/2)` — the same
value. **Position, velocity and acceleration are continuous across the
ramp/cruise seam by construction**, not by luck.

The actual reference is then `curveRef(θ₀ + θ, θ̇, θ̈, θ⃛)` — geometry from §4.1,
chain rule from §4.2.

---

## 7. STOPPING: ramping down without a velocity jump

The mirror-image law, starting from some absolute curve parameter `θ_s`:

```
θ̇ = dir·ω·(1 − h(τ))
θ  = θ_s + dir·ω·T·(τ − H(τ))
θ̈ = −dir·ω·h′(τ) / T
θ⃛ = −dir·ω·h″(τ) / T²
```

Rate goes `ω → 0` over `ramp_time`, smoothly at both ends.

**The subtle case: `~/stop` arrives while still ramping *up*.** At that moment
the angular rate is `ω·h(τ_u)`, not `ω`. Entering the down-ramp at `τ = 0`
would command an instantaneous jump up to `ω` — a velocity discontinuity the
controller would see as a large tracking error.

The fix uses the antisymmetry `h(1 − τ) = 1 − h(τ)`. We need the down-ramp
entry time `τ_s` where the rates match:

```
ω(1 − h(τ_s)) = ω·h(τ_u)   ⟹   h(τ_s) = 1 − h(τ_u) = h(1 − τ_u)   ⟹   τ_s = 1 − τ_u
```

So the node starts STOPPING at `t = (1 − τ_u)·T` (and at `τ_s = 0` if already
cruising). The offset `θ_s` is then back-solved so position is continuous too:

```
θ_s = θ_current − dir·ω·T·(τ_s − H(τ_s))
```

Result: velocity matches, position matches, and the deceleration profile is the
exact time-reverse of however much of the ramp-up had been completed.

---

## 8. Yaw

Four modes, each producing an **analytic** `yaw_dot` so the feedforward matches
the commanded yaw:

**`tangent`** — nose along the direction of travel. With `d1 = p′`:

```
ψ = atan2(sgn·d1_y , sgn·d1_x)                      (sgn = travel direction)

d/dt atan2(d1_y, d1_x) = (d1_x d2_y − d1_y d2_x) / (d1_x² + d1_y²) · θ̇
```

The numerator is the 2-D cross product `p′ × p″` — the standard signed-
curvature numerator. So `ψ̇ = κ·speed`, expressed in curve parameters.

**`center`** — nose at the orbit centre. With `d = c − p`:

```
ψ = atan2(d_y , d_x) ,     ψ̇ = (v_x d_y − v_y d_x) / (d_x² + d_y²)
```

(the sign follows from `ḋ = −v`). Denominators are guarded by `1e−9` so the
degenerate "sitting on the centre" case cannot blow up.

**`fixed`** — hold `setpoint.yaw`. **`hold`** — hold the yaw captured at the
start trigger. Both give `ψ̇ = 0`.

**Final rate limiter** (in `publishRef`, applied to every mode). Per tick the
commanded yaw may move at most `yaw_rate_max · dt`:

```
Δψ = wrapPi(ψ_cmd − ψ_last)
if |Δψ| > yaw_rate_max·dt :
    ψ_cmd     = wrapPi(ψ_last + clamp(Δψ, ±yaw_rate_max·dt))
    yaw_dot   = ±yaw_rate_max               (sign of Δψ)
```

The point of also rewriting `yaw_dot` is consistency: when the limiter is
saturating, the published rate is the rate actually being flown. `wrapPi` keeps
every yaw difference in `[−π, π]`, so crossing ±π never causes a "long way
round" spin.

---

## 9. Why the result is safe by construction

| Property | Mechanism |
|---|---|
| No setpoint jump on engage | HOLD reference tracks live odometry until engaged |
| No jump entering a trajectory | GOTO min-jerk move to the *nearest* curve point |
| Continuity at ramp seams | `H(1) = 0.5` offset (§6); rate-matched stop entry (§7) |
| Speed / accel / jerk respected | closed-form ω derating from `max|p′|, max|p″|, max|p‴|` (§4.3) |
| Feedforward consistent with position | all derivatives analytic via the chain rule (§4.2) |
| Yaw rate respected | analytic ψ̇ + per-tick limiter that rewrites `yaw_dot` (§8) |
| Stays in the box | whole-path geofence pre-check + per-tick runtime check (§4.6) |
| Fails safe | any disengage/odom loss/breach → HOLD, requires new `~/start` |

## 10. Known conservatisms and limitations

These are deliberate trade-offs, worth knowing when tuning:

- **Decoupled maxima.** `m₁, m₂, m₃` are maxima taken *independently* over the
  curve; they generally do not occur at the same θ. The resulting ω is
  therefore a safe under-estimate, not the tightest possible one.
- **Ramp margin is a scalar sum.** §4.4 adds `|p″|ω²` and `|p′||θ̈|` as if the
  two vectors were parallel. They are not, so again the bound is conservative.
- **Grid resolution.** Maxima and the nearest entry point come from a
  2000-point scan (400 for the geofence). Fine for these smooth curves, but it
  is sampling, not a proof.
- **Planar curves.** Both periodic shapes are flat, at constant `z`. Altitude
  changes only happen during GOTO.
- **Open loop in time.** The reference is a function of time from the start
  trigger; it does not slow down if the vehicle falls behind. Tracking error is
  the controller's problem — which is exactly what makes this a controller
  *test*.
