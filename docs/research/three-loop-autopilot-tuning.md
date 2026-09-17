# ThreeLoopAutopilot tuning and gain scheduling

Date: 2026-09-16

## What this controller controls

The project's [`ThreeLoopAutopilot`](../../src/rocket_sim/autopilot.py) is an acceleration-demand autopilot patterned after the earlier `sim_rocketpy` controller. Its command is **linear acceleration normal to the rocket body**, not angular acceleration:

```text
Body-FLU command = [forward, left, up] m/s²
```

The pitch test commands body-up acceleration. The yaw test would command body-left acceleration. This matches the standard missile-autopilot purpose: guidance requests normal acceleration, while the autopilot uses fin deflection and body rotation to produce it. MathWorks' authoritative missile example describes the same separation between guidance acceleration demand, a three-loop autopilot, rate-gyro damping, actuator dynamics, and nonlinear airframe dynamics ([Design a Guidance System](https://www.mathworks.com/help/simulink/slref/designing-a-guidance-system-in-matlab-and-simulink.html)).

The three loops in this implementation are:

1. **Acceleration loop:** lateral acceleration PI control produces pitch- and yaw-rate commands.
2. **Rate loop:** pitch/yaw rate error and measured roll rate produce virtual control-axis fin demands.
3. **Actuator loop:** the simulation moves each physical fin toward its target subject to position and rate limits.

This is the terminology used by the local `sim_rocketpy` prototype. In classical missile-control literature, the actuator is normally part of the plant rather than counted as a feedback loop. A classical “three-loop autopilot” may instead count acceleration feedback, rate-gyro feedback, and a synthetic-stability/integrating path. The local class should be understood from its equations: it contains an acceleration PI loop, a rate P loop, and a rate-limited actuator.

The controlled measurement must also be chosen before hardware tuning. The code feeds back `acceleration_body_mps2`, which is inertial acceleration including gravity, rotated into Body-FLU. A physical accelerometer measures **specific force**, exposed separately as `specific_force_body_mps2`. Gains tuned on the current ideal signal will not transfer unchanged to an IMU implementation, especially after the rocket tilts.

The current gains are initial experimental values, not validated flight gains:

```python
k_accel_p = 0.02       # (rad/s) / (m/s²) = s/m
k_accel_i = 0.5        # (rad/s) / (m/s) = 1/m
k_rate_p = 0.08        # fin rad / (body-rate rad/s) = s
k_roll_rate = 0.001    # fin rad / (roll-rate rad/s) = s
```

## Exact equations currently implemented

Let the measured Body-FLU lateral acceleration be `(a_y, a_z)` and its command be `(a_y,c, a_z,c)`.

The acceleration error is:

```text
e_a = [a_y,c - a_y, a_z,c - a_z]
```

The controller integrates this error at the configured controller period, currently 0.01 s at 100 Hz:

```text
I_a[k] = clamp(I_a[k-1] + e_a[k] Δt, -20, +20) m/s
```

The acceleration PI output is a lateral body-rate demand:

```text
r_cmd = clamp(Kap e_y + Kai I_y, -5.5, +5.5) rad/s
q_cmd = clamp(-(Kap e_z + Kai I_z), -5.5, +5.5) rad/s
```

The minus sign in `q_cmd` follows the Body-FLU convention: positive body-up acceleration requires negative pitch rate. Positive body-left acceleration requires positive yaw rate.

The rate loop then computes:

```text
u_roll  = Kroll p
u_pitch = Krate,eff (q - q_cmd)
u_yaw   = Krate,eff (r - r_cmd)
```

This sign is intentional. With aft fins and the project's Body-FLU force convention, positive mixer input creates a negative moment about the corresponding body axis.

The X-fin mixer maps those virtual commands into the four fins:

```text
fin 1 = roll + pitch + yaw
fin 2 = roll - pitch + yaw
fin 3 = roll - pitch - yaw
fin 4 = roll + pitch - yaw
```

When one mixed output exceeds the physical fin limit, all four outputs are scaled by the same factor. This preserves the requested moment direction. When saturation occurs, the acceleration integral update is undone for that control tick, providing simple conditional-integration anti-windup. The physical actuator then applies the YAML rate limit at every 1 ms physics step.

## How gain scheduling is currently implemented

Only the inner pitch/yaw rate gain is scheduled. At each controller call, the code calculates:

```text
q_used = max(q_dynamic, 250 Pa)
scale  = clamp(2030 Pa / q_used, 0.35, 3.0)
Krate,eff = 0.08 s × scale
```

The resulting regions are:

- Below approximately `677 Pa`, scale is fixed at `3.0`, so `Krate,eff = 0.24 s`.
- At the reference condition `2030 Pa`, scale is `1.0`, so `Krate,eff = 0.08 s`.
- Above approximately `5800 Pa`, scale is fixed at `0.35`, so `Krate,eff = 0.028 s`.

The reason for inverse-dynamic-pressure scheduling is the first-order fin relationship:

```text
fin force  ∝ q_dynamic × fin deflection
fin moment ∝ q_dynamic × fin deflection × moment arm
angular acceleration ∝ q_dynamic × fin deflection × moment arm / inertia
```

Without scheduling, the same rate error produces the same deflection at all speeds. Near max-Q that deflection produces a much larger moment than it does just after rail exit. Multiplying the rate gain approximately by `q_reference/q` attempts to keep rate-loop authority and bandwidth similar as dynamic pressure changes.

The clamps serve two purposes:

- the upper clamp prevents extremely large commands when `q` is small and aerodynamic control is weak;
- the lower clamp prevents the inner-loop gain from becoming too small at high `q`.

The controller is also disabled before 0.35 s and below 20 m/s airspeed. While disabled, fin commands and the acceleration integrator are reset to zero. This prevents the acceleration PI loop from winding up while the rail constrains the rocket or before aerodynamic control is useful.

This schedule is only a first approximation. A NASA review of gain-scheduled control describes the standard method as designing gains at a grid of flight conditions, then selecting or interpolating them using variables such as altitude, Mach, dynamic pressure, and rate of climb ([NASA/CR-2015-218702](https://ntrs.nasa.gov/api/citations/20150005863/downloads/20150005863.pdf?attachment=true)). MathWorks' three-loop example similarly trims and linearizes the nonlinear airframe at a grid of incidence and speed conditions, tunes at every point, and makes the gain surfaces vary smoothly ([gain-scheduled three-loop autopilot](https://in.mathworks.com/help/control/ug/tuning-of-gain-scheduled-three-loop-autopilot.html)).

## What the present schedule does not compensate

Dynamic pressure is not the complete control-effectiveness variable. Fin-to-rate response also changes with:

- decreasing mass and inertia during motor burn;
- the changing CG-to-fin CP moment arm;
- Mach-dependent fin lift slope;
- angle of attack and sideslip;
- fin stall and nonlinear effectiveness;
- powered versus coast flow around the aft body;
- actuator bandwidth, rate saturation, backlash, and load-dependent servo speed;
- structural flexibility and sensor filtering.

The current aerodynamic model does not yet represent several of these effects accurately. A more elaborate schedule cannot be more trustworthy than the plant model from which it is tuned.

The outer gains `k_accel_p` and `k_accel_i` are constant. That means the acceleration-loop dynamics can still change significantly even when inverse-q scheduling makes the inner rate loop more uniform. The roll damper is also constant, although roll control effectiveness changes strongly with dynamic pressure.

## Recommended tuning workflow

### 1. Establish measurable requirements

Define separate requirements for pitch and yaw before changing gains:

- command magnitude in m/s²;
- rise time and settling time;
- permitted overshoot and steady-state error;
- maximum body rate and angle of attack;
- maximum fin angle and rate;
- permitted cross-axis acceleration;
- noise amplification;
- minimum control airspeed and usable dynamic-pressure range.

For the current prototype, a sensible initial test is a `±10 m/s²` command. Larger commands may simply be unreachable late in coast because dynamic pressure is falling. Physical saturation must be reported as a command-envelope limit rather than “fixed” by increasing integral gain.

### 2. Validate the sign and mixer before tuning gains

At a frozen representative state, apply a small pulse independently in virtual roll, pitch, and yaw. Confirm:

- positive pitch mixer demand produces negative pitch angular acceleration;
- positive yaw demand produces negative yaw angular acceleration;
- positive roll demand produces negative roll angular acceleration;
- pitch produces negligible roll/yaw, and similarly for the other axes.

Do this at several speeds. If the sign or mixer is wrong, gain tuning can appear to work at one condition while relying on saturation or passive stability.

### 3. Characterize the actuator first

The fin actuator must be faster than the desired body-rate loop. The current actuator has a 600 deg/s rate limit but no servo time constant, delay, hinge-moment limit, or load dependence. Before hardware transfer, measure or specify:

- small-signal bandwidth;
- command-to-position delay;
- rate and position limits;
- response under aerodynamic hinge load;
- backlash/deadband.

Put those dynamics into the simulation before selecting final rate-loop bandwidth. Otherwise the simulated controller can demand reversals that the real servo cannot follow.

Because the current actuator is only a slew limiter, it has no meaningful small-signal bandwidth below that limit: it is effectively instantaneous apart from the 100 Hz zero-order hold. Frequency-domain tuning requires an explicit first- or second-order servo model.

### 4. Tune the inner rate loop first

Set `k_accel_p = k_accel_i = 0` and inject pitch- and yaw-rate commands directly. At one nominal condition near the middle of the useful dynamic-pressure range:

1. Start with small `k_rate_p`.
2. Increase it until the rate response is fast and well damped without sustained oscillation.
3. Check phase lag from the actuator and 100 Hz sample/update delay.
4. Confirm ample distance from fin-angle and fin-rate saturation.
5. Repeat for pitch and yaw separately; do not assume exact equality if inertia or geometry differs.

An authoritative worked example closes the inner `p`, `q`, and `r` loops first, examines deflection-to-rate frequency responses, and selects gains for target crossover bandwidths ([MathWorks angular-rate control example](https://www.mathworks.com/help/control/ug/HL20RateControlExample.html)). The specific published bandwidths apply to that aircraft, not to this rocket; the method is what should be copied.

For this simulation, the controller runs at 100 Hz. A conservative first target is a rate-loop bandwidth well below the 50 Hz Nyquist frequency and below the actuator bandwidth. The exact target must come from the identified airframe and servo response, not a generic fraction alone.

### 5. Build the inner-loop gain schedule

Repeat the rate-loop tuning at representative points covering:

- rail exit / minimum useful airspeed;
- peak thrust;
- burnout;
- max-Q;
- early and late coast;
- expected positive and negative incidence;
- initial and burnout mass/CG/inertia.

At each point, record the tuned pitch, yaw, and roll gains. Fit smooth lookup surfaces. Dynamic pressure will probably dominate below Mach 0.7, but the data should decide whether a second variable such as mass state, airspeed, or angle of attack is needed.

The best near-term improvement over the current equation is scheduling against estimated control effectiveness:

```text
G_control ≈ q S CNa_fin |x_CP - x_CG| / I_transverse
```

Then make the rate gain approximately inversely proportional to `G_control`, with bounded interpolation. This incorporates changing inertia and moment arm rather than using `q` alone.

### 6. Tune acceleration proportional gain with integral disabled

Restore the acceleration loop with `k_accel_i = 0`:

1. Apply small positive and negative acceleration steps.
2. Increase `k_accel_p` until the response is fast enough without excessive rate command, overshoot, or excitation of the inner-loop mode.
3. Verify that the acceleration loop is appreciably slower than the already closed rate loop.
4. Test pitch and yaw independently and with simultaneous demands.

The outer loop must not try to correct acceleration faster than the rate loop can rotate the airframe and build normal force.

### 7. Add integral gain slowly

Increase `k_accel_i` only enough to remove steady acceleration bias caused by trim, asymmetry, wind, or model error. Too much integral gain produces overshoot, slow oscillation, and prolonged saturation after command reversal.

Test specifically:

- zero → positive → negative → zero commands;
- commands at the physical acceleration limit;
- commands during declining coast dynamic pressure;
- controller enable and disable transitions;
- fin position and rate saturation.

The existing “undo this tick's integration when saturated” logic is a reasonable first anti-windup measure, but it is partial. It does not react when the ±5.5 rad/s rate command saturates, does not see actuator slew saturation, and freezes both lateral integrators whenever any mixed fin saturates. A production controller should use directional conditional integration or back-calculation from the realized mixer and actuator output. MathWorks' [anti-windup guidance](https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html) distinguishes conditional integration from back-calculation and supports tracking actual actuator output. The missile example also calls for nonlinear validation with fin position and rate limits ([MathWorks guidance-system example](https://www.mathworks.com/help/simulink/slref/designing-a-guidance-system-in-matlab-and-simulink.html)).

### 8. Tune roll separately

The roll channel currently has proportional rate damping only:

```text
u_roll = k_roll_rate p
```

Command a roll-rate disturbance at several dynamic pressures and increase `k_roll_rate` until roll decays at the desired rate without fin chatter or saturation. It should receive its own schedule because roll moment and roll damping both vary with `q`, and roll inertia is much smaller than pitch/yaw inertia.

### 9. Validate the complete nonlinear system

After point tuning, run the complete 6-DoF model with:

- step, pulse, ramp, and sine-sweep acceleration commands;
- positive and negative pitch/yaw commands;
- simultaneous pitch/yaw/roll demand;
- motor and mass-property uncertainty;
- CG, inertia, fin-alignment, and control-effectiveness uncertainty;
- wind and sensor noise/delay;
- actuator lag, rate limit, and saturation;
- controller timing jitter and lower update rates;
- the full low-Mach and angle-of-attack envelope.

Track rise time, settling time, steady-state error, overshoot, fin-angle margin, fin-rate margin, rate-command saturation, integral state, cross-axis response, and local fin incidence. Validate scheduled gains at the grid points **and between them**. NASA notes that conventional scheduling does not automatically guarantee performance between design points, even when each point design is satisfactory ([NASA/CR-2015-218702](https://ntrs.nasa.gov/api/citations/20150005863/downloads/20150005863.pdf?attachment=true)).

## Assessment of the current values

The checked-in pitch test shows that the present gains can follow a `±10 m/s²` command qualitatively. The regression test measures approximately `+11.5 m/s²` and `-8.3 m/s²` over selected steady windows. That is useful evidence that the loop signs, mixer, integrator, and scheduling operate coherently.

The current acceleration PI zero is `k_accel_i / k_accel_p = 0.5 / 0.02 = 25 rad/s`. That value must be compared with the measured outer-loop crossover; without a linearized or identified plant, there is no evidence that this zero is placed appropriately.

It is not a completed tuning result:

- positive and negative response are asymmetric;
- the response changes across burn and coast;
- the gains were inherited from the educational `sim_rocketpy` experiment rather than re-derived for the YAML rocket;
- acceleration is measured at the simulated reference/CG, while classical missile autopilots may place the accelerometer ahead of the CG, changing sensed rotational acceleration and loop dynamics;
- the aerodynamic and servo models remain insufficiently validated for hardware gains.

The next engineering step should be an automated operating-point sweep that identifies fin-to-rate and rate-to-acceleration responses at fixed flight states, then produces scheduled `k_rate_p`, `k_accel_p`, and `k_accel_i` tables with explicit bandwidth, damping, saturation, and robustness targets.

## Primary and authoritative references

- MathWorks, [Design a Guidance System in MATLAB and Simulink](https://www.mathworks.com/help/simulink/slref/designing-a-guidance-system-in-matlab-and-simulink.html): classical acceleration-demand missile autopilot, rate gyro, gain tables, anti-windup, nonlinear actuator validation.
- MathWorks, [Tuning of Gain-Scheduled Three-Loop Autopilot](https://in.mathworks.com/help/control/ug/tuning-of-gain-scheduled-three-loop-autopilot.html): trim/linearize/tune/reconcile workflow and scheduled gain surfaces.
- MathWorks, [Angular Rate Control in the HL-20 Autopilot](https://www.mathworks.com/help/control/ug/HL20RateControlExample.html): inner-loop-first frequency-response tuning and gain selection.
- NASA, [Advanced Control Law Concepts and Flight Validation for Intelligent Aircraft](https://ntrs.nasa.gov/api/citations/20150005863/downloads/20150005863.pdf?attachment=true), NASA/CR-2015-218702: scheduling variables, operating-point grids, interpolation, and limitations between design points.
- NASA, [Orion Launch Abort Vehicle gain scheduling](https://ntrs.nasa.gov/citations/20110013502): smooth interpolation, linear robustness checks including actuator and latency effects, followed by nonlinear 6-DoF Monte Carlo validation.
