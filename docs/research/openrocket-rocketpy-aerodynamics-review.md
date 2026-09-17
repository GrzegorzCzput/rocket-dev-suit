# Aerodynamics review: OpenRocket, RocketPy, and this simulator

Date: 2026-09-16

## Scope and conclusion

This review compares the current YAML-driven simulator with OpenRocket's `unstable` branch at commit [`fe6ec251`](https://github.com/openrocket/openrocket/tree/fe6ec251b697067e3e2a48b08050a526ec508367) and the current RocketPy `master` implementation and technical documentation. It focuses on a small, slender model rocket below Mach 0.7 and approximately 10 degrees angle of attack.

The current simulator is a sound **6-DoF prototype**, but it is not yet a validated aerodynamic predictor. Its best ideas are explicit reference frames and units, reproducible inputs, forces applied at component centers of pressure, local velocity `V + omega x r`, a full Euler gyroscopic term, changing inertia, and a deliberately narrow validity envelope. Its weakest area is axial aerodynamics: the drag build-up is too incomplete to establish credible apogee, maximum velocity, or max-Q. Its fin model also cannot yet represent the movable fins described by the YAML.

OpenRocket is the strongest of the three at deriving whole-rocket drag and conventional passive stability from geometry. RocketPy has the strongest flight dynamics, atmosphere integration, event handling, and facility for measured or externally calculated coefficient data. A good implementation for this project should combine:

- OpenRocket-like transparent component drag build-up as a fallback;
- RocketPy-like powered/coast coefficient tables and variable-mass dynamics;
- this simulator's local component-flow method, explicit low-angle/Mach validity limits, and inspectable component force ledger.

Neither comparison program is ground truth. OpenRocket documents material errors against tests and uses empirical damping. RocketPy's standard Barrowman surfaces remain linear, and whole-rocket drag accuracy depends on data supplied by the user. The final authority should be measured motor data plus flight, wind-tunnel, or CFD data with uncertainty bounds.

## How OpenRocket simulates aerodynamics

### Stability and normal force

OpenRocket separates stability from drag in [`BarrowmanCalculator`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/BarrowmanCalculator.java). The extended Barrowman path asks a calculator for each active aerodynamic component for its normal-force coefficient, center of pressure, and moments, then merges those contributions in [`BarrowmanStabilityCalculator`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/BarrowmanStabilityCalculator.java).

For conventional rockets it includes more geometry than this project:

- nose cones and transitions contribute Barrowman normal force and CP;
- symmetric body tubes receive a Galejs body-lift correction at incidence;
- fins use a finite-wing lift slope with subsonic compressibility correction;
- fin CP changes with Mach;
- fin-fin and fin-body interference are included;
- the current branch contains a NACA 1307 fin/body carryover model where its geometric assumptions apply;
- fin lift saturates at a fixed 17.5-degree stall angle rather than increasing without limit.

The detailed fin implementation is in [`FinSetCalc`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/barrowman/FinSetCalc.java), and the newer load-transfer model is in [`NACA1307FinBodyInterference`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/barrowman/NACA1307FinBodyInterference.java). Below Mach 0.7, these methods are being used in their most defensible region. They are still engineering correlations, rather than a solution of the viscous flow field.

OpenRocket supports roll forcing from fin cant and roll damping. Its pitch/yaw damping is more empirical: a global damping coefficient is calculated from the body and fin geometry, multiplied by three with a source comment that the larger value gives a more realistic apogee turn, and capped against the static moment. That tuning may improve trajectories, but it is not a clean first-principles derivative.

### Drag

OpenRocket's [`BarrowmanDragCalculator`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/BarrowmanDragCalculator.java) builds total drag from skin-friction, pressure/form, base, and user-override terms.

Its friction model uses a rocket-length Reynolds number, laminar and mixed/turbulent correlations, surface finish/roughness limits, Mach corrections, wetted area, and a fineness correction for cylindrical bodies. Its pressure model treats noses, transitions, fin leading and trailing edges, launch lugs/buttons, and radius discontinuities. Base drag depends on Mach and exposed aft area; current code subtracts known nozzle exit area from the terminal wake area. Total drag is converted to axial force as angle of attack grows.

This is much richer than this project's single flat-plate coefficient plus fixed powered/coast base terms. It is still a semi-empirical build-up. OpenRocket applies the resulting drag as one rocket-level axial force, so its component drag report does not imply that every drag contribution is applied at a separate location or generates an asymmetric moment.

OpenRocket also supports CSV lookup alternatives for drag and stability. [`LookupTableDragCalculator`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/LookupTableDragCalculator.java) interpolates coefficient data over Mach and angle of attack. The stability lookup is not a complete dynamic-derivative model: it supplies no component breakdown, roll model, or damping derivatives.

### Atmosphere, wind, and integration

OpenRocket's [`ExtendedISAModel`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/models/atmosphere/ExtendedISAModel.java) covers the layered atmosphere through 84.852 km and allows launch-site temperature, pressure, and humidity adjustments. Wind can include deterministic seeded pink-noise turbulence and altitude-dependent profiles.

Flight state contains position, velocity, quaternion attitude, and angular velocity. [`RK4SimulationStepper`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/simulation/RK4SimulationStepper.java) uses RK4 with step limits based on events, rail travel, attitude increments, and angular-rate changes. The event engine handles ignition, burnout, staging, deployment, rail clearance, apogee, impact, and tumble detection.

OpenRocket's rotational equation is less rigorous than this project's in one respect: it uses moment divided by inertia and omits the full `omega x (I omega)` and explicit `I_dot omega` terms. Off-axis motor moments remain incomplete. This matters most for high roll rates, strongly asymmetric inertia, or fast mass-property changes.

### OpenRocket's documented accuracy

The official [OpenRocket technical documentation](https://openrocket.info/documentation.html) reports comparisons rather than universal accuracy. Examples include apogee overprediction of about 7–16% for two small-motor flights, roughly half the measured peak roll rate in one roll comparison, and a 16% apogee underprediction for a hybrid rocket. Wind-tunnel comparisons found useful subsonic CP agreement, but lower-than-measured normal-force slope in one dataset and known failures where fin-profile or boattail assumptions did not apply.

Those results support using OpenRocket as an engineering reference inside this project's low-Mach envelope. They do not support treating its output as exact.

## How RocketPy simulates aerodynamics

### Component forces and moments

RocketPy's full path is a 13-state 6-DoF model in [`Flight.u_dot_generalized`](https://github.com/RocketPy-Team/RocketPy/blob/master/rocketpy/simulation/flight.py). It obtains density, pressure, sound speed, viscosity, gravity, and altitude-dependent wind from an `Environment`, transforms relative flow into body coordinates, and evaluates aerodynamic surfaces at their own centers of pressure.

For every surface, RocketPy adds `omega x r` to the local translational velocity. The surface then computes its local normal force and its moment about the vehicle reference. This is the same physically useful mechanism used by the current simulator: a rotating tail sees extra incidence and naturally creates pitch/yaw-rate damping.

The standard [`AeroSurface`](https://github.com/RocketPy-Team/RocketPy/blob/master/rocketpy/rocket/aero_surface/aero_surface.py) model is Barrowman-like. Nose lift slope and CP are derived from nose and rocket geometry. Fin lift slope includes planform, number-of-fins, body interference, airfoil options, and a subsonic Mach correction. [`Fins`](https://github.com/RocketPy-Team/RocketPy/blob/master/rocketpy/rocket/aero_surface/fins/fins.py) also adds cant-induced roll forcing and explicit roll-rate damping.

The standard normal-force law remains linear in angle. It does not model separation, deep stall, nonlinear crossflow, or viscous Reynolds corrections to lift. RocketPy's `GenericSurface` and `LinearGenericSurface` provide the escape hatch: coefficients can depend on angle of attack, sideslip, Mach, Reynolds number, and pitch/yaw/roll rates. That is the suitable route for wind-tunnel or CFD derivatives.

### Drag

RocketPy deliberately does not derive complete rocket drag from geometry. The user supplies separate powered and coast drag coefficient functions, commonly versus Mach and now optionally against more variables. RocketPy recommends deriving accurate curves from measurement, CFD, or tools such as RASAero in its [rocket usage documentation](https://docs.rocketpy.org/en/latest/user/rocket/rocket_usage.html).

This has a clear tradeoff. It avoids pretending that a short geometric formula is a calibrated drag model, but it provides no built-in component attribution and can be very wrong when the supplied curve is generic or poorly normalized. Standard surface lift also adds no induced drag of its own. Rocket-level drag is applied along the body axis rather than exactly opposite the full relative-flow vector, which is a reasonable slender, low-angle approximation and becomes inaccurate at large incidence.

### Variable mass, events, and environment

RocketPy's [equations of motion](https://github.com/RocketPy-Team/RocketPy/blob/master/docs/technical/equations_of_motion.rst) are the most complete of the three. They include time-varying center of mass and its derivatives, a full time-varying inertia tensor and derivative, mass-flow/nozzle coupling, thrust pressure correction, gyroscopic terms, gravity torque about the selected reference, and Earth Coriolis effects.

Integration uses adaptive SciPy solvers, with LSODA as the normal default, and separates rail, free-flight, and parachute phases. Rail exit is reconstructed using dense output and a root solution rather than rounded to the next output step. RocketPy also represents rail-button geometry and exposes rail reaction loads. Its environment accepts standard atmosphere, soundings, forecasts, reanalysis, ensembles, and height-dependent wind.

RocketPy has a broader validation workflow than this project, including comparison of simulated and recorded flights. Claims of sub-percent apogee error should be read as validation of particular rockets and inputs, not proof that every aerodynamic coefficient is sub-percent accurate.

## What the current simulator actually computes

The current force path is in [`physics.py`](../../src/rocket_sim/physics.py).

1. It evaluates a tropospheric ISA model at the world `z` coordinate and subtracts a constant ENU wind vector from vehicle velocity.
2. It transforms relative velocity into the Body-FLU frame and calculates dynamic pressure, Mach, angle of attack, and sideslip.
3. It calculates one total drag coefficient from a flat-plate skin-friction correlation, summed wetted area, authored skin/pressure terms, and a fixed base contribution of 0.04 powered or 0.12 coast. Powered/coast calibration multipliers scale the entire result.
4. It applies drag exactly opposite the relative-flow vector.
5. It creates one normal-force source at the nose CP and one at each whole fin-set CP. Each source samples local velocity using `V + omega x r`, applies the geometry-derived Barrowman slope, and contributes `r x F` about the current CG.
6. It adds axial thrust, transforms total body force to world coordinates, adds gravity, and integrates position, velocity, quaternion, and body rates.
7. On the rail it projects velocity and acceleration onto the rail and freezes attitude. Off the rail it uses diagonal inertia with gyroscopic and `I_dot omega` terms.

The neutral geometry calculation in [`barrowman.py`](../../src/rocket_model/barrowman.py) uses a constant nose slope of 2, a fixed half-ellipsoid nose CP, and one slope/CP for each fin set. The body tube has no normal-force contribution. Movable fins enter the runtime force calculation individually through local incidence and configured control effectiveness.

## Differences that matter below Mach 0.7

### Drag and predicted altitude

This is the largest difference. OpenRocket derives many friction, pressure, interference, hardware, and base contributions. RocketPy requires a supplied powered/coast curve. This simulator uses one rough build-up with an abrupt laminar/turbulent switch at Reynolds number 500,000, no roughness, no compressibility, no body fineness correction, no launch hardware, no fin-edge profile drag, no radius-step drag, no interference drag, and no induced or angle-dependent drag.

The calibration multipliers scale every contribution, so they can conceal missing physics instead of identifying it. The checked-in configuration currently uses multipliers of 1.0, but older generated runs and any plots made under other values must be tied to their input hash before comparison. Existing run output should be considered a software smoke test until drag is benchmarked.

### Static stability and normal force

All three start from Barrowman concepts. OpenRocket covers the most geometry and incorporates more Mach behavior, body lift, interference, and stall handling. RocketPy is less elaborate in the standard surface model but supports data-driven generic surfaces. This simulator has only nose and aggregate fin-set terms with constant slopes and CPs.

At small angle and Mach below 0.7, the current equations should give the correct qualitative restoring direction for a conventional symmetric rocket. They cannot yet predict individual fin loads, body lift, interference changes, stall, or control effectiveness.

### Damping and roll

This simulator and RocketPy calculate local flow at component CPs, which gives an interpretable pitch/yaw damping mechanism. OpenRocket uses a global empirically amplified pitch/yaw damping model. OpenRocket and RocketPy both model fin-cant roll forcing and roll damping; this simulator does neither. Calling its current aerodynamics “full 6-DoF” can therefore mislead: the state has six degrees of freedom, while aerodynamic roll authority is effectively absent.

### Dynamics and numerics

RocketPy has the most complete variable-mass mechanics. This simulator includes gyroscopic and inertia-derivative terms that OpenRocket omits, but it lacks moving-reference/CG derivative and nozzle mass-flow terms found in RocketPy. The state position is not documented precisely enough when CG moves: force moments use the instantaneous CG, while the translational state is initialized at the rail origin and does not explicitly shift with CG. That reference must be defined and made consistent.

OpenRocket and RocketPy treat events as integration boundaries or reconstruct their roots. This simulator detects rail exit, burnout, validity crossing, and apogee on the fixed 1 ms sample grid. RK4 stages can straddle the discontinuous rail/free-flight branch. One millisecond makes the raw timing error small for the demo, but a convergence test and root-located events are still needed.

## What is good in this project

- **The physics is inspectable.** SI units, ENU world coordinates, Body-FLU axes, quaternion convention, force application points, and hashes are explicit.
- **Local component flow is a strong foundation.** Applying `omega x r` at the nose and tail is more physically traceable than inserting an arbitrary damping moment.
- **The rotational equation has useful terms.** Gyroscopic coupling and changing diagonal inertia are already represented.
- **The validity envelope is honest.** Stopping at Mach 0.7 or 10 degrees is safer than silently extrapolating a linear Barrowman model. It needs to be enforced locally inside force evaluation, but the policy is good.
- **Drag opposes the actual relative-flow vector.** This is geometrically consistent at incidence, although its lateral component must be reconciled with the separately applied normal-force convention.
- **Inputs are strict and reproducible.** Schema validation, finite values, immutable motor histories, and content hashes provide a good basis for validation.
- **The exact motor curve and changing mass properties are much better than a constant-thrust toy model.** They make controlled comparison against other tools practical.

## Inaccuracies, their origins, and consequences

### Critical: aerodynamic results are not yet calibrated

The drag model is the dominant trajectory uncertainty. The fixed base values and simplified friction law are engineering placeholders without documented validation for this airframe. Authored component skin-friction coefficients are added on top of a computed whole-rocket skin-friction term, which can double count friction. A scalar multiplier can make a chosen flight match while leaving the Mach, Reynolds, power-state, and geometry dependence wrong.

**Origin:** the implementation grew from the simpler `sim_rocketpy` educational model, which used fixed powered/coast drag, and from the geometry preview path, which needed visible plausible forces rather than calibrated drag prediction.

**Consequence:** apogee, maximum speed, max-Q, coast duration, and wind drift are not defensible until drag is independently established.

### Critical: configured control surfaces do nothing

The YAML validates all-moving fins, deflection limits, actuator rate, and a control-effectiveness multiplier, but the physics collapses the complete fin set to one centerline force. No deflection state or command reaches the force model. There is no radial CP offset, differential-fin moment, roll torque, hinge/servo model, or individual-fin local incidence.

**Origin:** the first implementation intentionally delivered passive flight before closing the control loop; the earlier `sim_rocketpy` prototype actually had an individual-fin loop that has not yet been ported into the YAML simulator.

**Consequence:** the current simulator cannot validate a controller or even passive roll response, despite accepting control-surface configuration.

### High: the local validity limit is checked too late

The force function evaluates linear normal force for every RK4 stage at any local incidence. Only after a completed step does the runner inspect CG alpha/beta. A nose or fin can exceed 10 degrees because of angular rate while CG alpha and beta remain allowed. Diagnostic-continuation mode keeps using the invalid linear force indefinitely.

**Origin:** the envelope was implemented as a run termination policy rather than a domain constraint and diagnostic on every aerodynamic source.

**Consequence:** the final accepted step, intermediate RK stages, and rotating components may use coefficients outside their declared domain.

### High: normal-force and interference coverage is incomplete

The body contributes no normal force. Nose slope, fin slope, and CP are independent of Mach, Reynolds number, angle, and power state. Fin sets have a coarse interference factor, with no individual fin loading, body carryover load, asymmetric configuration, or wake/shadow interaction. Normal force is constrained to the body transverse plane and produces no induced axial force.

**Origin:** the implementation reuses the repository's deliberately neutral, low-angle Barrowman CP calculator, originally designed for geometry feedback and the archived Gazebo force plugin.

**Consequence:** static margin is useful for a first layout check, while transient attitude response and loads have larger, unquantified errors.

### High: variable-mass reference mechanics need an audit

The code combines dry and propellant mass/CG/inertia and includes `I_dot omega`, but omits CG velocity/acceleration terms, products of inertia, propellant-flow angular momentum, nozzle coupling, and thrust pressure correction. `nozzle_station_m` is validated but unused; thrust always acts on the body axis with zero moment. The motor mass curve is not checked for physical consistency with mass flow and thrust.

**Origin:** this is a pragmatic rigid-body extension of the earlier fixed-parameter simulator, rather than a control-volume derivation such as RocketPy's.

**Consequence:** errors may be small for a symmetric, nearly axial solid motor, but the assumptions must be quantified before simulating fast roll, thrust misalignment, TVC, or strongly shifting propellant.

### Medium: rail and event models are discrete approximations

The rail is a kinematic projection based on the translated state crossing rail length. It has no button positions, effective rail length, friction, binding, reaction loads, partial-constraint geometry, or clearance check. Logged body force excludes the rail reaction even though acceleration includes its projection. Apogee is the first sampled `vz <= 0`, rather than a root-located maximum.

**Origin:** a compact fixed-step prototype optimized for determinism and simple output.

**Consequence:** rail loads cannot be designed from these results, and event state/timing carries grid and branch-switching error.

### Medium: environment and outputs limit validation

World `z` is treated as ISA altitude with no launch elevation. Atmosphere clamps at 11 km, gravity is constant, and wind is constant in space and time. This is adequate for the current altitude range in calm conditions, but not for realistic dispersion. Output omits Reynolds number, atmosphere state, body/world force and moment, inertia, local component incidence, and a component force ledger.

**Origin:** initial scope targeted one deterministic ascent to apogee.

**Consequence:** missing forces are difficult to diagnose, and comparisons can match trajectory for the wrong reason.

### Medium: tests demonstrate execution, not aerodynamic accuracy

Current tests check sea-level atmosphere, launch attitude, coarse mass closure, successful apogee, and output files. The nominal altitude acceptance range is deliberately broad. There are no timestep-convergence, analytic force/moment, symmetry, event-root, local-envelope, or matched OpenRocket/RocketPy tests.

**Origin:** prototype acceptance criteria were aimed at establishing the simulation pipeline.

**Consequence:** passing tests establish software continuity, not agreement with physical measurements.

## Recommended implementation order

1. **Make every result auditable.** Emit each component's local Mach, Reynolds number, angle, force, moment, CP, coefficient source, and validity flags. Record the resolved YAML and source versions with every run.
2. **Replace the drag contract.** Support powered and coast `Cd(Mach, Reynolds, alpha)` tables as authoritative input. Keep an OpenRocket-like geometry build-up as a named fallback with separate friction, pressure, base, hardware, and interference terms. Never hide all terms behind an unexplained global scale.
3. **Create a matched axial benchmark.** Run the same mass history, CTI 141G78 thrust, atmosphere, wind, rail, and Cd table in this simulator and RocketPy. Compare rail exit, burnout, max-Q, maximum speed, and apogee. Then repeat against OpenRocket using equivalent geometry and settings.
4. **Implement individual movable fins.** Give every fin its radial and axial CP, local flow, deflection state, rate limit, normal-force law, and `r x F`. Add cant roll forcing and roll damping. This unlocks controller testing and component load checks.
5. **Improve passive stability.** Add body lift and a documented fin/body interference method. Add subsonic Mach correction and a nonlinear/stall transition, while retaining explicit invalid/out-of-model flags.
6. **Enforce model domains inside force evaluation.** Check every local component state at every derivative call. Choose an explicit behavior—terminate at the located boundary, switch to a broader model, or mark and continue—without silently extrapolating.
7. **Upgrade phases and event roots.** Separate constrained-rail and free-flight integration, locate rail exit and apogee roots, and run timestep convergence. Add rail-button geometry and reaction loads if structural or controller validation needs them.
8. **Audit variable-mass equations.** Define the translational reference point, derive the moving-CG terms, support a full inertia tensor, and quantify nozzle/mass-flow effects. Implement only terms that materially affect this small solid rocket, but document each omitted term and its estimated size.
9. **Validate against measurements.** At minimum measure loaded/dry mass and CG, thrust curve provenance, fin alignment/cant, dimensions, surface roughness, launch weather, and observed apogee. Calibrate uncertain inputs with uncertainty bands rather than a single best-fit multiplier.

## Acceptance criteria before calling it predictive

For the intended Mach-below-0.7 envelope, a credible first release should show:

- timestep convergence of key events and states;
- force-level unit tests with known signs and magnitudes;
- matched RocketPy axial trajectory using the same Cd and environment;
- matched OpenRocket static CP/CNa and a documented explanation for differences;
- individual-fin/control moment tests and roll damping tests;
- a component drag ledger whose sum equals total drag;
- local validity flags for every aerodynamic source;
- comparison with at least one measured flight, including uncertainty in wind, drag, mass, CG, and motor impulse.

Until then, the simulator is valuable for software integration, coordinate/frame verification, qualitative stability, controller plumbing after fin forces are implemented, and sensitivity studies. Numerical apogee or load predictions should be labeled prototype estimates.

## Primary sources

- OpenRocket source at the reviewed commit: [`BarrowmanCalculator`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/BarrowmanCalculator.java), [`BarrowmanStabilityCalculator`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/BarrowmanStabilityCalculator.java), [`BarrowmanDragCalculator`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/BarrowmanDragCalculator.java), [`FinSetCalc`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/aerodynamics/barrowman/FinSetCalc.java), and [`RK4SimulationStepper`](https://github.com/openrocket/openrocket/blob/fe6ec251b697067e3e2a48b08050a526ec508367/core/src/main/java/info/openrocket/core/simulation/RK4SimulationStepper.java).
- OpenRocket [technical documentation and validation report](https://openrocket.info/documentation.html).
- RocketPy source: [`Flight`](https://github.com/RocketPy-Team/RocketPy/blob/master/rocketpy/simulation/flight.py), [`AeroSurface`](https://github.com/RocketPy-Team/RocketPy/blob/master/rocketpy/rocket/aero_surface/aero_surface.py), [`Fins`](https://github.com/RocketPy-Team/RocketPy/blob/master/rocketpy/rocket/aero_surface/fins/fins.py), and [equations of motion](https://github.com/RocketPy-Team/RocketPy/blob/master/docs/technical/equations_of_motion.rst).
- RocketPy official documentation: [environment](https://docs.rocketpy.org/en/latest/reference/classes/Environment.html), [generic surfaces](https://docs.rocketpy.org/en/latest/user/rocket/generic_surface.html), and [flight comparison](https://docs.rocketpy.org/en/latest/user/flight_comparator.html).
