# YAML-driven 6-DoF aerodynamics below Mach 0.7

**Question.** What aerodynamic model should a custom six-degree-of-freedom simulator use for the project's small, slender model rockets below Mach 0.7?

## Recommendation

Use an extended Barrowman model for small-angle normal forces and centers of pressure, combined with a component drag build-up and explicit empirical calibration. Evaluate each aerodynamic component using its local air-relative velocity, including the rotational contribution at its force application point. Integrate the rigid-body state with deterministic fixed-step RK4 and verify time-step convergence.

The model should enforce its declared envelope: Mach below 0.7 and absolute angle of attack and sideslip no greater than 10 degrees. It must report an out-of-envelope condition rather than silently extrapolate through separated flow, fin stall, or transonic behavior.

## Evidence and limits

The original Barrowman method was developed for slender finned vehicles and supplies component normal-force derivatives and centers of pressure. The original report is available from the [NASA Technical Reports Server](https://ntrs.nasa.gov/api/citations/20010047838/downloads/20010047838.pdf). NASA's later TAD work likewise calculates normal-force, pitch, and roll derivatives for slender, finned, axisymmetric vehicles at small angle of attack and across Mach number; see the [NASA TAD record](https://ntrs.nasa.gov/citations/19790041753).

NASA's model-rocketry material describes the relevant Barrowman assumptions: a slender, rigid, axisymmetric rocket with thin fins in smooth, slowly varying flow at small angle of attack. It also notes that below approximately Mach 0.8, skin friction dominates nose-cone drag while transonic pressure drag has not yet grown sharply. See the [NASA High Power Rocketry counterpart document](https://www.nasa.gov/wp-content/uploads/2023/11/sl-video-instruction-book.pdf).

Drag cannot be inferred reliably from one constant coefficient. NASA separates skin-friction, pressure, interference, and base drag, and notes the importance of base drag after motor burnout in its [rocket aerodynamics guide](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/rocket-aerodynamics/). Experimental or published coefficients must match Reynolds number as well as Mach number; NASA explains the similarity requirement in [Similarity Parameters](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/similarity-parameters-2/).

OpenRocket provides the closest open precedent for this project: six-degree-of-freedom simulation built around extended Barrowman aerodynamics and model-rocket-specific environment and recovery models. See [OpenRocket features](https://openrocket.info/features.html) and its [technical documentation](https://dokk.org/library/openrocket_technical_documentation_v13.05_2013_Niskanen).

## Proposed force model

For each component, compute local air-relative velocity in the Body Frame:

```text
v_local = v_air_body + omega_body × r_component
```

Use dynamic pressure from the local speed. Baseline axial drag is a documented build-up of skin-friction, pressure/form, fin/interference, and base contributions. Distinguish powered and coasting base drag. Apply optional Rocket-owned calibration factors or coefficient tables when measured data is available.

For the Nose and each Fin Set, derive neutral normal-force slope and center of pressure from geometry. Apply the resulting force at the component center of pressure. Evaluate movable fins individually with their local flow, deflection, actuator rate and angle limits, and calibrated Control Effectiveness. This distributed-force formulation produces pitch and yaw moments and rate damping through the local `omega × r` term.

## Configuration implications

The Rocket Definition should own geometry, reference dimensions, motor installation, checked-in motor-data references, dry and propellant mass properties, aerodynamic calibration, and actuator limits. Motor files should supply sampled thrust and time-varying propellant mass, center of mass, and inertia data.

The separate Simulation Scenario should own atmosphere and wind profiles, gravity, launch-rail pose and length, initial state, controller and sensor selection, integration step, stop condition, and output requests.

The first release should simulate rail-constrained powered ascent, free-flight ascent and coast through apogee. Recovery belongs to a later aerodynamic regime with separate coefficients and events.

## Validation plan

Validate derived Barrowman values against worked cases and compare passive trajectories with RocketPy or OpenRocket using identical inputs. Unit tests should cover force and moment signs, symmetric zero-angle flight, local-flow rate damping, motor interpolation, time-varying mass properties, rail release, burnout, quaternion normalization, actuator limits, and out-of-envelope diagnostics. Run the same cases at successively smaller RK4 steps and define numerical acceptance tolerances before treating a trajectory as a reference.

Flight-test calibration should begin with measured motor curves and altitude/velocity data. Control-effectiveness claims require stronger evidence, such as repeatable instrumented flights, force-balance testing, or suitable CFD. The model does not cover high-angle separated flow, fin stall, structural flexibility, transonic flow, plume interaction beyond calibrated powered drag, or parachute aerodynamics.
