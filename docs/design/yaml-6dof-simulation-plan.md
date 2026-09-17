# YAML-driven 6-DoF Flight Simulation design

## Goal

Build a transparent, deterministic six-degree-of-freedom small-rocket simulator for powered ascent through apogee. The simulator consumes the same Rocket Definition used by the geometry editor plus a separate Simulation Scenario. It supports passive trajectory analysis first and controller, sensor, estimator, and guidance development through stable interfaces.

The trusted aerodynamic envelope is Mach below 0.7 with absolute angle of attack and sideslip no greater than 10 degrees. Recovery, high-angle separated flow, fin stall, structural flexibility, and transonic flight are outside the first release.

## Target architecture

The implementation is split between `src/rocket_model/` for reusable vehicle definitions and `src/rocket_sim/` for runtime configuration, motors, physics, control, and results. `src/rocket_tool/` provides the command line.

- `configuration`: versioned Rocket Definition and Simulation Scenario parsing, validation, migration, and immutable input hashes;
- `vehicle`: motor interpolation and time-varying total mass, center of mass, and inertia;
- `atmosphere`: gravity, density, temperature, speed of sound, and altitude-dependent wind;
- `aerodynamics`: component drag, Barrowman normal forces and CPs, movable-fin forces, local-flow calculation, calibration, and validity checks;
- `dynamics`: rigid-body state, quaternion kinematics, force/moment aggregation, launch-rail constraint, RK4 step, and discrete flight events;
- `control`: perfect-state controller protocol plus the explicit Body-FLU to Body-FRD adapter;
- `results`: time-series records, event log, validity status, manifest, CSV writer, and standard dashboard;
- `runner`: orchestration only—load inputs, assemble modules, run, and write outputs.

Port useful code and tests from `sim_rocketpy` module by module. Do not import the nested project at runtime. After parity and validation, preserve the original under `legacy/sim_rocketpy/`.

## Configuration contracts

Keep Rocket Definition `schema_version: 1` independent of simulation. A Simulation Run Definition references that rocket, shared Motor Performance, and the complete scenario. The `rocket-tool migrate` command splits the former combined v2 rocket and scenario inputs.

Rocket-owned data includes:

- neutral geometry and reference diameter/area;
- dry mass, center of mass, and full or principal inertia;
- motor-mount location, while a run selects the installed motor;
- actuator angle limits and neutral positions;
- optional coefficient-table overrides with provenance.

Simulation Run data includes:

- atmosphere and altitude-dependent wind profile;
- gravity and world-frame convention;
- launch location, rail pose and length, and initial state;
- fixed physics step and controller/sensor sampling rates;
- selected motor, actuator rates, aerodynamic calibration, and controller parameters;
- stop condition, random seed, output directory, and diagnostic continuation policy.

Resolve all referenced paths relative to the owning YAML file. Hash the normalized YAML and referenced data files into every run manifest.

## Physics model

Use the 13-value rigid-body state: world position and velocity, scalar-first unit quaternion rotating Body-FLU vectors to the world frame, and Body-FLU angular velocity. Actuator commands and motor phase are explicit simulation state where needed.

At each force evaluation:

1. Evaluate motor thrust, propellant mass, center of mass, and inertia at time `t`.
2. Evaluate atmospheric properties and wind at altitude.
3. Transform air-relative velocity into Body-FLU.
4. For each Aerodynamic Component, compute `v_local = v_air_body + omega × r_component`.
5. Compute component drag using the documented build-up, powered/coasting correction, and any coefficient override.
6. Compute small-angle Nose and Fin normal forces from Barrowman derivatives; evaluate movable fins individually using local incidence and actual deflection.
7. Sum body forces and moments about the instantaneous center of mass, add thrust and gravity, and evaluate translational, rotational, quaternion, and actuator derivatives.
8. Integrate with fixed-step RK4 and normalize the quaternion after each accepted step.

While rail-constrained, permit translation only along the rail and suppress incompatible attitude motion. Interpolate the rail-exit event within the step, then continue with free-flight dynamics. Detect burnout and apogee as structured events.

End a trusted run at Mach 0.7 or when absolute angle of attack or sideslip exceeds 10 degrees. The optional diagnostic mode may continue but marks all subsequent samples invalid.

## Aerodynamic approach

Derive neutral `C_Nalpha` and CP from geometry using the extended Barrowman method. Calculate baseline drag as explicit skin-friction, pressure/form, fin/interference, and base contributions with Reynolds-number dependence and distinct powered/coasting base drag. Apply optional calibration factors. When coefficient tables are supplied, use validated interpolation inside their declared domain and reject extrapolation by default.

Evaluate distributed forces at component CPs. The `omega × r` local-flow term supplies physically interpretable pitch/yaw rate damping. Model each movable fin separately so deflection, radial placement, actuator limits, and Control Effectiveness create forces and moments rather than commanded axis torques.

The evidence and limitations are recorded in [`../research/yaml-6dof-subsonic-aerodynamics.md`](../research/yaml-6dof-subsonic-aerodynamics.md).

## Delivery sequence

1. **Archive and contracts**: archive Gazebo, add the architecture ADR, define schema-version-2 models, fixtures, migration, and path/hash behavior.
2. **Vehicle and environment**: port motor interpolation, changing mass properties, atmosphere, wind, coordinate transforms, and isolated tests.
3. **Passive dynamics**: port rigid-body state and RK4, implement rail constraint/events, add perfect-state zero control, and produce CSV/manifest outputs.
4. **Aerodynamics**: implement local component flow, Barrowman forces, drag build-up, validity envelope, movable-fin forces, and calibration overrides.
5. **Validation**: analytical cases, signs/conservation/quaternion tests, RK4 convergence, and matched-input comparisons against RocketPy or OpenRocket.
6. **Control interface**: port the perfect-state controller and explicit FLU/FRD adapter; validate actuator sampling, limits, and rate limits.
7. **Sensors and guidance**: port sensors, complementary estimator, camera guidance, and autopilot behind the unchanged control boundary.
8. **Retire the reference project**: compare results, migrate useful examples and plots, then move `sim_rocketpy` to `legacy/sim_rocketpy/`.

## Validation gates

The new engine becomes the trusted project simulator only after all gates pass:

- analytical force, moment, frame/sign, zero-force conservation, and quaternion tests;
- motor interpolation, mass/CG/inertia evolution, actuator, rail release, burnout, apogee, and validity-event tests;
- step convergence at the normal step and at least two smaller steps;
- passive matched-input trajectory comparison against RocketPy or OpenRocket;
- controller response comparison against the current `sim_rocketpy` reference cases;
- documented tolerances for rail-exit time/speed, burnout state, apogee time/altitude, and lateral/angular response.

## Run interface and artifacts

Add a CLI shaped as:

```bash
rocket-tool sim run examples/passive_flight/simulation.yaml --output .artifacts/runs/example
```

Every successful or validity-terminated run writes:

- `timeseries.csv` with state, forces, moments, environment, actuators, commands, and validity columns;
- `run.json` with normalized input hashes, referenced-file hashes, model versions, step sizes, events, and final status;
- `simulation_dashboard.png` with trajectory, velocities, attitude/rates, stability/aerodynamic angles, motor/mass state, and actuator/control traces.

These artifacts must be reproducible from the recorded inputs and random seed.
