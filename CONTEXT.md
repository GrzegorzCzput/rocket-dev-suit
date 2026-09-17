# Rocket Geometry

This context describes the physical shape and aerodynamic reference locations of a hobby rocket.

## Language

**Rocket**:
A rigid, single-stage vehicle consisting of a circular body, a nose, and zero or more circumferential fin sets.
_Avoid_: Aircraft, missile

**Body Tube**:
The straight, circular cylindrical core of the rocket.
_Avoid_: Body tune, fuselage

**Nose**:
The axisymmetric ellipsoidal forward end of the rocket.
_Avoid_: Nose body, nose tube

**Aft Datum**:
The centre of the body tube's rear plane and the origin of the rocket's axial coordinate system.
_Avoid_: Rocket bottom, fin zero

**Body Frame**:
The rocket-fixed, right-handed coordinate frame whose origin is the Aft Datum, with +x toward the nose, +y toward the rocket's left, and +z toward its top.
_Avoid_: World frame, screen coordinates

**Station**:
The axial distance measured forward from the Aft Datum toward the nose.
_Avoid_: X position, nose-referenced station

**Azimuth**:
The right-hand angle around the Body Frame's +x axis, measured from +z.
_Avoid_: Fin angle, roll position

**Fin Set**:
A group of matching trapezoidal fins mounted at the same station and distributed evenly around the body tube.
_Avoid_: Wing set

**Fin Station**:
The station of a fin's root trailing edge. A Fin Station of zero places that edge at the Aft Datum.
_Avoid_: Fin start, fin position

**Fixed Fin**:
A rigid trapezoidal fin with no movable aerodynamic portion.
_Avoid_: Passive fin, static wing

**All-Moving Fin**:
A trapezoidal fin that rotates as a whole about a radial hinge axis passing through its Neutral Geometry Center of Pressure.
_Avoid_: Whole controlled fin, rotating fin

**Control-Surface Fin**:
A trapezoidal fin whose fixed primary portion carries a movable trailing Control Surface.
_Avoid_: Controllable fin, flapped fin

**Control Surface**:
The movable trailing portion of a fin, bounded by a hinge line and a configurable spanwise start.
_Avoid_: Flap, movable fin

**Actuator**:
One independently commandable All-Moving Fin or Control Surface. A Fin Set shares geometry but is not an actuator group.
_Avoid_: Fin set, control channel

**Neutral Geometry**:
The rocket geometry with every Control Surface at zero deflection.
_Avoid_: Current geometry, default pose

**Preview Pose**:
A transient set of actuator deflections used to inspect the rocket without changing its Rocket Definition.
_Avoid_: Saved geometry, rocket configuration

**Center of Pressure (CP)**:
The effective axial location of the aerodynamic normal force for a component or the assembled rocket. It is an aerodynamic result, not inherently a mechanical pivot.
_Avoid_: Hinge point, rotation point

**Force Application Point**:
A full three-dimensional Body-Frame point at which a simulator may apply a component aerodynamic force. The Nose point is its CP Station on the centreline; a Fin point is its Barrowman CP Station at mid-span.
_Avoid_: CP station, hinge point

**Barrowman Neutral CP**:
A Center of Pressure estimated from Neutral Geometry using the simplified Barrowman assumptions and component normal-force contributions.
_Avoid_: Deflected CP, flight CP

**Normal-Force Slope**:
The rate at which a component's normal-force coefficient changes with angle of attack at Neutral Geometry. Component slopes weight their Center of Pressure contributions to the assembled Rocket result.
_Avoid_: Lift, force

**Mass Properties**:
The rocket's total mass, Center of Mass Station, and principal moments of inertia about that Center of Mass in the Body Frame.
_Avoid_: Weight, inertia configuration

**Center of Mass Station**:
The Station at which the rocket's mass is balanced along its longitudinal axis. In this uniform-gravity model it is also the Center of Gravity (CG).
_Avoid_: CG location

**Rocket Definition**:
The authoritative, persistable description of a rocket's neutral geometry, dry Mass Properties, actuator geometry and limits, motor-mount location, and intrinsic Aerodynamic Properties. It does not select a motor, environment, controller, launch state, or numerical solver.
_Avoid_: Drawing, mesh, image

**Rocket Modeling**:
The activity and tool boundary for creating, validating, inspecting, and rendering reusable Rocket Definitions without running a Flight Simulation.
_Avoid_: Robot creation, simulation setup

**Simulation Run Definition**:
The persistable assembly that selects one Rocket Definition and Motor Performance and contains the Simulation Scenario needed for one reproducible Flight Simulation.
_Avoid_: Rocket Definition, generated run, example folder

**Simulation Scenario**:
A separate, versioned description of the atmosphere, wind, launch state, controller, and run conditions used to simulate a Rocket Definition.
_Avoid_: Rocket Definition, flight configuration

**Motor Performance**:
The reusable description of a motor's thrust curve, dimensions, mass properties, and provenance. A Simulation Run Definition selects Motor Performance and installs it at the Rocket Definition's motor mount.
_Avoid_: Motor geometry, throttle command

**Model Editor**:
The Rocket Modeling application used to create and inspect Rocket Definitions. It is exposed as the `rocket-tool model edit` workflow.
_Avoid_: Simulation editor, flight controller

**Run Artifact**:
A generated result of a Flight Simulation, such as sampled state data, metadata, or plots. Run Artifacts are written beneath `.artifacts/runs/` by default and are not source inputs.
_Avoid_: Simulation Run Definition, documentation asset

**Runnable Example**:
A small, self-contained workflow that combines a Rocket Definition and Simulation Run Definition to demonstrate one simulation or control capability. Shared Motor Performance data is referenced from `data/motors/` rather than copied into the example.
_Avoid_: Test fixture, generated run

**Aerodynamic Calibration**:
Measured or empirically fitted factors or coefficient tables that correct the geometry-derived Aerodynamic Model for a particular Rocket.
_Avoid_: Aerodynamic Properties, arbitrary tuning

**Flight Simulation**:
A deterministic six-degree-of-freedom integration of a Rocket Definition in a Simulation Scenario, including propulsion, actuator, and launch-rail state transitions.
_Avoid_: Geometry preview, Gazebo simulation

**Aerodynamic Component**:
One Body Tube, Nose, Fin, or Control Surface evaluated as a distinct source of aerodynamic force and moment.
_Avoid_: Rocket mesh, visual part

**Aerodynamic Model**:
The runtime calculation that converts air-relative motion, atmospheric conditions, and Aerodynamic Properties into forces and moments on the Rocket.
_Avoid_: Geometry preview, Barrowman CP

**Aerodynamic Properties**:
The Rocket-authored coefficients used by an Aerodynamic Model, including component drag coefficients and control effectiveness.
_Avoid_: Wind, simulation settings

**Skin-Friction Coefficient**:
A dimensionless coefficient that multiplies a component's wetted area to model viscous drag.
_Avoid_: Drag coefficient, lift slope

**Pressure-Drag Coefficient**:
A dimensionless coefficient that multiplies a component's reference area to model pressure or form drag.
_Avoid_: Skin-friction coefficient, Barrowman slope

**Aerodynamic Environment**:
The simulation-authored atmospheric state used by the Aerodynamic Model, including air density and world-frame wind.
_Avoid_: Rocket Definition, Aerodynamic Properties

**Control Effectiveness**:
An empirical factor that scales a trailing Control Surface's aerodynamic response to its deflection.
_Avoid_: Actuator limit, Normal-Force Slope

**Wetted Area**:
The total exposed surface area of an Aerodynamic Component used with its Skin-Friction Coefficient.
_Avoid_: Frontal area, planform area

**Aerodynamic Reference Area**:
The documented derived area used with a Pressure-Drag Coefficient: rocket frontal area for the Body Tube and Nose, and total planform area for a Fin Set.
_Avoid_: Wetted area, arbitrary user-entered area

**Fin-Set Normal Force**:
The Barrowman normal force calculated for an entire Fin Set before it is divided evenly across its matching Fin Links.
_Avoid_: Per-fin normal-force slope, Rocket normal force
