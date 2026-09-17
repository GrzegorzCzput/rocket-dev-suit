# Rocket simulation toolkit

This repository separates reusable rocket modeling from flight simulation:

- `rocket_model` creates, validates, renders, and edits rocket definitions.
- `rocket_sim` loads motors and run definitions, integrates 6-DoF dynamics, and supports control loops.
- `rocket-tool` provides the command-line workflow.

The simulator targets small subsonic rockets below Mach 0.7 and angles of attack below 10 degrees.

## Install

```bash
python -m pip install -e '.[test]'
```

## Create a rocket

Start from `examples/models/control_surface/rocket.yaml`, or launch the editor:

```bash
rocket-tool model edit
```

Validate and render the result:

```bash
rocket-tool model validate examples/passive_flight/rocket.yaml
rocket-tool model render examples/passive_flight/rocket.yaml --output .artifacts/rocket.png
```

Rocket YAML contains geometry, dry mass and inertia, aerodynamic properties, and actuator geometry. It does not select a motor or launch conditions.

## Run a simulation

`simulation.yaml` references its rocket and a shared motor, then defines the rail, atmosphere, integration, and actuator dynamics.

```bash
rocket-tool sim validate examples/passive_flight/simulation.yaml
rocket-tool sim run examples/passive_flight/simulation.yaml
```

Results are written to `.artifacts/runs/passive_flight/`. Use `--output PATH --overwrite` to choose and replace another output directory.

Runnable workflows are in:

- `examples/passive_flight`
- `examples/fin_step_response`
- `examples/custom_control_loop`
- `examples/three_loop_autopilot`

Run the Python control examples directly after installation:

```bash
python examples/custom_control_loop/run.py
python examples/three_loop_autopilot/run.py
```

## Migrate old inputs

```bash
rocket-tool migrate legacy-rocket-v2.yaml --scenario legacy-scenario-v2.yaml --output migrated-run
```

The old `rocket_geometry` imports and `rocket-geometry` command remain as deprecated compatibility aliases for the 0.2 release.

Schemas are under `schemas/`, design and research notes under `docs/`, tests under `tests/`, and the retired Gazebo implementation under `legacy/gazebo/`.
# rocket-dev-suit
