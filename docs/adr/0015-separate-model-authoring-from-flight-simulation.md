# Separate model authoring from flight simulation

Split the public Python API into `rocket_model` and `rocket_sim`. `rocket_model` owns reusable rocket definitions, validation, geometry calculations, rendering, and the interactive editor. `rocket_sim` depends on `rocket_model` and owns motor loading, run configuration, dynamics, control, integration, and result generation. A small `rocket_tool` package provides the `rocket-tool model ...` and `rocket-tool sim ...` command groups.

A runnable simulation is represented by a Simulation Run Definition that references a separate Rocket Definition and shared Motor Performance data. Complete demonstrations live in workflow-specific directories under `examples/`; shared certified motor data lives under `data/motors/`; generated output defaults to ignored `.artifacts/runs/` paths.

The old `rocket_geometry` Python imports, `rocket-geometry` command, and combined v2 YAML loader remain as deprecated compatibility surfaces for one minor release. A migration command splits a combined legacy YAML file into current rocket and simulation inputs. Legacy combined schemas move to `legacy/schemas/`, while maintained schemas are organized under `schemas/model/` and `schemas/simulation/`.

This boundary lets model authoring evolve independently from numerical simulation and gives users one reproducible input for each simulation run. The temporary compatibility layer adds maintenance cost, but makes the package and configuration migration explicit and testable.
