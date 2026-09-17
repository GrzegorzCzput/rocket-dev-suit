from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import shutil
import sys

import yaml

from rocket_model.rendering import render_png
from rocket_model.yaml_io import dump_definition, load_definition
from rocket_sim import load_run, run_simulation, write_simulation_outputs


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="rocket-tool")
    areas = parser.add_subparsers(dest="area", required=True)
    model = areas.add_parser("model", help="create and inspect rocket definitions")
    model_commands = model.add_subparsers(dest="command", required=True)
    validate_model = model_commands.add_parser("validate")
    validate_model.add_argument("path", type=Path)
    render = model_commands.add_parser("render")
    render.add_argument("path", type=Path)
    render.add_argument("--output", type=Path, required=True)
    model_commands.add_parser("edit")

    simulation = areas.add_parser("sim", help="validate and run flight simulations")
    simulation_commands = simulation.add_subparsers(dest="command", required=True)
    validate_run = simulation_commands.add_parser("validate")
    validate_run.add_argument("path", type=Path)
    run = simulation_commands.add_parser("run")
    run.add_argument("path", type=Path)
    run.add_argument("--output", type=Path)
    run.add_argument("--overwrite", action="store_true")

    migrate = areas.add_parser("migrate", help="split a legacy v2 rocket and scenario")
    migrate.add_argument("rocket", type=Path)
    migrate.add_argument("--scenario", type=Path, required=True)
    migrate.add_argument("--output", type=Path, required=True)
    return parser


def _launch_editor() -> int:
    from streamlit.web import cli as streamlit_cli

    import rocket_model.editor

    sys.argv = ["streamlit", "run", str(Path(rocket_model.editor.__file__))]
    return streamlit_cli.main()


def _migrate(rocket_path: Path, scenario_path: Path, output: Path) -> None:
    rocket_document = yaml.safe_load(rocket_path.read_text(encoding="utf-8"))
    scenario_document = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
    legacy = rocket_document["rocket"]
    flight = legacy.pop("flight")
    legacy["mass_properties"] = legacy.pop("dry_mass_properties")
    current_rocket = {"schema_version": 1, "rocket": legacy}
    output.mkdir(parents=True, exist_ok=True)
    rocket_output = output / "rocket.yaml"
    rocket_output.write_text(dump_definition(current_rocket), encoding="utf-8")

    csv_source = (rocket_path.parent / flight["motor"]["data_file"]).resolve()
    eng_source = csv_source.with_suffix(".eng")
    if not eng_source.is_file():
        raise FileNotFoundError(f"matching RASP thrust curve is required for migration: {eng_source}")
    header = next(
        line.split()
        for line in eng_source.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith(";") and len(line.split()) > 2
    )
    with csv_source.open("r", encoding="utf-8", newline="") as stream:
        first = next(csv.DictReader(stream))
    mass = float(first["propellant_mass_kg"])
    radius = math.sqrt(2 * float(first["inertia_xx_kg_m2"]) / mass)
    length = math.sqrt(max(0.0, 12 * float(first["inertia_yy_kg_m2"]) / mass - 3 * radius**2))
    shutil.copyfile(eng_source, output / "thrust.eng")
    motor_document = {
        "schema_version": 1,
        "id": csv_source.stem.lower(),
        "name": csv_source.stem.replace("_", " "),
        "manufacturer": " ".join(header[6:]).replace("_", " "),
        "thrust_curve_file": "thrust.eng",
        "loaded_mass_kg": float(header[5]),
        "propellant_mass_kg": mass,
        "propellant_cg_station_m": float(first["propellant_cg_station_m"]),
        "propellant_radius_m": radius,
        "propellant_length_m": length,
    }
    (output / "motor.yaml").write_text(yaml.safe_dump(motor_document, sort_keys=False), encoding="utf-8")
    run_document = {
        "schema_version": 1,
        "rocket": "rocket.yaml",
        "motor": {"definition": "motor.yaml", "nozzle_station_m": flight["motor"]["nozzle_station_m"]},
        "aerodynamics": flight.get("aerodynamics", {}),
        "actuators": flight.get("actuators", []),
        "simulation": scenario_document["simulation"],
    }
    (output / "simulation.yaml").write_text(yaml.safe_dump(run_document, sort_keys=False), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    if arguments.area == "model":
        if arguments.command == "edit":
            return _launch_editor()
        definition = load_definition(arguments.path)
        if arguments.command == "validate":
            print(f"Valid rocket definition: {definition.rocket.name}")
        else:
            render_png(definition, arguments.output)
            print(f"Wrote {arguments.output}")
        return 0
    if arguments.area == "migrate":
        _migrate(arguments.rocket, arguments.scenario, arguments.output)
        print(f"Wrote migrated inputs to {arguments.output}")
        return 0
    inputs = load_run(arguments.path)
    if arguments.command == "validate":
        print(f"Valid simulation: {inputs.rocket.rocket.name}; motor burnout {inputs.motor.burnout_time_s:.3f} s")
        return 0
    result = run_simulation(inputs)
    output = arguments.output or Path(".artifacts") / "runs" / arguments.path.parent.name
    write_simulation_outputs(inputs, result, output, overwrite=arguments.overwrite)
    print(f"Simulation {result.status}: maximum altitude {result.altitude_m.max():.3f} m; wrote {output}")
    return 0
