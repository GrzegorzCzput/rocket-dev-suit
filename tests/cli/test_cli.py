from __future__ import annotations

from pathlib import Path

import yaml

from rocket_tool.cli import main


ROOT = Path(__file__).parents[2]


def test_documented_validation_commands(capsys):
    assert main(["model", "validate", str(ROOT / "examples/passive_flight/rocket.yaml")]) == 0
    assert main(["sim", "validate", str(ROOT / "examples/passive_flight/simulation.yaml")]) == 0
    output = capsys.readouterr().out
    assert "Valid rocket definition" in output
    assert "Valid simulation" in output


def test_migrate_splits_legacy_rocket_and_scenario(tmp_path):
    current_rocket = yaml.safe_load((ROOT / "examples/passive_flight/rocket.yaml").read_text(encoding="utf-8"))
    rocket = current_rocket["rocket"]
    rocket["dry_mass_properties"] = rocket.pop("mass_properties")
    motor_dir = tmp_path / "motors"
    motor_dir.mkdir()
    csv_source = ROOT / "legacy/motor_samples/CTI_141G78_15A.csv"
    eng_source = ROOT / "data/motors/cti/pro38_1g_141_g78/thrust.eng"
    (motor_dir / "demo.csv").write_bytes(csv_source.read_bytes())
    (motor_dir / "demo.eng").write_bytes(eng_source.read_bytes())
    rocket["flight"] = {
        "motor": {"data_file": "motors/demo.csv", "data_format": "csv", "nozzle_station_m": 0.0},
        "aerodynamics": {},
        "actuators": [{"fin_set_id": "rear", "maximum_rate_radps": 10.0}],
    }
    legacy_rocket = tmp_path / "legacy-rocket.yaml"
    legacy_scenario = tmp_path / "legacy-scenario.yaml"
    legacy_rocket.write_text(yaml.safe_dump({"schema_version": 2, "rocket": rocket}), encoding="utf-8")
    scenario = yaml.safe_load((ROOT / "legacy/examples/scenario_crosswind_v2.yaml").read_text(encoding="utf-8"))
    legacy_scenario.write_text(yaml.safe_dump(scenario), encoding="utf-8")

    destination = tmp_path / "migrated"
    assert main(["migrate", str(legacy_rocket), "--scenario", str(legacy_scenario), "--output", str(destination)]) == 0
    assert main(["sim", "validate", str(destination / "simulation.yaml")]) == 0
