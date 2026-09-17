from pathlib import Path

import pytest
from pydantic import ValidationError

from rocket_sim import MotorCurve, load_run


ROOT = Path(__file__).parents[2]


def test_run_definition_resolves_motor_and_produces_stable_hashes():
    path = ROOT / "examples" / "passive_flight" / "simulation.yaml"
    first = load_run(path)
    second = load_run(path)

    assert first.motor.sample(0.018).thrust_n == pytest.approx(137.928)
    assert first.motor.sample(99).thrust_n == 0
    assert first.motor.burnout_time_s == pytest.approx(1.852)
    assert first.source_paths["motor"].name == "motor.yaml"
    assert first.sha256 == second.sha256
    assert all(len(value) == 64 for value in first.sha256.values())


def test_motor_curve_rejects_increasing_propellant_mass(tmp_path):
    source = tmp_path / "bad.csv"
    source.write_text(
        "time_s,thrust_n,propellant_mass_kg,propellant_cg_station_m,inertia_xx_kg_m2,inertia_yy_kg_m2,inertia_zz_kg_m2\n"
        "0,0,0.1,0.1,0.001,0.001,0.001\n"
        "1,0,0.2,0.1,0.001,0.001,0.001\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="must not increase"):
        MotorCurve.from_csv(source)


def test_configuration_rejects_aerodynamic_limits_beyond_trusted_envelope(tmp_path):
    scenario = (ROOT / "examples" / "passive_flight" / "simulation.yaml").read_text(encoding="utf-8").replace("maximum_mach: 0.7", "maximum_mach: 0.71")
    scenario = scenario.replace("rocket: rocket.yaml", f"rocket: {ROOT / 'examples' / 'passive_flight' / 'rocket.yaml'}")
    scenario = scenario.replace("definition: ../../data/motors/cti/pro38_1g_141_g78/motor.yaml", f"definition: {ROOT / 'data' / 'motors' / 'cti' / 'pro38_1g_141_g78' / 'motor.yaml'}")
    path = tmp_path / "simulation.yaml"
    path.write_text(scenario, encoding="utf-8")

    with pytest.raises(ValidationError, match="maximum_mach"):
        load_run(path)


def test_every_movable_fin_set_requires_actuator_dynamics(tmp_path):
    run = (ROOT / "examples" / "passive_flight" / "simulation.yaml").read_text(encoding="utf-8").replace(
        "actuators:\n  - fin_set_id: rear\n    maximum_rate_radps: 10.471975511966\n",
        "actuators: []\n",
    )
    run = run.replace("rocket: rocket.yaml", f"rocket: {ROOT / 'examples' / 'passive_flight' / 'rocket.yaml'}")
    run = run.replace("definition: ../../data/motors/cti/pro38_1g_141_g78/motor.yaml", f"definition: {ROOT / 'data' / 'motors' / 'cti' / 'pro38_1g_141_g78' / 'motor.yaml'}")
    path = tmp_path / "simulation.yaml"
    path.write_text(run, encoding="utf-8")

    with pytest.raises(ValueError, match="every movable Fin Set"):
        load_run(path)
