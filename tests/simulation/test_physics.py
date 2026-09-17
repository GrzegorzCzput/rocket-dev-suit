from pathlib import Path

import numpy as np
import pytest

from rocket_sim import load_run, run_simulation, write_simulation_outputs
from rocket_sim.physics import VehicleDynamics, isa1976, quaternion_to_matrix


ROOT = Path(__file__).parents[2]


@pytest.fixture(scope="module")
def inputs():
    return load_run(ROOT / "examples" / "passive_flight" / "simulation.yaml")


def test_isa_sea_level_reference_values():
    atmosphere = isa1976(0)
    assert atmosphere.density_kg_m3 == pytest.approx(1.225, rel=5e-4)
    assert atmosphere.speed_of_sound_mps == pytest.approx(340.3, rel=5e-4)


def test_initial_attitude_aligns_body_forward_with_launch_rail(inputs):
    dynamics = VehicleDynamics(inputs)
    rotation = quaternion_to_matrix(dynamics.initial_quaternion)
    assert rotation[:, 0] == pytest.approx(dynamics.rail_direction)
    assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_motor_mass_properties_are_combined_about_total_center_of_mass(inputs):
    dynamics = VehicleDynamics(inputs)
    initial = dynamics.mass_state(0)
    final = dynamics.mass_state(99)
    assert initial.mass_kg == pytest.approx(0.919)
    assert final.mass_kg == pytest.approx(0.850)
    assert initial.center_of_mass_station_m < final.center_of_mass_station_m
    assert np.all(initial.inertia_kg_m2 > 0)


def test_passive_example_reaches_apogee_inside_validity_envelope(inputs, tmp_path):
    result = run_simulation(inputs)
    event_names = [event["name"] for event in result.events]
    assert result.status == "apogee_reached"
    assert event_names == ["rail_exit", "burnout", "apogee"]
    assert 100 < result.altitude_m.max() < 2000
    assert result.mach.max() < 0.7
    assert np.max(np.abs(result.angle_of_attack_rad)) < 1e-8

    output = write_simulation_outputs(inputs, result, tmp_path / "run")
    assert (output / "timeseries.csv").is_file()
    assert (output / "run.json").is_file()
    assert (output / "simulation_dashboard.png").stat().st_size > 10_000


def test_all_moving_fins_follow_five_degree_pulse_and_generate_roll():
    commanded = load_run(ROOT / "examples" / "fin_step_response" / "simulation.yaml")

    result = run_simulation(commanded)

    assert result.fin_labels == ("rear_1", "rear_2", "rear_3", "rear_4")
    before = np.searchsorted(result.time_s, 1.99)
    during = np.searchsorted(result.time_s, 2.02)
    after = np.searchsorted(result.time_s, 3.02)
    assert result.fin_deflection_rad[before] == pytest.approx(np.zeros(4), abs=1e-12)
    assert result.fin_deflection_rad[during] == pytest.approx(np.full(4, np.deg2rad(5)), abs=1e-6)
    assert result.fin_deflection_rad[after] == pytest.approx(np.zeros(4), abs=1e-6)
    assert np.max(np.abs(result.state[:, 10])) > np.deg2rad(1)
