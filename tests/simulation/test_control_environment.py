from pathlib import Path

import numpy as np
import pytest

from rocket_sim import ControlSimulation, ThreeLoopAutopilot, load_run, mix_x_fin_commands


ROOT = Path(__file__).parents[2]


def test_x_fin_mixer_generates_roll_pitch_yaw_patterns_and_desaturates():
    assert mix_x_fin_commands(1, 0, 0, maximum_abs_rad=10) == pytest.approx(
        {"rear_1": 1, "rear_2": 1, "rear_3": 1, "rear_4": 1}
    )
    assert mix_x_fin_commands(0, 1, 0, maximum_abs_rad=10) == pytest.approx(
        {"rear_1": 1, "rear_2": -1, "rear_3": -1, "rear_4": 1}
    )
    assert mix_x_fin_commands(0, 0, 1, maximum_abs_rad=10) == pytest.approx(
        {"rear_1": 1, "rear_2": 1, "rear_3": -1, "rear_4": -1}
    )
    saturated = mix_x_fin_commands(1, 1, 1, maximum_abs_rad=0.2)
    assert max(abs(value) for value in saturated.values()) == pytest.approx(0.2)


def test_controller_receives_telemetry_at_configured_rate_and_commands_fins():
    inputs = load_run(ROOT / "examples" / "custom_control_loop" / "simulation.yaml")
    calls = []

    def controller(telemetry):
        calls.append(telemetry)
        return {label: np.deg2rad(5) for label in telemetry.fin_deflection_rad}

    simulation = ControlSimulation(inputs, controller, controller_rate_hz=100)
    while simulation.time_s < 0.031:
        telemetry = simulation.step()

    assert [sample.time_s for sample in calls] == pytest.approx([0.0, 0.01, 0.02, 0.03])
    assert calls[0].position_world_m.shape == (3,)
    assert calls[0].velocity_world_mps.shape == (3,)
    assert calls[0].velocity_body_mps.shape == (3,)
    assert calls[0].acceleration_world_mps2.shape == (3,)
    assert calls[0].acceleration_body_mps2.shape == (3,)
    assert calls[0].specific_force_body_mps2.shape == (3,)
    assert calls[0].attitude_rpy_rad.shape == (3,)
    assert calls[0].angular_velocity_body_radps.shape == (3,)
    assert calls[0].angular_acceleration_body_radps2.shape == (3,)
    assert telemetry.fin_deflection_rad == pytest.approx(
        {"rear_1": np.deg2rad(5), "rear_2": np.deg2rad(5), "rear_3": np.deg2rad(5), "rear_4": np.deg2rad(5)}
    )


def test_three_loop_autopilot_tracks_positive_and_negative_lateral_acceleration_steps():
    inputs = load_run(ROOT / "examples" / "three_loop_autopilot" / "simulation.yaml")

    def command(time_s):
        pitch_axis = 10.0 if 0.5 <= time_s < 2.0 else -10.0 if 2.0 <= time_s < 3.5 else 0.0
        return np.array((0.0, 0.0, pitch_axis))

    simulation = ControlSimulation(inputs, ThreeLoopAutopilot(inputs, command), controller_rate_hz=100)
    history = []
    while simulation.time_s < 4.5:
        history.append(simulation.step())

    time = np.array([sample.time_s for sample in history])
    pitch_axis_acceleration = np.array([sample.acceleration_body_mps2[2] for sample in history])
    rates = np.array([sample.angular_velocity_body_radps for sample in history])
    positive_steady = (time >= 1.3) & (time < 1.9)
    negative_steady = (time >= 2.7) & (time < 3.4)
    assert np.mean(pitch_axis_acceleration[positive_steady]) == pytest.approx(10.0, abs=2.0)
    assert np.mean(pitch_axis_acceleration[negative_steady]) == pytest.approx(-10.0, abs=2.0)
    assert np.max(np.abs(rates[:, (0, 2)])) < np.deg2rad(0.001)
