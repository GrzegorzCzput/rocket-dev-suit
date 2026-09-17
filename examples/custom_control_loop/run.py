"""Minimal workspace for developing a rocket stabilization controller."""

from pathlib import Path

import numpy as np

from rocket_sim import ControlSimulation, ControlTelemetry, load_run, mix_x_fin_commands
from rocket_sim.plotting import plot_history


HERE = Path(__file__).parent
OUTPUT = HERE.parents[1] / ".artifacts" / "runs" / "custom-control-loop"


def stabilization_controller(data: ControlTelemetry) -> dict[str, float]:
    """Called at 100 Hz. Return an absolute target angle for each fin."""
    # Euler angles are convenient for display but singular near a vertical
    # attitude. Use the quaternion directly in a serious stabilizer.
    roll, pitch, yaw = np.degrees(data.attitude_rpy_rad)
    roll_rate, pitch_rate, yaw_rate = np.degrees(data.angular_velocity_body_radps)

    # print(
    #     f"t={data.time_s:6.3f} s  "
    #     f"rpy=({roll:7.2f}, {pitch:7.2f}, {yaw:7.2f}) deg  "
    #     f"rates=({roll_rate:8.2f}, {pitch_rate:8.2f}, {yaw_rate:8.2f}) deg/s  "
    #     f"velocity={data.velocity_world_mps} m/s  "
    #     f"acceleration={data.acceleration_world_mps2} m/s²"
    # )

    # Virtual roll/pitch/yaw demands expressed as equivalent fin angles. Put
    # your stabilization controller outputs here. Positive roll moves all fins
    # together; pitch and yaw use differential X-fin patterns.

    pitch_deg = 0.0
    roll_deg = 0.0
    yaw_deg = 0.0


    roll_command_rad = np.radians(roll_deg)
    pitch_command_rad = np.radians(pitch_deg)
    yaw_command_rad = np.radians(yaw_deg)
    return mix_x_fin_commands(
        roll_command_rad,
        pitch_command_rad,
        yaw_command_rad,
        maximum_abs_rad=np.radians(15.0),
    )


inputs = load_run(HERE / "simulation.yaml")
simulation = ControlSimulation(inputs, stabilization_controller, controller_rate_hz=100)

history: list[ControlTelemetry] = []
while simulation.running:
    telemetry = simulation.step()  # one 1 ms physics step
    history.append(telemetry)

plot_path = OUTPUT / "control_dashboard.png"
plot_history(history, plot_path)
print(f"Simulation finished at {telemetry.time_s:.3f} s and {telemetry.position_world_m[2]:.2f} m altitude")
print(f"Wrote {plot_path}")
