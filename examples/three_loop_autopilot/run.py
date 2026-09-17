"""Vertical-flight three-axis autopilot and pitch-acceleration step demo."""

from pathlib import Path
import os
import tempfile

_mpl_config = Path(tempfile.gettempdir()) / "rocket_sim_mplconfig"
_mpl_config.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_mpl_config))

import matplotlib.pyplot as plt
import numpy as np

from rocket_sim import ControlSimulation, ControlTelemetry, ThreeLoopAutopilot, load_run
from rocket_sim.plotting import plot_history


HERE = Path(__file__).parent
OUTPUT = HERE.parents[1] / ".artifacts" / "runs" / "three-loop-autopilot"


def acceleration_command(time_s: float) -> np.ndarray:
    """Command Body-FLU lateral acceleration `[forward, left, up]` in m/s²."""
    pitch_axis_acceleration = 0.0
    if 0.5 <= time_s < 3.0:
        pitch_axis_acceleration = 15.0
    elif 3.0 <= time_s < 6.0:
        pitch_axis_acceleration = -15.0
    return np.array((0.0, 0.0, pitch_axis_acceleration))


def plot_autopilot_response(history: list[ControlTelemetry], output: Path) -> None:
    time = np.array([sample.time_s for sample in history])
    command = np.array([acceleration_command(value) for value in time])
    body_acceleration = np.array([sample.acceleration_body_mps2 for sample in history])
    angular_rate = np.degrees(np.array([sample.angular_velocity_body_radps for sample in history]))
    fins = np.degrees(np.array([list(sample.fin_deflection_rad.values()) for sample in history]))

    figure, axes = plt.subplots(3, 1, figsize=(12, 10), constrained_layout=True, sharex=True)
    axes[0].step(time, command[:, 2], where="post", label="commanded body-up acceleration")
    axes[0].plot(time, body_acceleration[:, 2], label="actual body-up acceleration", alpha=0.8)
    axes[0].set(ylabel="m/s²", title="Pitch-channel lateral-acceleration tracking")
    axes[1].plot(time, angular_rate[:, 0], label="roll p")
    axes[1].plot(time, angular_rate[:, 1], label="pitch q")
    axes[1].plot(time, angular_rate[:, 2], label="yaw r")
    axes[1].set(ylabel="deg/s", title="Body angular rates")
    for index, label in enumerate(history[0].fin_deflection_rad):
        axes[2].plot(time, fins[:, index], label=label)
    axes[2].set(xlabel="Time (s)", ylabel="deg", title="Rate-limited fin commands")
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    plt.close(figure)


inputs = load_run(HERE / "simulation.yaml")
autopilot = ThreeLoopAutopilot(inputs, acceleration_command)
simulation = ControlSimulation(inputs, autopilot, controller_rate_hz=100)

history: list[ControlTelemetry] = []
while simulation.running:
    history.append(simulation.step())

plot_history(history, OUTPUT / "flight_dashboard.png")
plot_autopilot_response(history, OUTPUT / "autopilot_response.png")
print(f"Finished at {history[-1].time_s:.3f} s and {history[-1].position_world_m[2]:.2f} m altitude")
print(f"Wrote {OUTPUT}")
