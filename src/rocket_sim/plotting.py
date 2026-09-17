from pathlib import Path
import os
import tempfile

_mpl_config = Path(tempfile.gettempdir()) / "rocket_sim_mplconfig"
_mpl_config.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(_mpl_config))

from .control import ControlTelemetry

import matplotlib.pyplot as plt
import numpy as np


def plot_history(history: list[ControlTelemetry], output: Path) -> None:
    """Write a control-development dashboard with a true-scale 3D trajectory."""
    time = np.array([sample.time_s for sample in history])
    position = np.array([sample.position_world_m for sample in history])
    velocity = np.array([sample.velocity_world_mps for sample in history])
    acceleration = np.array([sample.acceleration_world_mps2 for sample in history])
    attitude = np.degrees(np.array([sample.attitude_rpy_rad for sample in history]))
    rates = np.degrees(np.array([sample.angular_velocity_body_radps for sample in history]))
    fin_labels = tuple(history[0].fin_deflection_rad)
    fins = np.degrees(np.array([[sample.fin_deflection_rad[label] for label in fin_labels] for sample in history]))

    figure = plt.figure(figsize=(16, 13), constrained_layout=True)
    trajectory_axis = figure.add_subplot(3, 2, 1, projection="3d")
    trajectory_axis.plot(position[:, 0], position[:, 1], position[:, 2], color="tab:blue")
    trajectory_axis.scatter(*position[0], color="tab:green", label="launch", s=35)
    trajectory_axis.scatter(*position[-1], color="tab:red", label="final", s=35)
    trajectory_axis.set(title="3D trajectory — equal axis scale", xlabel="East (m)", ylabel="North (m)", zlabel="Up (m)")
    trajectory_axis.legend()

    # Use one common numeric span so one metre has the same visual scale on
    # East, North, and Up. This intentionally leaves empty space for a nearly
    # vertical flight instead of exaggerating tiny lateral motion.
    lower = position.min(axis=0)
    upper = position.max(axis=0)
    center = (lower + upper) / 2
    span = max(float(np.max(upper - lower)), 1.0)
    for setter, midpoint in zip((trajectory_axis.set_xlim, trajectory_axis.set_ylim, trajectory_axis.set_zlim), center):
        setter(midpoint - span / 2, midpoint + span / 2)
    trajectory_axis.set_box_aspect((1, 1, 1))

    plots = (
        (figure.add_subplot(3, 2, 2), acceleration, ("east", "north", "up"), "World acceleration", "m/s²"),
        (figure.add_subplot(3, 2, 3), velocity, ("east", "north", "up"), "World velocity", "m/s"),
        (figure.add_subplot(3, 2, 4), rates, ("roll p", "pitch q", "yaw r"), "Body angular rates", "deg/s"),
        (figure.add_subplot(3, 2, 5), attitude, ("roll", "pitch", "yaw"), "Euler attitude (display only)", "deg"),
        (figure.add_subplot(3, 2, 6), fins, fin_labels, "Actual fin deflections", "deg"),
    )
    for axis, values, labels, title, unit in plots:
        for index, label in enumerate(labels):
            axis.plot(time, values[:, index], label=label)
        axis.set(title=title, xlabel="Time (s)", ylabel=unit)
        axis.grid(True, alpha=0.25)
        axis.legend()

    figure.suptitle("Closed-loop rocket simulation")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    plt.close(figure)
