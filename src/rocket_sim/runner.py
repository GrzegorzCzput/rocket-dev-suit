"""Run and persist a YAML-driven Flight Simulation."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np

from .configuration import SimulationInputs
from .physics import Diagnostics, VehicleDynamics


@dataclass(frozen=True)
class SimulationResult:
    time_s: np.ndarray
    state: np.ndarray
    acceleration_world_mps2: np.ndarray
    mass_kg: np.ndarray
    center_of_mass_station_m: np.ndarray
    thrust_n: np.ndarray
    airspeed_mps: np.ndarray
    mach: np.ndarray
    angle_of_attack_rad: np.ndarray
    sideslip_rad: np.ndarray
    dynamic_pressure_pa: np.ndarray
    drag_coefficient: np.ndarray
    on_rail: np.ndarray
    fin_labels: tuple[str, ...]
    fin_deflection_rad: np.ndarray
    events: tuple[dict[str, float | str], ...]
    status: str

    @property
    def position_world_m(self) -> np.ndarray:
        return self.state[:, 0:3]

    @property
    def velocity_world_mps(self) -> np.ndarray:
        return self.state[:, 3:6]

    @property
    def altitude_m(self) -> np.ndarray:
        return self.position_world_m[:, 2]


def _rk4_step(dynamics: VehicleDynamics, time_s: float, state: np.ndarray, dt: float) -> np.ndarray:
    k1 = dynamics.evaluate(time_s, state)[0]
    k2 = dynamics.evaluate(time_s + dt / 2, state + dt * k1 / 2)[0]
    k3 = dynamics.evaluate(time_s + dt / 2, state + dt * k2 / 2)[0]
    k4 = dynamics.evaluate(time_s + dt, state + dt * k3)[0]
    updated = state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    updated[6:10] /= np.linalg.norm(updated[6:10])
    return updated


def run_simulation(inputs: SimulationInputs) -> SimulationResult:
    dynamics = VehicleDynamics(inputs)
    settings = inputs.scenario.simulation
    dt = settings.integration.time_step_s
    state = dynamics.initial_state()
    rows: list[tuple[float, np.ndarray, Diagnostics]] = []
    events: list[dict[str, float | str]] = []
    was_on_rail = True
    was_powered = False
    status = "maximum_duration_reached"
    maximum_steps = int(round(settings.maximum_duration_s / dt))
    for step in range(maximum_steps + 1):
        time_s = step * dt
        _, diagnostics = dynamics.evaluate(time_s, state)
        rows.append((time_s, state.copy(), diagnostics))
        if was_on_rail and not diagnostics.on_rail:
            events.append({"name": "rail_exit", "time_s": time_s})
        powered = diagnostics.thrust_n > 0
        if was_powered and not powered:
            events.append({"name": "burnout", "time_s": time_s})
        was_on_rail, was_powered = diagnostics.on_rail, powered
        limits = settings.aerodynamic_limits
        # The rail supplies the reaction that keeps the vehicle aligned, and
        # aerodynamic angles are ill-conditioned near zero forward speed.
        envelope_applies = not diagnostics.on_rail and diagnostics.airspeed_mps >= 5.0
        exceeded = envelope_applies and (diagnostics.mach >= limits.maximum_mach or abs(diagnostics.angle_of_attack_rad) > limits.maximum_angle_of_attack_rad or abs(diagnostics.sideslip_rad) > limits.maximum_angle_of_attack_rad)
        if exceeded and not limits.diagnostic_continuation:
            events.append({"name": "aerodynamic_validity_exceeded", "time_s": time_s})
            status = "aerodynamic_validity_exceeded"
            break
        if not diagnostics.on_rail and time_s > inputs.motor.burnout_time_s and state[5] <= 0:
            events.append({"name": "apogee", "time_s": time_s})
            status = "apogee_reached"
            break
        state = _rk4_step(dynamics, time_s, state, dt)
    return SimulationResult(
        time_s=np.asarray([row[0] for row in rows]),
        state=np.asarray([row[1] for row in rows]),
        acceleration_world_mps2=np.asarray([row[2].acceleration_world_mps2 for row in rows]),
        mass_kg=np.asarray([row[2].mass.mass_kg for row in rows]),
        center_of_mass_station_m=np.asarray([row[2].mass.center_of_mass_station_m for row in rows]),
        thrust_n=np.asarray([row[2].thrust_n for row in rows]),
        airspeed_mps=np.asarray([row[2].airspeed_mps for row in rows]),
        mach=np.asarray([row[2].mach for row in rows]),
        angle_of_attack_rad=np.asarray([row[2].angle_of_attack_rad for row in rows]),
        sideslip_rad=np.asarray([row[2].sideslip_rad for row in rows]),
        dynamic_pressure_pa=np.asarray([row[2].dynamic_pressure_pa for row in rows]),
        drag_coefficient=np.asarray([row[2].drag_coefficient for row in rows]),
        on_rail=np.asarray([row[2].on_rail for row in rows]),
        fin_labels=dynamics.fin_labels,
        fin_deflection_rad=np.asarray([row[2].fin_deflection_rad for row in rows]),
        events=tuple(events),
        status=status,
    )


def write_simulation_outputs(inputs: SimulationInputs, result: SimulationResult, output_dir: str | Path, *, overwrite: bool = False) -> Path:
    output = Path(output_dir)
    if output.exists() and any(output.iterdir()):
        if not overwrite:
            raise FileExistsError(f"output directory is not empty: {output}; pass overwrite=True to replace it")
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)
    fields = ["time_s", "east_m", "north_m", "altitude_m", "velocity_east_mps", "velocity_north_mps", "velocity_up_mps", "acceleration_east_mps2", "acceleration_north_mps2", "acceleration_up_mps2", "quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z", "omega_x_radps", "omega_y_radps", "omega_z_radps", "mass_kg", "cg_station_m", "thrust_n", "airspeed_mps", "mach", "angle_of_attack_rad", "sideslip_rad", "dynamic_pressure_pa", "drag_coefficient", "on_rail", *(f"fin_{label}_deflection_rad" for label in result.fin_labels)]
    with (output / "timeseries.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        for index, time_s in enumerate(result.time_s):
            writer.writerow([time_s, *result.state[index, 0:6], *result.acceleration_world_mps2[index], *result.state[index, 6:13], result.mass_kg[index], result.center_of_mass_station_m[index], result.thrust_n[index], result.airspeed_mps[index], result.mach[index], result.angle_of_attack_rad[index], result.sideslip_rad[index], result.dynamic_pressure_pa[index], result.drag_coefficient[index], bool(result.on_rail[index]), *result.fin_deflection_rad[index]])
    manifest = {
        "status": result.status,
        "model": "subsonic_6dof_v1",
        "trusted_envelope": {"maximum_mach": inputs.scenario.simulation.aerodynamic_limits.maximum_mach, "maximum_angle_of_attack_rad": inputs.scenario.simulation.aerodynamic_limits.maximum_angle_of_attack_rad},
        "input_sha256": inputs.sha256,
        "time_step_s": inputs.scenario.simulation.integration.time_step_s,
        "events": list(result.events),
        "summary": {"maximum_altitude_m": float(np.max(result.altitude_m)), "maximum_speed_mps": float(np.max(np.linalg.norm(result.velocity_world_mps, axis=1))), "maximum_mach": float(np.max(result.mach)), "final_time_s": float(result.time_s[-1])},
    }
    (output / "run.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    _plot_result(result, output / "simulation_dashboard.png")
    return output


def _plot_result(result: SimulationResult, path: Path) -> None:
    mpl_config = Path(tempfile.gettempdir()) / "rocket_sim_mplconfig"
    mpl_config.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(mpl_config))
    import matplotlib.pyplot as plt

    time = result.time_s
    speed = np.linalg.norm(result.velocity_world_mps, axis=1)
    trusted_angles = (~result.on_rail) & (result.airspeed_mps >= 5.0)
    angles = np.degrees(np.column_stack((result.angle_of_attack_rad, result.sideslip_rad)))
    angles[~trusted_angles] = np.nan
    figure, axes = plt.subplots(3, 2, figsize=(13, 11), constrained_layout=True)
    figure.suptitle("YAML-driven 6-DoF passive-flight validation")
    axes[0, 0].plot(time, result.altitude_m, color="tab:blue")
    axes[0, 0].set(title="Altitude", xlabel="Time (s)", ylabel="Altitude ENU (m)")
    axes[0, 1].plot(time, speed, label="ground speed")
    axes[0, 1].plot(time, result.airspeed_mps, label="airspeed", linestyle="--")
    axes[0, 1].set(title="Speed", xlabel="Time (s)", ylabel="Speed (m/s)"); axes[0, 1].legend()
    thrust_axis = axes[1, 0]
    mass_axis = thrust_axis.twinx()
    thrust_axis.plot(time, result.thrust_n, label="thrust", color="tab:blue")
    mass_axis.plot(time, result.mass_kg, label="mass", color="tab:orange")
    thrust_axis.set(title="Propulsion and mass", xlabel="Time (s)", ylabel="Thrust (N)")
    mass_axis.set_ylabel("Mass (kg)")
    thrust_axis.legend(loc="upper left"); mass_axis.legend(loc="upper right")
    pressure_axis = axes[1, 1]
    mach_axis = pressure_axis.twinx()
    pressure_axis.plot(time, result.dynamic_pressure_pa / 1000, label="dynamic pressure", color="tab:blue")
    mach_axis.plot(time, result.mach, label="Mach", color="tab:orange")
    pressure_axis.set(title="Aerodynamic condition", xlabel="Time (s)", ylabel="Dynamic pressure (kPa)")
    mach_axis.set_ylabel("Mach")
    pressure_axis.legend(loc="upper left"); mach_axis.legend(loc="upper right")
    axes[2, 0].plot(time, angles[:, 0], label="angle of attack")
    axes[2, 0].plot(time, angles[:, 1], label="sideslip")
    axes[2, 0].axhline(10, color="tab:red", linestyle=":"); axes[2, 0].axhline(-10, color="tab:red", linestyle=":")
    axes[2, 0].set(title="Low-angle validity", xlabel="Time (s)", ylabel="Angle (deg)"); axes[2, 0].legend()
    axes[2, 1].plot(time, np.degrees(result.state[:, 10]), label="roll rate")
    axes[2, 1].plot(time, np.degrees(result.state[:, 11]), label="pitch rate")
    axes[2, 1].plot(time, np.degrees(result.state[:, 12]), label="yaw rate")
    axes[2, 1].set(title="Body angular rates", xlabel="Time (s)", ylabel="Rate (deg/s)")
    axes[2, 1].legend()
    if result.fin_labels:
        fin_axis = axes[2, 1].twinx()
        for index, label in enumerate(result.fin_labels):
            fin_axis.plot(time, np.degrees(result.fin_deflection_rad[:, index]), linestyle=":", linewidth=1, label=f"{label} angle")
        fin_axis.set_ylabel("Fin deflection (deg)")
        fin_axis.legend(loc="lower right", fontsize=7)
    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        for event in result.events:
            axis.axvline(float(event["time_s"]), color="0.55", linewidth=0.7, linestyle=":")
    figure.text(0.5, 0.005, f"Status: {result.status} · prototype model; validate before engineering use", ha="center", fontsize=9)
    figure.savefig(path, dpi=150)
    plt.close(figure)
