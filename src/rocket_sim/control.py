"""Synchronous control-development environment around the 6-DoF model."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
import math

import numpy as np

from .configuration import SimulationInputs
from .physics import Diagnostics, VehicleDynamics, quaternion_to_matrix


@dataclass(frozen=True)
class ControlTelemetry:
    time_s: float
    position_world_m: np.ndarray
    velocity_world_mps: np.ndarray
    velocity_body_mps: np.ndarray
    acceleration_world_mps2: np.ndarray
    acceleration_body_mps2: np.ndarray
    specific_force_body_mps2: np.ndarray
    attitude_quaternion_wxyz: np.ndarray
    attitude_rpy_rad: np.ndarray
    angular_velocity_body_radps: np.ndarray
    angular_acceleration_body_radps2: np.ndarray
    mass_kg: float
    thrust_n: float
    airspeed_mps: float
    mach: float
    angle_of_attack_rad: float
    sideslip_rad: float
    dynamic_pressure_pa: float
    on_rail: bool
    fin_deflection_rad: dict[str, float]


Controller = Callable[[ControlTelemetry], Mapping[str, float] | None]


def mix_x_fin_commands(
    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,
    *,
    maximum_abs_rad: float,
    fin_set_id: str = "rear",
) -> dict[str, float]:
    """Mix roll/pitch/yaw demands for fins at 45, 135, 225 and 315 degrees.

    If one output exceeds the limit, all outputs are scaled by the same factor,
    preserving the requested moment direction while respecting saturation.
    """
    values = (float(roll_rad), float(pitch_rad), float(yaw_rad), float(maximum_abs_rad))
    if not all(math.isfinite(value) for value in values) or maximum_abs_rad <= 0:
        raise ValueError("mixer inputs must be finite and maximum_abs_rad must be positive")
    raw = np.array(
        (
            roll_rad + pitch_rad + yaw_rad,
            roll_rad - pitch_rad + yaw_rad,
            roll_rad - pitch_rad - yaw_rad,
            roll_rad + pitch_rad - yaw_rad,
        ),
        dtype=float,
    )
    peak = float(np.max(np.abs(raw)))
    if peak > maximum_abs_rad:
        raw *= maximum_abs_rad / peak
    return {f"{fin_set_id}_{index + 1}": float(value) for index, value in enumerate(raw)}


def _roll_pitch_yaw(quaternion: np.ndarray) -> np.ndarray:
    rotation = quaternion_to_matrix(quaternion)
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return np.array((roll, pitch, yaw))


class ControlSimulation:
    """Advance physics one fixed step and invoke a controller at a fixed rate."""

    def __init__(self, inputs: SimulationInputs, controller: Controller, *, controller_rate_hz: float | None = None):
        self.inputs = inputs
        self.controller = controller
        self.dynamics = VehicleDynamics(inputs)
        self.physics_dt_s = inputs.scenario.simulation.integration.time_step_s
        self.controller_rate_hz = controller_rate_hz or inputs.scenario.simulation.integration.controller_rate_hz
        control_steps = 1.0 / (self.controller_rate_hz * self.physics_dt_s)
        if not math.isclose(control_steps, round(control_steps), rel_tol=0, abs_tol=1e-9):
            raise ValueError("controller period must be an integer number of physics steps")
        self._control_interval_steps = int(round(control_steps))
        self._step_index = 0
        self.state = self.dynamics.initial_state()
        self._targets = {label: 0.0 for label in self.dynamics.fin_labels}
        self._deflections = dict(self._targets)
        self._actuator_by_label = {
            f"{fin_set.id}_{index + 1}": self.dynamics.actuator_by_fin_set[fin_set.id]
            for fin_set in self.dynamics.rocket.fin_sets
            if fin_set.actuation.type != "fixed"
            for index in range(fin_set.count)
        }
        self._fin_set_by_label = {
            f"{fin_set.id}_{index + 1}": fin_set
            for fin_set in self.dynamics.rocket.fin_sets
            if fin_set.actuation.type != "fixed"
            for index in range(fin_set.count)
        }
        self._finished = False

    @property
    def time_s(self) -> float:
        return self._step_index * self.physics_dt_s

    @property
    def running(self) -> bool:
        return not self._finished

    def _telemetry(self, diagnostics: Diagnostics, derivative: np.ndarray) -> ControlTelemetry:
        rotation = quaternion_to_matrix(self.state[6:10])
        return ControlTelemetry(
            time_s=self.time_s,
            position_world_m=self.state[0:3].copy(),
            velocity_world_mps=self.state[3:6].copy(),
            velocity_body_mps=rotation.T @ self.state[3:6],
            acceleration_world_mps2=diagnostics.acceleration_world_mps2.copy(),
            acceleration_body_mps2=rotation.T @ diagnostics.acceleration_world_mps2,
            specific_force_body_mps2=diagnostics.force_body_n / diagnostics.mass.mass_kg,
            attitude_quaternion_wxyz=self.state[6:10].copy(),
            attitude_rpy_rad=_roll_pitch_yaw(self.state[6:10]),
            angular_velocity_body_radps=self.state[10:13].copy(),
            angular_acceleration_body_radps2=derivative[10:13].copy(),
            mass_kg=diagnostics.mass.mass_kg,
            thrust_n=diagnostics.thrust_n,
            airspeed_mps=diagnostics.airspeed_mps,
            mach=diagnostics.mach,
            angle_of_attack_rad=diagnostics.angle_of_attack_rad,
            sideslip_rad=diagnostics.sideslip_rad,
            dynamic_pressure_pa=diagnostics.dynamic_pressure_pa,
            on_rail=diagnostics.on_rail,
            fin_deflection_rad=dict(self._deflections),
        )

    def _accept_commands(self, commands: Mapping[str, float] | None) -> None:
        if commands is None:
            return
        unknown = set(commands) - set(self._targets)
        if unknown:
            raise ValueError(f"controller returned unknown fins: {', '.join(sorted(unknown))}")
        for label, value in commands.items():
            target = float(value)
            fin_set = self._fin_set_by_label[label]
            minimum = fin_set.actuation.minimum_deflection_rad
            maximum = fin_set.actuation.maximum_deflection_rad
            if not math.isfinite(target) or not minimum <= target <= maximum:
                raise ValueError(f"controller target for '{label}' exceeds its deflection limits")
            self._targets[label] = target

    def _move_actuators(self) -> None:
        for label, target in self._targets.items():
            rate = self._actuator_by_label[label].maximum_rate_radps
            self._deflections[label] = self.dynamics._slew(self._deflections[label], target, rate, self.physics_dt_s)

    def _rk4_step(self) -> np.ndarray:
        time_s, state, dt = self.time_s, self.state, self.physics_dt_s

        def derivative(time: float, vector: np.ndarray) -> np.ndarray:
            return self.dynamics.evaluate(time, vector, self._deflections)[0]

        k1 = derivative(time_s, state)
        k2 = derivative(time_s + dt / 2, state + dt * k1 / 2)
        k3 = derivative(time_s + dt / 2, state + dt * k2 / 2)
        k4 = derivative(time_s + dt, state + dt * k3)
        updated = state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6
        updated[6:10] /= np.linalg.norm(updated[6:10])
        return updated

    def step(self) -> ControlTelemetry:
        """Advance one physics step and return telemetry at the new time."""
        if self._finished:
            raise RuntimeError("simulation has finished")
        derivative, diagnostics = self.dynamics.evaluate(self.time_s, self.state, self._deflections)
        if self._step_index % self._control_interval_steps == 0:
            self._accept_commands(self.controller(self._telemetry(diagnostics, derivative)))
        self._move_actuators()
        self.state = self._rk4_step()
        self._step_index += 1
        derivative, diagnostics = self.dynamics.evaluate(self.time_s, self.state, self._deflections)
        settings = self.inputs.scenario.simulation
        if self.time_s >= settings.maximum_duration_s:
            self._finished = True
        elif not diagnostics.on_rail and self.time_s > self.inputs.motor.burnout_time_s and self.state[5] <= 0:
            self._finished = True
        return self._telemetry(diagnostics, derivative)
