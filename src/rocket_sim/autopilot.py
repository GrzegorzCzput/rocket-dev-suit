"""Acceleration to body-rate to fin-actuator autopilot."""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from .configuration import SimulationInputs
from .control import ControlTelemetry, mix_x_fin_commands


AccelerationCommand = Callable[[float], np.ndarray]


class ThreeLoopAutopilot:
    """Track Body-FLU lateral acceleration through three control loops.

    The input is `[forward, left, up]` acceleration in m/s². Loop 1 converts
    lateral acceleration error to pitch/yaw rate demands. Loop 2 converts rate
    error to fin demand. Loop 3 is the rate/position-limited physical actuator
    already implemented by ``ControlSimulation``.
    """

    def __init__(
        self,
        inputs: SimulationInputs,
        command: AccelerationCommand,
        *,
        k_accel_p: float = 0.04,
        k_accel_i: float = 1.2,
        k_rate_p: float = 0.08,
        k_roll_rate: float = 0.001,
        maximum_lateral_rate_radps: float = 5.5,
        maximum_accel_integral_mps: float = 20.0,
        maximum_fin_deflection_rad: float | None = None,
        control_start_time_s: float = 0.35,
        minimum_control_airspeed_mps: float = 20.0,
        gain_scheduling: bool = True,
        schedule_reference_q_pa: float = 2030.0,
        schedule_min_scale: float = 0.15,
        schedule_max_scale: float = 5.0,
        schedule_min_q_pa: float = 250.0,
    ):
        self.inputs = inputs
        self.command = command
        self.k_accel_p = k_accel_p
        self.k_accel_i = k_accel_i
        self.k_rate_p = k_rate_p
        self.k_roll_rate = k_roll_rate
        self.maximum_lateral_rate_radps = maximum_lateral_rate_radps
        self.maximum_accel_integral_mps = maximum_accel_integral_mps
        self.control_start_time_s = control_start_time_s
        self.minimum_control_airspeed_mps = minimum_control_airspeed_mps
        self.gain_scheduling = gain_scheduling
        self.schedule_reference_q_pa = schedule_reference_q_pa
        self.schedule_min_scale = schedule_min_scale
        self.schedule_max_scale = schedule_max_scale
        self.schedule_min_q_pa = schedule_min_q_pa
        self.controller_dt_s = 1.0 / inputs.scenario.simulation.integration.controller_rate_hz
        movable = [fin_set for fin_set in inputs.rocket.rocket.fin_sets if fin_set.actuation.type != "fixed"]
        if len(movable) != 1 or movable[0].count != 4:
            raise ValueError("ThreeLoopAutopilot requires one movable four-fin X set")
        fin_set = movable[0]
        self.fin_set_id = fin_set.id
        physical_limit = min(abs(fin_set.actuation.minimum_deflection_rad), abs(fin_set.actuation.maximum_deflection_rad))
        self.maximum_fin_deflection_rad = physical_limit if maximum_fin_deflection_rad is None else min(maximum_fin_deflection_rad, physical_limit)
        self.accel_integral = np.zeros(2)
        self.acceleration_target_body_mps2 = np.zeros(3)
        self.pitch_rate_command_radps = 0.0
        self.yaw_rate_command_radps = 0.0
        self.rate_gain_scale = 1.0
        self.effective_rate_gain = self.k_rate_p
        self.enabled = False
        self.saturated = False

    def _gain_scale(self, dynamic_pressure_pa: float) -> float:
        if not self.gain_scheduling:
            return 1.0
        pressure = max(float(dynamic_pressure_pa), self.schedule_min_q_pa)
        return float(np.clip(self.schedule_reference_q_pa / pressure, self.schedule_min_scale, self.schedule_max_scale))

    def __call__(self, data: ControlTelemetry) -> dict[str, float]:
        target = np.asarray(self.command(data.time_s), dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("autopilot command must be three finite Body-FLU accelerations")
        self.acceleration_target_body_mps2 = target
        self.rate_gain_scale = self._gain_scale(data.dynamic_pressure_pa)
        self.effective_rate_gain = self.k_rate_p * self.rate_gain_scale
        self.enabled = bool(data.time_s >= self.control_start_time_s and data.airspeed_mps >= self.minimum_control_airspeed_mps)
        if not self.enabled:
            self.accel_integral[:] = 0.0
            self.pitch_rate_command_radps = 0.0
            self.yaw_rate_command_radps = 0.0
            self.saturated = False
            return mix_x_fin_commands(0.0, 0.0, 0.0, maximum_abs_rad=self.maximum_fin_deflection_rad, fin_set_id=self.fin_set_id)

        # Loop 1: lateral acceleration PI -> pitch/yaw rate demand. In Body-FLU
        # coordinates, +left acceleration uses +yaw and +up uses -pitch.
        acceleration_error = target[1:3] - data.acceleration_body_mps2[1:3]
        previous_integral = self.accel_integral.copy()
        self.accel_integral = np.clip(
            self.accel_integral + acceleration_error * self.controller_dt_s,
            -self.maximum_accel_integral_mps,
            self.maximum_accel_integral_mps,
        )
        lateral_rate_demand = self.k_accel_p * acceleration_error + self.k_accel_i * self.accel_integral
        self.yaw_rate_command_radps = float(np.clip(lateral_rate_demand[0], -self.maximum_lateral_rate_radps, self.maximum_lateral_rate_radps))
        self.pitch_rate_command_radps = float(np.clip(-lateral_rate_demand[1], -self.maximum_lateral_rate_radps, self.maximum_lateral_rate_radps))

        # Loop 2: rate error -> virtual fin axes. Positive mixer input creates
        # a negative body moment for the aft-fin Body-FLU convention.
        roll_control = self.k_roll_rate * data.angular_velocity_body_radps[0]
        pitch_control = self.effective_rate_gain * (data.angular_velocity_body_radps[1] - self.pitch_rate_command_radps)
        yaw_control = self.effective_rate_gain * (data.angular_velocity_body_radps[2] - self.yaw_rate_command_radps)
        raw = np.array((
            roll_control + pitch_control + yaw_control,
            roll_control - pitch_control + yaw_control,
            roll_control - pitch_control - yaw_control,
            roll_control + pitch_control - yaw_control,
        ))
        self.saturated = bool(np.max(np.abs(raw)) > self.maximum_fin_deflection_rad)
        if self.saturated:
            self.accel_integral = previous_integral
        return mix_x_fin_commands(
            roll_control,
            pitch_control,
            yaw_control,
            maximum_abs_rad=self.maximum_fin_deflection_rad,
            fin_set_id=self.fin_set_id,
        )


# Compatibility with the name used by the first control demo.
ThreeAxisAutopilot = ThreeLoopAutopilot
