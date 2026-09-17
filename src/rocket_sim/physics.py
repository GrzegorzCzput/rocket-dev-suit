"""Body-FLU rigid-body physics for Flight Simulation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from rocket_model.aerodynamics import component_areas, rocket_reference_area_m2
from rocket_model.barrowman import calculate_barrowman, fin_cp_span_m
from .configuration import SimulationInputs


GRAVITY_MPS2 = 9.80665


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = left
    w2, x2, y2, z2 = right
    return np.array((
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ))


def quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion, dtype=float) / np.linalg.norm(quaternion)
    return np.array((
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    ))


def quaternion_from_matrix(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = 2 * math.sqrt(trace + 1)
        result = np.array((0.25 * scale, (matrix[2, 1] - matrix[1, 2]) / scale, (matrix[0, 2] - matrix[2, 0]) / scale, (matrix[1, 0] - matrix[0, 1]) / scale))
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = 2 * math.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            result = np.array(((matrix[2, 1] - matrix[1, 2]) / scale, 0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale))
        elif index == 1:
            scale = 2 * math.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            result = np.array(((matrix[0, 2] - matrix[2, 0]) / scale, (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale, (matrix[1, 2] + matrix[2, 1]) / scale))
        else:
            scale = 2 * math.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            result = np.array(((matrix[1, 0] - matrix[0, 1]) / scale, (matrix[0, 2] + matrix[2, 0]) / scale, (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale))
    return result / np.linalg.norm(result)


@dataclass(frozen=True)
class AtmosphereState:
    density_kg_m3: float
    speed_of_sound_mps: float
    dynamic_viscosity_pa_s: float


def isa1976(altitude_m: float) -> AtmosphereState:
    """ISA troposphere, sufficient for the first model-rocket ascent slice."""
    altitude = max(0.0, min(float(altitude_m), 11000.0))
    temperature = 288.15 - 0.0065 * altitude
    pressure = 101325.0 * (temperature / 288.15) ** (GRAVITY_MPS2 / (287.05287 * 0.0065))
    density = pressure / (287.05287 * temperature)
    speed_of_sound = math.sqrt(1.4 * 287.05287 * temperature)
    viscosity = 1.458e-6 * temperature ** 1.5 / (temperature + 110.4)
    return AtmosphereState(density, speed_of_sound, viscosity)


@dataclass(frozen=True)
class MassState:
    mass_kg: float
    center_of_mass_station_m: float
    inertia_kg_m2: np.ndarray


@dataclass(frozen=True)
class Diagnostics:
    acceleration_world_mps2: np.ndarray
    force_body_n: np.ndarray
    moment_body_nm: np.ndarray
    mass: MassState
    thrust_n: float
    airspeed_mps: float
    mach: float
    angle_of_attack_rad: float
    sideslip_rad: float
    dynamic_pressure_pa: float
    drag_coefficient: float
    on_rail: bool
    fin_deflection_rad: tuple[float, ...]


class VehicleDynamics:
    def __init__(self, inputs: SimulationInputs):
        self.inputs = inputs
        self.rocket = inputs.rocket.rocket
        scenario = inputs.scenario.simulation
        rail = scenario.launch_rail
        horizontal = math.cos(rail.inclination_rad)
        self.rail_direction = np.array((horizontal * math.cos(rail.heading_rad), horizontal * math.sin(rail.heading_rad), math.sin(rail.inclination_rad)))
        self.rail_origin = np.asarray(rail.position_world_m, dtype=float)
        left = np.array((-math.sin(rail.heading_rad), math.cos(rail.heading_rad), 0.0))
        body_up = np.cross(self.rail_direction, left)
        body_up /= np.linalg.norm(body_up)
        self.initial_quaternion = quaternion_from_matrix(np.column_stack((self.rail_direction, left, body_up)))
        self.reference_area = rocket_reference_area_m2(inputs.rocket)
        self.areas = component_areas(inputs.rocket)
        self.barrowman = calculate_barrowman(inputs.rocket)
        self.actuator_by_fin_set = {item.fin_set_id: item for item in self.rocket.flight.actuators}
        self.commands_by_fin_set = {
            fin_set.id: tuple(command for command in scenario.fin_commands if command.fin_set_id == fin_set.id)
            for fin_set in self.rocket.fin_sets
        }
        self.fin_labels = tuple(
            f"{fin_set.id}_{index + 1}"
            for fin_set in self.rocket.fin_sets
            if fin_set.actuation.type != "fixed"
            for index in range(fin_set.count)
        )

    @staticmethod
    def _slew(current: float, target: float, maximum_rate: float, elapsed: float) -> float:
        change = maximum_rate * max(elapsed, 0.0)
        return current + float(np.clip(target - current, -change, change))

    def fin_set_deflection(self, fin_set_id: str, time_s: float) -> float:
        """Return the scheduled, rate-limited common deflection for a Fin Set."""
        actuator = self.actuator_by_fin_set.get(fin_set_id)
        if actuator is None:
            return 0.0
        actual = 0.0
        target = 0.0
        previous_time = 0.0
        for command in self.commands_by_fin_set[fin_set_id]:
            if command.time_s > time_s:
                break
            actual = self._slew(actual, target, actuator.maximum_rate_radps, command.time_s - previous_time)
            target = command.deflection_rad
            previous_time = command.time_s
        return self._slew(actual, target, actuator.maximum_rate_radps, time_s - previous_time)

    def fin_deflections(self, time_s: float) -> tuple[float, ...]:
        return tuple(
            self.fin_set_deflection(fin_set.id, time_s)
            for fin_set in self.rocket.fin_sets
            if fin_set.actuation.type != "fixed"
            for _ in range(fin_set.count)
        )

    def mass_state(self, time_s: float) -> MassState:
        dry = self.rocket.dry_mass_properties
        motor = self.inputs.motor.sample(time_s)
        total = dry.mass_kg + motor.propellant_mass_kg
        cg = (dry.mass_kg * dry.center_of_mass_station_m + motor.propellant_mass_kg * motor.propellant_cg_station_m) / total
        dry_offset = dry.center_of_mass_station_m - cg
        motor_offset = motor.propellant_cg_station_m - cg
        inertia = np.array((
            dry.inertia_xx_kg_m2 + motor.inertia_kg_m2[0],
            dry.inertia_yy_kg_m2 + dry.mass_kg * dry_offset**2 + motor.inertia_kg_m2[1] + motor.propellant_mass_kg * motor_offset**2,
            dry.inertia_zz_kg_m2 + dry.mass_kg * dry_offset**2 + motor.inertia_kg_m2[2] + motor.propellant_mass_kg * motor_offset**2,
        ))
        return MassState(total, cg, inertia)

    def _drag_coefficient(self, speed: float, atmosphere: AtmosphereState, powered: bool) -> float:
        if speed < 1e-6:
            return 0.0
        length = self.rocket.body.length_m + self.rocket.nose.length_m
        reynolds = max(atmosphere.density_kg_m3 * speed * length / atmosphere.dynamic_viscosity_pa_s, 1.0)
        skin_friction = 1.328 / math.sqrt(reynolds) if reynolds < 5e5 else 0.074 / reynolds**0.2
        wetted = sum(area.wetted_area_m2 for area in self.areas)
        authored_area = 0.0
        properties = [self.rocket.body.aerodynamics, self.rocket.nose.aerodynamics, *(fin.aerodynamics for fin in self.rocket.fin_sets)]
        for area, prop in zip(self.areas, properties):
            authored_area += prop.skin_friction_coefficient * area.wetted_area_m2 + prop.pressure_drag_coefficient * area.reference_area_m2
        baseline = skin_friction * wetted / self.reference_area + authored_area / self.reference_area + (0.04 if powered else 0.12)
        calibration = self.rocket.flight.aerodynamics
        return baseline * (calibration.powered_drag_multiplier if powered else calibration.coast_drag_multiplier)

    def _aerodynamics(self, time_s: float, position: np.ndarray, velocity: np.ndarray, rotation: np.ndarray, omega: np.ndarray, cg: float, powered: bool, fin_deflections: dict[str, float] | None = None) -> tuple[np.ndarray, np.ndarray, tuple[float, ...]]:
        atmosphere = isa1976(position[2])
        wind = np.asarray(self.inputs.scenario.simulation.atmosphere.wind_world_mps)
        air_velocity_body = rotation.T @ (velocity - wind)
        speed = float(np.linalg.norm(air_velocity_body))
        q = 0.5 * atmosphere.density_kg_m3 * speed**2
        alpha = math.atan2(float(air_velocity_body[2]), max(abs(float(air_velocity_body[0])), 1e-9)) if speed > 1e-9 else 0.0
        beta = math.atan2(float(air_velocity_body[1]), max(abs(float(air_velocity_body[0])), 1e-9)) if speed > 1e-9 else 0.0
        cd = self._drag_coefficient(speed, atmosphere, powered)
        force = -q * self.reference_area * cd * air_velocity_body / speed if speed > 1e-9 else np.zeros(3)
        moment = np.zeros(3)
        nose = self.barrowman.nose
        arm = np.array((nose.cp_station_m - cg, 0.0, 0.0))
        local_velocity = air_velocity_body + np.cross(omega, arm)
        local_speed = float(np.linalg.norm(local_velocity))
        if local_speed >= 1e-9:
            local_q = 0.5 * atmosphere.density_kg_m3 * local_speed**2
            lateral = np.array((0.0, local_velocity[1], local_velocity[2]))
            lateral_angle = math.atan2(float(np.linalg.norm(lateral)), max(abs(float(local_velocity[0])), 1e-9))
            normal = -local_q * self.reference_area * nose.normal_force_slope_per_rad * lateral_angle * lateral / max(float(np.linalg.norm(lateral)), 1e-12)
            force += normal
            moment += np.cross(arm, normal)

        slopes = {entry.name: entry for entry in self.barrowman.fin_sets}
        body_radius = self.rocket.body.diameter_m / 2
        calibration = self.rocket.flight.aerodynamics.control_effectiveness_multiplier
        for fin_set in self.rocket.fin_sets:
            properties = slopes[fin_set.id]
            individual_slope = 2 * properties.normal_force_slope_per_rad / fin_set.count
            scheduled = self.fin_set_deflection(fin_set.id, time_s)
            radial_cp = body_radius + fin_cp_span_m(fin_set)
            for index in range(fin_set.count):
                label = f"{fin_set.id}_{index + 1}"
                commanded = scheduled if fin_deflections is None else fin_deflections.get(label, 0.0)
                effective_deflection = commanded * fin_set.control_effectiveness * calibration
                angle = fin_set.angular_offset_rad + index * 2 * math.pi / fin_set.count
                radial = np.array((0.0, math.cos(angle), math.sin(angle)))
                surface_normal = np.array((0.0, -math.sin(angle), math.cos(angle)))
                arm = np.array((properties.cp_station_m - cg, radial_cp * radial[1], radial_cp * radial[2]))
                local_velocity = air_velocity_body + np.cross(omega, arm)
                local_speed = float(np.linalg.norm(local_velocity))
                if local_speed < 1e-9:
                    continue
                local_q = 0.5 * atmosphere.density_kg_m3 * local_speed**2
                incidence = math.atan2(float(local_velocity @ surface_normal), max(abs(float(local_velocity[0])), 1e-9)) + effective_deflection
                normal = -local_q * self.reference_area * individual_slope * incidence * surface_normal
                force += normal
                moment += np.cross(arm, normal)
        mach = speed / atmosphere.speed_of_sound_mps
        return force, moment, (speed, mach, alpha, beta, q, cd)

    def evaluate(self, time_s: float, vector: np.ndarray, fin_deflections: dict[str, float] | None = None) -> tuple[np.ndarray, Diagnostics]:
        position, velocity = vector[0:3], vector[3:6]
        quaternion, omega = vector[6:10], vector[10:13]
        rotation = quaternion_to_matrix(quaternion)
        mass = self.mass_state(time_s)
        motor = self.inputs.motor.sample(time_s)
        powered = motor.thrust_n > 0
        aero_force, moment, aero = self._aerodynamics(time_s, position, velocity, rotation, omega, mass.center_of_mass_station_m, powered, fin_deflections)
        force_body = aero_force + np.array((motor.thrust_n, 0.0, 0.0))
        acceleration = rotation @ force_body / mass.mass_kg + np.array((0.0, 0.0, -GRAVITY_MPS2))
        rail_distance = float((position - self.rail_origin) @ self.rail_direction)
        on_rail = rail_distance < self.inputs.scenario.simulation.launch_rail.length_m
        if on_rail:
            velocity = self.rail_direction * max(0.0, float(velocity @ self.rail_direction))
            acceleration = self.rail_direction * max(0.0, float(acceleration @ self.rail_direction))
            derivative = np.r_[velocity, acceleration, np.zeros(4), np.zeros(3)]
            moment = np.zeros(3)
        else:
            inertia = mass.inertia_kg_m2
            epsilon = min(1e-3, max(time_s, 1e-3))
            previous = self.mass_state(max(0.0, time_s - epsilon)).inertia_kg_m2
            following = self.mass_state(time_s + epsilon).inertia_kg_m2
            inertia_dot = (following - previous) / (time_s + epsilon - max(0.0, time_s - epsilon))
            angular_acceleration = (moment - np.cross(omega, inertia * omega) - inertia_dot * omega) / inertia
            quaternion_dot = 0.5 * quaternion_multiply(quaternion, np.r_[0.0, omega])
            derivative = np.r_[velocity, acceleration, quaternion_dot, angular_acceleration]
        reported_deflections = self.fin_deflections(time_s) if fin_deflections is None else tuple(fin_deflections[label] for label in self.fin_labels)
        diagnostics = Diagnostics(acceleration, force_body, moment, mass, motor.thrust_n, *aero, on_rail, reported_deflections)
        return derivative, diagnostics

    def initial_state(self) -> np.ndarray:
        return np.r_[self.rail_origin, np.zeros(3), self.initial_quaternion, np.zeros(3)]
