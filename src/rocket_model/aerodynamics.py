"""Model-level low-angle aerodynamic property calculations."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .barrowman import calculate_barrowman
from .models import RocketDefinition


@dataclass(frozen=True)
class ComponentAreas:
    name: str
    wetted_area_m2: float
    reference_area_m2: float


@dataclass(frozen=True)
class AerodynamicForce:
    name: str
    application_point_body_m: tuple[float, float, float]
    drag_n: float
    normal_force_body_n: tuple[float, float, float]


def rocket_reference_area_m2(definition: RocketDefinition) -> float:
    return math.pi * (definition.rocket.body.diameter_m / 2) ** 2


def component_areas(definition: RocketDefinition) -> tuple[ComponentAreas, ...]:
    rocket = definition.rocket
    radius = rocket.body.diameter_m / 2
    body = ComponentAreas("body_tube", math.pi * rocket.body.diameter_m * rocket.body.length_m, rocket_reference_area_m2(definition))
    # Half of a prolate spheroid excluding its circular base.
    nose_length = rocket.nose.length_m
    eccentricity = math.sqrt(max(0.0, 1 - (radius / nose_length) ** 2)) if nose_length > radius else 0.0
    nose_wetted = 2 * math.pi * radius * radius if eccentricity < 1e-12 else math.pi * radius * radius * (1 + nose_length * math.asin(eccentricity) / (radius * eccentricity))
    nose = ComponentAreas("nose", nose_wetted, rocket_reference_area_m2(definition))
    fins = []
    for fin_set in rocket.fin_sets:
        geometry = fin_set.geometry
        planform_per_fin = (geometry.root_chord_m + geometry.tip_chord_m) * geometry.span_m / 2
        perimeter = geometry.root_chord_m + geometry.tip_chord_m + geometry.span_m + math.hypot(geometry.span_m, geometry.sweep_m + geometry.tip_chord_m - geometry.root_chord_m)
        fins.append(ComponentAreas(fin_set.id, fin_set.count * (2 * planform_per_fin + perimeter * geometry.thickness_m), fin_set.count * planform_per_fin))
    return (body, nose, *fins)


def drag_force_n(dynamic_pressure_pa: float, skin_friction_coefficient: float, wetted_area_m2: float, pressure_drag_coefficient: float, reference_area_m2: float) -> float:
    return dynamic_pressure_pa * (skin_friction_coefficient * wetted_area_m2 + pressure_drag_coefficient * reference_area_m2)


def low_angle_forces(
    definition: RocketDefinition,
    air_velocity_body_mps: tuple[float, float, float],
    air_density_kg_m3: float,
    maximum_angle_of_attack_rad: float = math.radians(10),
) -> tuple[AerodynamicForce, ...]:
    """Return component forces for the Barrowman low-angle subsonic model.

    `air_velocity_body_mps` is vehicle velocity relative to air, expressed in
    the Body Frame. Drag opposes it; normal force opposes its lateral component.
    """
    velocity = np.asarray(air_velocity_body_mps, dtype=float)
    speed = float(np.linalg.norm(velocity))
    if speed < 1e-9:
        return ()
    dynamic_pressure = 0.5 * air_density_kg_m3 * speed * speed
    drag_direction = -velocity / speed
    lateral = velocity.copy(); lateral[0] = 0
    lateral_speed = float(np.linalg.norm(lateral))
    # Barrowman is a linear, small-angle model. It has no valid broadside-flow
    # prediction, so omit normal force outside the configured model range.
    raw_alpha = math.atan2(lateral_speed, max(abs(float(velocity[0])), 1e-9))
    alpha = raw_alpha if raw_alpha <= maximum_angle_of_attack_rad else 0.0
    normal_direction = -lateral / lateral_speed if lateral_speed > 1e-9 else np.zeros(3)
    area_by_name = {area.name: area for area in component_areas(definition)}
    rocket = definition.rocket
    barrowman = calculate_barrowman(definition)
    components: list[AerodynamicForce] = []
    for name, properties, point, slope in (
        ("body_tube", rocket.body.aerodynamics, (rocket.body.length_m / 2, 0.0, 0.0), 0.0),
        ("nose", rocket.nose.aerodynamics, (barrowman.nose.cp_station_m, 0.0, 0.0), barrowman.nose.normal_force_slope_per_rad),
    ):
        areas = area_by_name[name]
        drag = drag_force_n(dynamic_pressure, properties.skin_friction_coefficient, areas.wetted_area_m2, properties.pressure_drag_coefficient, areas.reference_area_m2)
        normal = dynamic_pressure * rocket_reference_area_m2(definition) * slope * alpha * normal_direction
        components.append(AerodynamicForce(name, point, drag, tuple(float(value) for value in normal)))
    slopes = {entry.name: entry for entry in barrowman.fin_sets}
    for fin_set in rocket.fin_sets:
        areas = area_by_name[fin_set.id]
        drag = drag_force_n(dynamic_pressure, fin_set.aerodynamics.skin_friction_coefficient, areas.wetted_area_m2, fin_set.aerodynamics.pressure_drag_coefficient, areas.reference_area_m2)
        slope = slopes[fin_set.id].normal_force_slope_per_rad
        normal = dynamic_pressure * rocket_reference_area_m2(definition) * slope * alpha * normal_direction
        components.append(AerodynamicForce(fin_set.id, (slopes[fin_set.id].cp_station_m, 0.0, 0.0), drag, tuple(float(value) for value in normal)))
    return tuple(components)
