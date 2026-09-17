"""Neutral-geometry Barrowman stability calculations."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .models import FinSet, RocketDefinition, fin_cp_station


@dataclass(frozen=True)
class ComponentResult:
    name: str
    cp_station_m: float
    normal_force_slope_per_rad: float


@dataclass(frozen=True)
class BarrowmanResult:
    nose: ComponentResult
    fin_sets: tuple[ComponentResult, ...]
    body_tube_cp_station_m: None = None
    rocket_cp_station_m: float | None = None


def _fin_normal_force_slope(fin_set: FinSet, diameter_m: float) -> float:
    geometry = fin_set.geometry
    mid_chord_sweep = geometry.sweep_m + 0.5 * (geometry.tip_chord_m - geometry.root_chord_m)
    sweep_angle = math.atan2(mid_chord_sweep, geometry.span_m)
    mean_chord_sweep = geometry.span_m / math.cos(sweep_angle)
    raw = 4 * fin_set.count * (geometry.span_m / diameter_m) ** 2 / (1 + math.sqrt(1 + (2 * mean_chord_sweep / (geometry.root_chord_m + geometry.tip_chord_m)) ** 2))
    interference_factor = 1.0 if fin_set.count in (3, 4) else 0.5
    radius = diameter_m / 2
    body_interference = 1 + interference_factor * radius / (geometry.span_m + radius)
    return raw * body_interference


def calculate_barrowman(definition: RocketDefinition) -> BarrowmanResult:
    rocket = definition.rocket
    nose = ComponentResult("nose", rocket.body.length_m + 2 * rocket.nose.length_m / 3, 2.0)
    fin_results = tuple(ComponentResult(fin_set.id, fin_cp_station(fin_set), _fin_normal_force_slope(fin_set, rocket.body.diameter_m)) for fin_set in rocket.fin_sets)
    contributors = (nose, *fin_results)
    denominator = sum(item.normal_force_slope_per_rad for item in contributors)
    total_cp = sum(item.cp_station_m * item.normal_force_slope_per_rad for item in contributors) / denominator if denominator else None
    return BarrowmanResult(nose=nose, fin_sets=fin_results, rocket_cp_station_m=total_cp)


def fin_cp_span_m(fin_set: FinSet) -> float:
    geometry = fin_set.geometry
    return geometry.span_m * (geometry.root_chord_m + 2 * geometry.tip_chord_m) / (3 * (geometry.root_chord_m + geometry.tip_chord_m))


def static_margin_calibers(definition: RocketDefinition, result: BarrowmanResult | None = None) -> float | None:
    result = result or calculate_barrowman(definition)
    if result.rocket_cp_station_m is None:
        return None
    center_of_mass = definition.rocket.mass_properties.center_of_mass_station_m
    return (center_of_mass - result.rocket_cp_station_m) / definition.rocket.body.diameter_m
