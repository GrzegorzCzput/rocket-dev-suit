"""Validated domain models for reusable rocket definitions."""

from __future__ import annotations

import math
import re
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", validate_assignment=True, allow_inf_nan=False)


class Body(StrictModel):
    length_m: float = Field(gt=0)
    diameter_m: float = Field(gt=0)
    aerodynamics: "ComponentAerodynamics" = Field(default_factory=lambda: ComponentAerodynamics())


class Nose(StrictModel):
    type: Literal["half_ellipsoid"]
    length_m: float = Field(gt=0)
    aerodynamics: "ComponentAerodynamics" = Field(default_factory=lambda: ComponentAerodynamics())


class ComponentAerodynamics(StrictModel):
    skin_friction_coefficient: float = Field(default=0.0, ge=0)
    pressure_drag_coefficient: float = Field(default=0.0, ge=0)


class FinGeometry(StrictModel):
    root_chord_m: float = Field(gt=0)
    tip_chord_m: float = Field(gt=0)
    span_m: float = Field(gt=0)
    sweep_m: float = Field(ge=0)
    thickness_m: float = Field(gt=0)


class Actuation(StrictModel):
    type: Literal["fixed", "all_moving", "control_surface"]
    span_start_fraction: float | None = None
    hinge_fraction: float | None = None
    minimum_deflection_rad: float | None = None
    maximum_deflection_rad: float | None = None

    @model_validator(mode="after")
    def validate_mode(self) -> "Actuation":
        if self.type == "fixed":
            if any(value is not None for value in (self.span_start_fraction, self.hinge_fraction, self.minimum_deflection_rad, self.maximum_deflection_rad)):
                raise ValueError("fixed actuation cannot define movable-surface fields")
            return self
        if self.minimum_deflection_rad is None or self.maximum_deflection_rad is None:
            raise ValueError("movable actuation requires minimum and maximum deflection")
        if not math.isfinite(self.minimum_deflection_rad) or not math.isfinite(self.maximum_deflection_rad):
            raise ValueError("deflection limits must be finite")
        if self.minimum_deflection_rad > self.maximum_deflection_rad or not (self.minimum_deflection_rad <= 0 <= self.maximum_deflection_rad):
            raise ValueError("deflection limits must contain zero")
        if self.type == "all_moving":
            if self.span_start_fraction is not None or self.hinge_fraction is not None:
                raise ValueError("all_moving actuation cannot define control-surface fractions")
        else:
            if self.span_start_fraction is None or not (0 <= self.span_start_fraction < 1):
                raise ValueError("span_start_fraction must be in [0, 1)")
            if self.hinge_fraction is None or not (0 < self.hinge_fraction < 1):
                raise ValueError("hinge_fraction must be in (0, 1)")
        return self


class FinSet(StrictModel):
    id: str = Field(pattern=r"^[a-z][a-z0-9_\-]*$")
    station_m: float = Field(ge=0)
    count: Literal[3, 4, 6]
    angular_offset_rad: float = Field(ge=0, lt=2 * math.pi)
    geometry: FinGeometry
    actuation: Actuation
    aerodynamics: ComponentAerodynamics = Field(default_factory=ComponentAerodynamics)
    control_effectiveness: float = Field(default=0.7, ge=0, le=1)


class MassProperties(StrictModel):
    mass_kg: float = Field(gt=0)
    center_of_mass_station_m: float = Field(ge=0)
    inertia_xx_kg_m2: float = Field(gt=0)
    inertia_yy_kg_m2: float = Field(gt=0)
    inertia_zz_kg_m2: float = Field(gt=0)


class Rocket(StrictModel):
    id: str = Field(pattern=r"^[a-z][a-z0-9_\-]*$")
    name: str = Field(min_length=1)
    body: Body
    nose: Nose
    mass_properties: MassProperties
    fin_sets: list[FinSet] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_mass_properties(self) -> "Rocket":
        if self.mass_properties.center_of_mass_station_m > self.body.length_m + self.nose.length_m:
            raise ValueError("Center of Mass Station must stay between the Aft Datum and nose tip")
        return self


class RocketDefinition(StrictModel):
    schema_version: Literal[1]
    rocket: Rocket

    @model_validator(mode="after")
    def validate_geometry(self) -> "RocketDefinition":
        rocket = self.rocket
        identifiers = [fin_set.id for fin_set in rocket.fin_sets]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("Fin Set identifiers must be unique")
        body_end = rocket.body.length_m
        body_radius = rocket.body.diameter_m / 2
        for fin_set in rocket.fin_sets:
            geometry = fin_set.geometry
            root_leading = fin_set.station_m + geometry.root_chord_m
            tip_leading = root_leading - geometry.sweep_m
            tip_trailing = tip_leading - geometry.tip_chord_m
            if min(fin_set.station_m, tip_leading, tip_trailing) < 0 or max(root_leading, tip_leading) > body_end:
                raise ValueError(f"Fin Set '{fin_set.id}' must stay inside the body envelope")
            if geometry.span_m <= 0 or body_radius + geometry.span_m <= body_radius:
                raise ValueError(f"Fin Set '{fin_set.id}' must have positive radial span")
            if fin_set.actuation.type == "all_moving":
                cp_station = fin_set.station_m + geometry.root_chord_m - geometry.sweep_m * (geometry.root_chord_m + 2 * geometry.tip_chord_m) / (3 * (geometry.root_chord_m + geometry.tip_chord_m)) - (geometry.root_chord_m + geometry.tip_chord_m - geometry.root_chord_m * geometry.tip_chord_m / (geometry.root_chord_m + geometry.tip_chord_m)) / 6
                if not fin_set.station_m <= cp_station <= root_leading:
                    raise ValueError(f"All-Moving Fin '{fin_set.id}' CP hinge must intersect its root chord")
        return self


def fin_cp_station(fin_set: FinSet) -> float:
    geometry = fin_set.geometry
    root = geometry.root_chord_m
    tip = geometry.tip_chord_m
    return fin_set.station_m + root - geometry.sweep_m * (root + 2 * tip) / (3 * (root + tip)) - (root + tip - root * tip / (root + tip)) / 6

