"""Validated interpolation of reusable motor performance samples."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
from pydantic import BaseModel, ConfigDict, Field
import yaml


_COLUMNS = (
    "time_s",
    "thrust_n",
    "propellant_mass_kg",
    "propellant_cg_station_m",
    "inertia_xx_kg_m2",
    "inertia_yy_kg_m2",
    "inertia_zz_kg_m2",
)


@dataclass(frozen=True)
class MotorSample:
    thrust_n: float
    propellant_mass_kg: float
    propellant_cg_station_m: float
    inertia_kg_m2: tuple[float, float, float]


@dataclass(frozen=True)
class MotorCurve:
    """Immutable motor history with clamped linear interpolation."""

    time_s: np.ndarray
    thrust_n: np.ndarray
    propellant_mass_kg: np.ndarray
    propellant_cg_station_m: np.ndarray
    inertia_xx_kg_m2: np.ndarray
    inertia_yy_kg_m2: np.ndarray
    inertia_zz_kg_m2: np.ndarray

    @classmethod
    def from_csv(cls, path: str | Path) -> "MotorCurve":
        source = Path(path)
        with source.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != list(_COLUMNS):
                raise ValueError(f"motor CSV columns must be exactly: {', '.join(_COLUMNS)}")
            rows = list(reader)
        if len(rows) < 2:
            raise ValueError("motor CSV must contain at least two samples")
        try:
            columns = {name: np.asarray([float(row[name]) for row in rows], dtype=float) for name in _COLUMNS}
        except (TypeError, ValueError) as error:
            raise ValueError("motor CSV values must be finite numbers") from error
        if not all(np.all(np.isfinite(values)) for values in columns.values()):
            raise ValueError("motor CSV values must be finite numbers")
        time = columns["time_s"]
        if time[0] != 0 or np.any(np.diff(time) <= 0):
            raise ValueError("motor time_s must start at zero and increase strictly")
        for name in ("thrust_n", "propellant_mass_kg", "propellant_cg_station_m", "inertia_xx_kg_m2", "inertia_yy_kg_m2", "inertia_zz_kg_m2"):
            if np.any(columns[name] < 0):
                raise ValueError(f"motor {name} values must be non-negative")
        if np.any(np.diff(columns["propellant_mass_kg"]) > 1e-12):
            raise ValueError("motor propellant_mass_kg must not increase")
        return cls(**columns)

    @classmethod
    def from_definition(cls, path: str | Path) -> "MotorCurve":
        """Load a motor YAML and derive mass history from its RASP thrust curve."""
        source = Path(path).resolve()
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
        definition = MotorDefinition.model_validate(document)
        thrust_path = (source.parent / definition.thrust_curve_file).resolve()
        samples: list[tuple[float, float]] = []
        for raw_line in thrust_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith(";"):
                continue
            fields = line.split()
            if len(fields) == 2:
                try:
                    samples.append((float(fields[0]), float(fields[1])))
                except ValueError:
                    continue
        if len(samples) < 2:
            raise ValueError(f"RASP thrust curve must contain at least two samples: {thrust_path}")
        if samples[0][0] > 0:
            samples.insert(0, (0.0, 0.0))
        time = np.asarray([sample[0] for sample in samples], dtype=float)
        thrust = np.asarray([sample[1] for sample in samples], dtype=float)
        if time[0] != 0 or np.any(np.diff(time) <= 0) or np.any(thrust < 0):
            raise ValueError("RASP samples must start at or after zero, increase in time, and have non-negative thrust")
        segment_impulse = 0.5 * (thrust[:-1] + thrust[1:]) * np.diff(time)
        cumulative = np.r_[0.0, np.cumsum(segment_impulse)]
        total_impulse = float(cumulative[-1])
        if total_impulse <= 0:
            raise ValueError("motor thrust curve must have positive total impulse")
        propellant_mass = definition.propellant_mass_kg * np.maximum(0.0, 1.0 - cumulative / total_impulse)
        radius = definition.propellant_radius_m
        length = definition.propellant_length_m
        inertia_xx = 0.5 * propellant_mass * radius**2
        inertia_transverse = propellant_mass * (3 * radius**2 + length**2) / 12
        return cls(
            time_s=time,
            thrust_n=thrust,
            propellant_mass_kg=propellant_mass,
            propellant_cg_station_m=np.full_like(time, definition.propellant_cg_station_m),
            inertia_xx_kg_m2=inertia_xx,
            inertia_yy_kg_m2=inertia_transverse,
            inertia_zz_kg_m2=inertia_transverse.copy(),
        )

    @property
    def burnout_time_s(self) -> float:
        positive = np.flatnonzero(self.thrust_n > 0)
        if not len(positive):
            return 0.0
        last_positive = int(positive[-1])
        burnout_index = min(last_positive + 1, len(self.time_s) - 1)
        return float(self.time_s[burnout_index])

    def sample(self, time_s: float) -> MotorSample:
        if not math.isfinite(time_s) or time_s < 0:
            raise ValueError("motor sample time must be finite and non-negative")

        def interpolate(values: np.ndarray, *, after: float | None = None) -> float:
            right = float(values[-1]) if after is None else after
            return float(np.interp(time_s, self.time_s, values, left=float(values[0]), right=right))

        return MotorSample(
            thrust_n=interpolate(self.thrust_n, after=0.0),
            propellant_mass_kg=interpolate(self.propellant_mass_kg),
            propellant_cg_station_m=interpolate(self.propellant_cg_station_m),
            inertia_kg_m2=(
                interpolate(self.inertia_xx_kg_m2),
                interpolate(self.inertia_yy_kg_m2),
                interpolate(self.inertia_zz_kg_m2),
            ),
        )


class MotorDefinition(BaseModel):
    """Metadata and mass assumptions paired with a manufacturer thrust curve."""

    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    schema_version: int = Field(default=1, ge=1, le=1)
    id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    manufacturer: str = Field(min_length=1)
    thrust_curve_file: str = Field(min_length=1)
    loaded_mass_kg: float = Field(gt=0)
    propellant_mass_kg: float = Field(gt=0)
    propellant_cg_station_m: float = Field(ge=0)
    propellant_radius_m: float = Field(gt=0)
    propellant_length_m: float = Field(gt=0)
    source_url: str | None = None
