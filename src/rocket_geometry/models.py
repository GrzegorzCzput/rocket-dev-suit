from rocket_model.models import *  # noqa: F403

import math
from typing import Literal

from pydantic import Field
from rocket_model.models import StrictModel


class AerodynamicEnvironment(StrictModel):
    air_density_kg_m3: float = Field(default=1.225, gt=0)
    wind_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    initial_position_world_m: tuple[float, float, float] = (0.0, 0.0, 1.0)
    initial_rpy_world_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    initial_linear_velocity_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    initial_angular_velocity_world_radps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    maximum_mach: float = Field(default=0.8, gt=0, le=1)
    speed_of_sound_mps: float = Field(default=340.0, gt=0)
    maximum_angle_of_attack_rad: float = Field(default=math.radians(10), gt=0, le=math.radians(30))


class SimulationDefinition(StrictModel):
    schema_version: Literal[1]
    simulation: AerodynamicEnvironment = Field(default_factory=AerodynamicEnvironment)
