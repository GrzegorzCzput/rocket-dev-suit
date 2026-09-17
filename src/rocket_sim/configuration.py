"""Load all validated inputs needed to construct a Flight Simulation."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator
import yaml

from rocket_model.models import Body, FinSet, Nose, RocketDefinition
from .motor import MotorCurve, MotorDefinition


class SimulationModel(BaseModel):
    model_config = ConfigDict(extra="forbid", validate_assignment=True, allow_inf_nan=False)


class DryMassProperties(SimulationModel):
    mass_kg: float = Field(gt=0)
    center_of_mass_station_m: float = Field(ge=0)
    inertia_xx_kg_m2: float = Field(gt=0)
    inertia_yy_kg_m2: float = Field(gt=0)
    inertia_zz_kg_m2: float = Field(gt=0)


class MotorInstallation(SimulationModel):
    data_file: str = Field(min_length=1)
    data_format: Literal["csv"] = "csv"
    nozzle_station_m: float = Field(ge=0)


class AerodynamicCalibration(SimulationModel):
    powered_drag_multiplier: float = Field(default=1.0, gt=0)
    coast_drag_multiplier: float = Field(default=1.0, gt=0)
    control_effectiveness_multiplier: float = Field(default=1.0, gt=0)


class ActuatorDynamics(SimulationModel):
    fin_set_id: str = Field(pattern=r"^[a-z][a-z0-9_\-]*$")
    maximum_rate_radps: float = Field(gt=0)


class FlightProperties(SimulationModel):
    motor: MotorInstallation
    aerodynamics: AerodynamicCalibration = Field(default_factory=AerodynamicCalibration)
    actuators: list[ActuatorDynamics] = Field(default_factory=list)


class FlightRocket(SimulationModel):
    id: str = Field(pattern=r"^[a-z][a-z0-9_\-]*$")
    name: str = Field(min_length=1)
    body: Body
    nose: Nose
    dry_mass_properties: DryMassProperties
    fin_sets: list[FinSet] = Field(default_factory=list)
    flight: FlightProperties

    @property
    def mass_properties(self) -> DryMassProperties:
        """Expose the dry properties to geometry-only calculations."""
        return self.dry_mass_properties

    @model_validator(mode="after")
    def validate_vehicle(self) -> "FlightRocket":
        RocketDefinition.model_validate(
            {
                "schema_version": 1,
                "rocket": {
                    "id": self.id,
                    "name": self.name,
                    "body": self.body.model_dump(mode="json"),
                    "nose": self.nose.model_dump(mode="json"),
                    "mass_properties": self.dry_mass_properties.model_dump(mode="json"),
                    "fin_sets": [fin_set.model_dump(mode="json") for fin_set in self.fin_sets],
                },
            }
        )
        length = self.body.length_m + self.nose.length_m
        if self.dry_mass_properties.center_of_mass_station_m > length:
            raise ValueError("dry Center of Mass Station must stay between the Aft Datum and nose tip")
        if self.flight.motor.nozzle_station_m > self.body.length_m:
            raise ValueError("motor nozzle must stay inside the Body Tube")
        identifiers = [fin_set.id for fin_set in self.fin_sets]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("Fin Set identifiers must be unique")
        movable = {fin_set.id for fin_set in self.fin_sets if fin_set.actuation.type != "fixed"}
        configured = [actuator.fin_set_id for actuator in self.flight.actuators]
        if len(configured) != len(set(configured)):
            raise ValueError("actuator fin_set_id values must be unique")
        if set(configured) != movable:
            raise ValueError("actuator dynamics must be supplied exactly once for every movable Fin Set")
        return self


class FlightRocketDefinition(SimulationModel):
    schema_version: Literal[2]
    rocket: FlightRocket


class Atmosphere(SimulationModel):
    model: Literal["isa1976"] = "isa1976"
    wind_world_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)


class LaunchRail(SimulationModel):
    position_world_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    length_m: float = Field(gt=0)
    inclination_rad: float = Field(default=math.pi / 2, ge=0, le=math.pi / 2)
    heading_rad: float = Field(default=0.0, ge=-math.pi, le=math.pi)


class IntegrationSettings(SimulationModel):
    time_step_s: float = Field(default=0.001, gt=0, le=0.02)
    controller_rate_hz: float = Field(default=100.0, gt=0)

    @model_validator(mode="after")
    def validate_sample_period(self) -> "IntegrationSettings":
        steps = 1.0 / (self.controller_rate_hz * self.time_step_s)
        if not math.isclose(steps, round(steps), rel_tol=0, abs_tol=1e-9):
            raise ValueError("controller period must be an integer number of physics steps")
        return self


class FinCommand(SimulationModel):
    time_s: float = Field(ge=0)
    fin_set_id: str = Field(pattern=r"^[a-z][a-z0-9_\-]*$")
    deflection_rad: float


class AerodynamicLimits(SimulationModel):
    maximum_mach: float = Field(default=0.7, gt=0, le=0.7)
    maximum_angle_of_attack_rad: float = Field(default=math.radians(10), gt=0, le=math.radians(10))
    diagnostic_continuation: bool = False


class Scenario(SimulationModel):
    atmosphere: Atmosphere = Field(default_factory=Atmosphere)
    launch_rail: LaunchRail
    integration: IntegrationSettings = Field(default_factory=IntegrationSettings)
    fin_commands: list[FinCommand] = Field(default_factory=list)
    aerodynamic_limits: AerodynamicLimits = Field(default_factory=AerodynamicLimits)
    stop_condition: Literal["apogee"] = "apogee"
    maximum_duration_s: float = Field(default=30.0, gt=0, le=300)
    random_seed: int = Field(default=1, ge=0)


class SimulationScenario(SimulationModel):
    schema_version: Literal[2]
    simulation: Scenario


class RunMotor(SimulationModel):
    definition: str = Field(min_length=1)
    nozzle_station_m: float = Field(default=0.0, ge=0)


class SimulationRunDefinition(SimulationModel):
    """Current composition format for one reproducible simulation run."""

    schema_version: Literal[1]
    rocket: str = Field(min_length=1)
    motor: RunMotor
    aerodynamics: AerodynamicCalibration = Field(default_factory=AerodynamicCalibration)
    actuators: list[ActuatorDynamics] = Field(default_factory=list)
    simulation: Scenario


@dataclass(frozen=True)
class SimulationInputs:
    rocket: FlightRocketDefinition
    scenario: SimulationScenario
    motor: MotorCurve
    source_paths: dict[str, Path]
    sha256: dict[str, str]


def _load_mapping(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return document


def _model_hash(model: BaseModel) -> str:
    normalized = json.dumps(model.model_dump(mode="json"), sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(normalized).hexdigest()


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_simulation_inputs(rocket_path: str | Path, scenario_path: str | Path) -> SimulationInputs:
    """Resolve and validate a complete, reproducible simulation input set."""
    rocket_source = Path(rocket_path).resolve()
    scenario_source = Path(scenario_path).resolve()
    rocket = FlightRocketDefinition.model_validate(_load_mapping(rocket_source))
    scenario = SimulationScenario.model_validate(_load_mapping(scenario_source))
    fin_sets = {fin_set.id: fin_set for fin_set in rocket.rocket.fin_sets}
    previous_time_by_set: dict[str, float] = {}
    for command in scenario.simulation.fin_commands:
        fin_set = fin_sets.get(command.fin_set_id)
        if fin_set is None or fin_set.actuation.type == "fixed":
            raise ValueError(f"fin command references unknown or fixed Fin Set: {command.fin_set_id}")
        previous = previous_time_by_set.get(command.fin_set_id, -1.0)
        if command.time_s <= previous:
            raise ValueError(f"fin commands for '{command.fin_set_id}' must increase strictly in time")
        if command.time_s > scenario.simulation.maximum_duration_s:
            raise ValueError(f"fin command for '{command.fin_set_id}' occurs after maximum_duration_s")
        limits = fin_set.actuation
        if not limits.minimum_deflection_rad <= command.deflection_rad <= limits.maximum_deflection_rad:
            raise ValueError(f"fin command for '{command.fin_set_id}' exceeds its deflection limits")
        previous_time_by_set[command.fin_set_id] = command.time_s
    motor_source = (rocket_source.parent / rocket.rocket.flight.motor.data_file).resolve()
    if not motor_source.is_file():
        raise FileNotFoundError(f"motor data file does not exist: {motor_source}")
    motor = MotorCurve.from_csv(motor_source)
    return SimulationInputs(
        rocket=rocket,
        scenario=scenario,
        motor=motor,
        source_paths={"rocket": rocket_source, "scenario": scenario_source, "motor": motor_source},
        sha256={"rocket": _model_hash(rocket), "scenario": _model_hash(scenario), "motor": _file_hash(motor_source)},
    )


def load_run(path: str | Path) -> SimulationInputs:
    """Load a current Simulation Run Definition and all referenced inputs."""
    run_source = Path(path).resolve()
    run = SimulationRunDefinition.model_validate(_load_mapping(run_source))
    rocket_source = (run_source.parent / run.rocket).resolve()
    motor_source = (run_source.parent / run.motor.definition).resolve()
    rocket_definition = RocketDefinition.model_validate(_load_mapping(rocket_source))
    rocket = rocket_definition.rocket
    if run.motor.nozzle_station_m > rocket.body.length_m:
        raise ValueError("motor nozzle must stay inside the Body Tube")
    movable = {fin_set.id for fin_set in rocket.fin_sets if fin_set.actuation.type != "fixed"}
    configured = [actuator.fin_set_id for actuator in run.actuators]
    if len(configured) != len(set(configured)) or set(configured) != movable:
        raise ValueError("actuator dynamics must be supplied exactly once for every movable Fin Set")
    flight_rocket = FlightRocketDefinition.model_validate(
        {
            "schema_version": 2,
            "rocket": {
                "id": rocket.id,
                "name": rocket.name,
                "body": rocket.body.model_dump(mode="json"),
                "nose": rocket.nose.model_dump(mode="json"),
                "dry_mass_properties": rocket.mass_properties.model_dump(mode="json"),
                "fin_sets": [fin_set.model_dump(mode="json") for fin_set in rocket.fin_sets],
                "flight": {
                    "motor": {"data_file": str(motor_source), "data_format": "csv", "nozzle_station_m": run.motor.nozzle_station_m},
                    "aerodynamics": run.aerodynamics.model_dump(mode="json"),
                    "actuators": [actuator.model_dump(mode="json") for actuator in run.actuators],
                },
            },
        }
    )
    scenario = SimulationScenario(schema_version=2, simulation=run.simulation)
    motor = MotorCurve.from_definition(motor_source)
    motor_definition = MotorDefinition.model_validate(_load_mapping(motor_source))
    thrust_source = (motor_source.parent / motor_definition.thrust_curve_file).resolve()
    return SimulationInputs(
        rocket=flight_rocket,
        scenario=scenario,
        motor=motor,
        source_paths={"run": run_source, "rocket": rocket_source, "motor": motor_source, "thrust_curve": thrust_source},
        sha256={"run": _model_hash(run), "rocket": _model_hash(rocket_definition), "motor": _file_hash(motor_source), "thrust_curve": _file_hash(thrust_source)},
    )
