from rocket_model.yaml_io import *  # noqa: F403

from pathlib import Path
from typing import Any

import yaml

from .models import SimulationDefinition


def load_simulation(path: str | Path) -> SimulationDefinition:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("simulation YAML must contain a mapping")
    return SimulationDefinition.model_validate(document)


def dump_simulation(document: SimulationDefinition | dict[str, Any]) -> str:
    simulation = document if isinstance(document, SimulationDefinition) else SimulationDefinition.model_validate(document)
    return yaml.safe_dump(simulation.model_dump(mode="json"), sort_keys=False, allow_unicode=False)
