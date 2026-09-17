"""YAML serialization for simulation-independent rocket definitions."""

from pathlib import Path
from typing import Any

import yaml

from .models import RocketDefinition


def load_definition(path: str | Path) -> RocketDefinition:
    with Path(path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("rocket YAML must contain a mapping")
    return RocketDefinition.model_validate(document)


def dump_definition(document: RocketDefinition | dict[str, Any]) -> str:
    definition = document if isinstance(document, RocketDefinition) else RocketDefinition.model_validate(document)
    return yaml.safe_dump(definition.model_dump(mode="json", exclude_none=True), sort_keys=False, allow_unicode=False)


def save_definition(document: RocketDefinition, path: str | Path) -> None:
    Path(path).write_text(dump_definition(document), encoding="utf-8")
