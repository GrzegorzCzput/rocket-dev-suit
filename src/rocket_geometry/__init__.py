"""Deprecated compatibility package; use :mod:`rocket_model` and :mod:`rocket_sim`."""

import warnings

warnings.warn(
    "rocket_geometry is deprecated; import rocket_model or rocket_sim instead",
    DeprecationWarning,
    stacklevel=2,
)

from rocket_model import BarrowmanResult, RocketDefinition, calculate_barrowman

__all__ = ["BarrowmanResult", "RocketDefinition", "calculate_barrowman"]
