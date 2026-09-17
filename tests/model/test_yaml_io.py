import math
from pathlib import Path

from rocket_model.yaml_io import dump_definition, load_definition
from tests.model.test_models import valid_document


def test_yaml_round_trip_preserves_rocket_definition(tmp_path):
    source = tmp_path / "rocket.yaml"
    source.write_text(dump_definition(valid_document()))
    loaded = load_definition(source)
    assert loaded.model_dump() == load_definition(source).model_dump()


def test_default_rocket_uses_a_45_degree_fin_azimuth_in_radians():
    source = Path(__file__).parents[2] / "examples" / "models" / "control_surface" / "rocket.yaml"
    definition = load_definition(source)

    assert definition.rocket.fin_sets[0].angular_offset_rad == math.pi / 4
