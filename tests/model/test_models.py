import math

import pytest
from pydantic import ValidationError

from rocket_model.models import RocketDefinition


def valid_document():
    return {
        "schema_version": 1,
        "rocket": {
            "id": "demo",
            "name": "Demo Rocket",
            "body": {"length_m": 1.2, "diameter_m": 0.1},
            "nose": {"type": "half_ellipsoid", "length_m": 0.3},
            "mass_properties": {"mass_kg": 2.5, "center_of_mass_station_m": 0.65, "inertia_xx_kg_m2": 0.012, "inertia_yy_kg_m2": 0.31, "inertia_zz_kg_m2": 0.31},
            "fin_sets": [
                {
                    "id": "rear",
                    "station_m": 0.0,
                    "count": 4,
                    "angular_offset_rad": 0.0,
                    "geometry": {
                        "root_chord_m": 0.22,
                        "tip_chord_m": 0.10,
                        "span_m": 0.12,
                        "sweep_m": 0.07,
                        "thickness_m": 0.004,
                    },
                    "actuation": {"type": "control_surface", "span_start_fraction": 0.2, "hinge_fraction": 0.7, "minimum_deflection_rad": -0.35, "maximum_deflection_rad": 0.35},
                }
            ],
        },
    }


def test_valid_definition_parses_with_explicit_si_fields():
    definition = RocketDefinition.model_validate(valid_document())
    assert definition.rocket.body.length_m == 1.2
    assert definition.rocket.fin_sets[0].actuation.type == "control_surface"


def test_unknown_yaml_fields_are_rejected():
    document = valid_document()
    document["rocket"]["body"]["length"] = 1.2
    with pytest.raises(ValidationError):
        RocketDefinition.model_validate(document)


def test_fin_vertices_must_stay_between_aft_datum_and_body_end():
    document = valid_document()
    document["rocket"]["fin_sets"][0]["geometry"]["sweep_m"] = 0.4
    with pytest.raises(ValidationError, match="body envelope"):
        RocketDefinition.model_validate(document)


def test_all_moving_limits_contain_zero_and_control_surface_fractions_are_valid():
    document = valid_document()
    actuation = document["rocket"]["fin_sets"][0]["actuation"]
    actuation.update({"type": "all_moving", "minimum_deflection_rad": 0.1})
    with pytest.raises(ValidationError, match="contain zero"):
        RocketDefinition.model_validate(document)

    actuation = document["rocket"]["fin_sets"][0]["actuation"]
    actuation.update({"type": "control_surface", "span_start_fraction": 1.0, "minimum_deflection_rad": -0.2, "maximum_deflection_rad": 0.2})
    with pytest.raises(ValidationError, match="span_start_fraction"):
        RocketDefinition.model_validate(document)


def test_angles_are_normalized_only_by_the_user_not_the_model():
    document = valid_document()
    document["rocket"]["fin_sets"][0]["angular_offset_rad"] = 2 * math.pi
    with pytest.raises(ValidationError, match="angular_offset_rad"):
        RocketDefinition.model_validate(document)


def test_mass_properties_are_persisted_in_si_units_and_center_of_mass_stays_on_rocket():
    document = valid_document()
    definition = RocketDefinition.model_validate(document)
    assert definition.rocket.mass_properties.mass_kg == 2.5
    document["rocket"]["mass_properties"]["center_of_mass_station_m"] = 1.51
    with pytest.raises(ValidationError, match="Center of Mass Station"):
        RocketDefinition.model_validate(document)
