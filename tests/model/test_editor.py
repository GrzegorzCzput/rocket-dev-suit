from rocket_model.editor import constrain_draft_center_of_mass
from rocket_model.models import RocketDefinition


def test_editor_constrains_center_of_mass_when_rocket_is_shortened():
    draft = {
        "schema_version": 1,
        "rocket": {
            "id": "shortened",
            "name": "Shortened rocket",
            "body": {"length_m": 0.4, "diameter_m": 0.05},
            "nose": {"type": "half_ellipsoid", "length_m": 0.1},
            "mass_properties": {
                "mass_kg": 1.0,
                "center_of_mass_station_m": 0.65,
                "inertia_xx_kg_m2": 0.01,
                "inertia_yy_kg_m2": 0.1,
                "inertia_zz_kg_m2": 0.1,
            },
            "fin_sets": [],
        },
    }

    assert constrain_draft_center_of_mass(draft) is True
    assert draft["rocket"]["mass_properties"]["center_of_mass_station_m"] == 0.5
    RocketDefinition.model_validate(draft)
