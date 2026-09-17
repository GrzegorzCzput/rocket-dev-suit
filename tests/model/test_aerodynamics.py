import math

import pytest

from rocket_model.aerodynamics import component_areas, drag_force_n, low_angle_forces, rocket_reference_area_m2
from rocket_model.models import RocketDefinition
from tests.model.test_models import valid_document


def test_derived_component_areas_and_drag_use_their_documented_reference_areas():
    document = valid_document()
    document["rocket"]["body"]["aerodynamics"] = {"skin_friction_coefficient": 0.01, "pressure_drag_coefficient": 0.2}
    definition = RocketDefinition.model_validate(document)

    areas = {item.name: item for item in component_areas(definition)}

    assert areas["body_tube"].wetted_area_m2 == pytest.approx(math.pi * 0.1 * 1.2)
    assert areas["body_tube"].reference_area_m2 == pytest.approx(rocket_reference_area_m2(definition))
    assert areas["rear"].reference_area_m2 == pytest.approx(4 * (0.22 + 0.10) * 0.12 / 2)
    assert drag_force_n(100, 0.01, areas["body_tube"].wetted_area_m2, 0.2, areas["body_tube"].reference_area_m2) == pytest.approx(areas["body_tube"].wetted_area_m2 + 20 * areas["body_tube"].reference_area_m2)


def test_low_angle_forces_have_zero_normal_force_at_zero_aoa_and_reverse_with_lateral_velocity():
    document = valid_document()
    document["rocket"]["nose"]["aerodynamics"] = {"pressure_drag_coefficient": 0.1}
    definition = RocketDefinition.model_validate(document)

    axial = {force.name: force for force in low_angle_forces(definition, (50, 0, 0), 1.225)}
    upward = {force.name: force for force in low_angle_forces(definition, (50, 0, 5), 1.225)}

    assert axial["nose"].drag_n > 0
    assert axial["nose"].normal_force_body_n == (0.0, 0.0, 0.0)
    assert upward["nose"].normal_force_body_n[2] < 0


def test_low_angle_normal_force_is_omitted_outside_the_model_angle_range():
    """A vertical drop is roughly 90 degrees AoA for a horizontal rocket."""
    definition = RocketDefinition.model_validate(valid_document())

    force = {item.name: item for item in low_angle_forces(definition, (0, 0, 50), 1.225)}["nose"]
    normal_magnitude = abs(force.normal_force_body_n[2])
    assert normal_magnitude == 0.0
