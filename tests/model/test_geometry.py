import numpy as np

from rocket_model.geometry import build_geometry, rotate_about_axis
from rocket_model.models import RocketDefinition
from tests.model.test_models import valid_document


def test_geometry_contains_body_nose_and_four_fin_instances():
    definition = RocketDefinition.model_validate(valid_document())
    meshes = build_geometry(definition)
    names = [mesh.name for mesh in meshes]
    assert names[:2] == ["body", "nose"]
    assert sum(name.startswith("rear_fin_") for name in names) == 4


def test_nose_has_its_full_radius_at_the_body_and_a_point_at_positive_x_tip():
    definition = RocketDefinition.model_validate(valid_document())
    nose = next(mesh for mesh in build_geometry(definition) if mesh.name == "nose")
    body_length = definition.rocket.body.length_m
    body_radius = definition.rocket.body.diameter_m / 2
    at_body = nose.vertices[np.isclose(nose.vertices[:, 0], body_length)]
    at_tip = nose.vertices[np.isclose(nose.vertices[:, 0], body_length + definition.rocket.nose.length_m)]
    np.testing.assert_allclose(np.hypot(at_body[:, 1], at_body[:, 2]), body_radius)
    np.testing.assert_allclose(np.hypot(at_tip[:, 1], at_tip[:, 2]), 0.0)


def test_positive_all_moving_deflection_uses_right_hand_rule_about_outward_axis():
    point = np.array([1.0, 0.0, 0.0])
    axis_point = np.array([0.0, 0.0, 0.0])
    axis = np.array([0.0, 0.0, 1.0])
    rotated = rotate_about_axis(point, axis_point, axis, np.pi / 2)
    np.testing.assert_allclose(rotated, [0.0, 1.0, 0.0], atol=1e-12)


def test_control_surface_preview_changes_only_the_control_surface_mesh():
    definition = RocketDefinition.model_validate(valid_document())
    neutral = build_geometry(definition)
    preview = build_geometry(definition, {"rear_control_surface_0": 0.2})
    neutral_surface = next(mesh for mesh in neutral if mesh.name == "rear_control_surface_0")
    preview_surface = next(mesh for mesh in preview if mesh.name == "rear_control_surface_0")
    neutral_fin = next(mesh for mesh in neutral if mesh.name == "rear_fin_0")
    preview_fin = next(mesh for mesh in preview if mesh.name == "rear_fin_0")
    assert not np.allclose(neutral_surface.vertices, preview_surface.vertices)
    np.testing.assert_allclose(neutral_fin.vertices, preview_fin.vertices)
