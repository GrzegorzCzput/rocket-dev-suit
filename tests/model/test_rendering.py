from rocket_model.models import RocketDefinition
from rocket_model.rendering import plotly_figure
from tests.model.test_models import valid_document


def test_plotly_figure_has_geometry_and_cp_traces():
    definition = RocketDefinition.model_validate(valid_document())
    figure = plotly_figure(definition)
    assert len(figure.data) >= 8
    rocket_cp = next(trace for trace in figure.data if trace.name == "rocket CP")
    component_cp = next(trace for trace in figure.data if trace.name == "component CP")
    assert rocket_cp.marker.size >= 14
    assert rocket_cp.mode == "markers"
    assert list(rocket_cp.y) == [0.0]
    assert list(rocket_cp.z) == [0.0]
    assert component_cp.marker.size >= 10
    assert figure.layout.scene.camera.eye.y == 2.5
    assert figure.layout.scene.camera.up.z == 1


def test_plotly_figure_marks_center_of_gravity_at_the_configured_center_of_mass_station():
    definition = RocketDefinition.model_validate(valid_document())
    figure = plotly_figure(definition)
    cg = next(trace for trace in figure.data if trace.name == "CG / center of mass")
    assert list(cg.x) == [0.65]
    assert list(cg.y) == [0.0]
    assert list(cg.z) == [0.0]
    assert cg.mode == "markers"
