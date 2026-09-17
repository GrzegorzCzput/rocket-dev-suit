import pytest

from rocket_model.barrowman import calculate_barrowman, fin_cp_span_m, static_margin_calibers
from rocket_model.models import RocketDefinition
from tests.model.test_models import valid_document


def test_half_ellipsoid_cp_is_two_thirds_of_nose_length_from_body_base():
    definition = RocketDefinition.model_validate(valid_document())
    result = calculate_barrowman(definition)
    assert result.nose.cp_station_m == pytest.approx(1.2 + 2 * 0.3 / 3)
    assert result.nose.normal_force_slope_per_rad == pytest.approx(2.0)


def test_total_cp_is_weighted_from_nose_and_fin_set_contributions():
    definition = RocketDefinition.model_validate(valid_document())
    result = calculate_barrowman(definition)
    expected = (
        result.nose.normal_force_slope_per_rad * result.nose.cp_station_m
        + result.fin_sets[0].normal_force_slope_per_rad * result.fin_sets[0].cp_station_m
    ) / (result.nose.normal_force_slope_per_rad + result.fin_sets[0].normal_force_slope_per_rad)
    assert result.rocket_cp_station_m == pytest.approx(expected)
    assert result.body_tube_cp_station_m is None


def test_fin_component_marker_uses_mean_aerodynamic_chord_span():
    definition = RocketDefinition.model_validate(valid_document())
    value = fin_cp_span_m(definition.rocket.fin_sets[0])
    assert value == pytest.approx(0.12 * (0.22 + 2 * 0.10) / (3 * (0.22 + 0.10)))


def test_static_margin_is_center_of_mass_aft_distance_from_cp_in_body_diameters():
    definition = RocketDefinition.model_validate(valid_document())
    result = calculate_barrowman(definition)
    expected = (0.65 - result.rocket_cp_station_m) / 0.1
    assert static_margin_calibers(definition, result) == pytest.approx(expected)
