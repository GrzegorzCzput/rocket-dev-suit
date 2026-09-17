"""Generate triangle meshes for reusable rocket definitions."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .barrowman import calculate_barrowman
from .models import FinSet, RocketDefinition, fin_cp_station


@dataclass(frozen=True)
class Mesh:
    name: str
    vertices: np.ndarray
    faces: np.ndarray
    color: str = "#7f8c8d"


def rotate_about_axis(point: np.ndarray, axis_point: np.ndarray, axis: np.ndarray, angle_rad: float) -> np.ndarray:
    unit_axis = axis / np.linalg.norm(axis)
    relative = point - axis_point
    return axis_point + relative * math.cos(angle_rad) + np.cross(unit_axis, relative) * math.sin(angle_rad) + unit_axis * np.dot(unit_axis, relative) * (1 - math.cos(angle_rad))


def _cylinder(length: float, radius: float, segments: int = 48) -> tuple[np.ndarray, np.ndarray]:
    vertices = []
    for x in (0.0, length):
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            vertices.append((x, -radius * math.sin(angle), radius * math.cos(angle)))
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        faces.append((i, j, segments + j, segments + i))
    return np.asarray(vertices, dtype=float), np.asarray(faces, dtype=int)


def _nose(body_length: float, length: float, radius: float, rings: int = 20, segments: int = 48) -> tuple[np.ndarray, np.ndarray]:
    vertices = []
    for ring in range(rings + 1):
        fraction = ring / rings
        x = body_length + fraction * length
        radial = radius * math.sqrt(max(0.0, 1 - fraction * fraction))
        for i in range(segments):
            angle = 2 * math.pi * i / segments
            vertices.append((x, -radial * math.sin(angle), radial * math.cos(angle)))
    faces = []
    for ring in range(rings):
        start = ring * segments
        next_start = (ring + 1) * segments
        for i in range(segments):
            j = (i + 1) % segments
            faces.append((start + i, start + j, next_start + j, next_start + i))
    return np.asarray(vertices, dtype=float), np.asarray(faces, dtype=int)


def _radial_basis(azimuth: float) -> tuple[np.ndarray, np.ndarray]:
    radial = np.array([0.0, -math.sin(azimuth), math.cos(azimuth)])
    tangent = np.array([0.0, -math.cos(azimuth), -math.sin(azimuth)])
    return radial, tangent


def _fin_polygon(fin_set: FinSet) -> list[tuple[float, float]]:
    geometry = fin_set.geometry
    root_leading = fin_set.station_m + geometry.root_chord_m
    tip_leading = root_leading - geometry.sweep_m
    return [(fin_set.station_m, 0.0), (root_leading, 0.0), (tip_leading, geometry.span_m), (tip_leading - geometry.tip_chord_m, geometry.span_m)]


def _extruded_fin(fin_set: FinSet, body_radius: float, azimuth: float, angle_rad: float, name: str, color: str, polygon: list[tuple[float, float]] | None = None, rotation_axis_point: np.ndarray | None = None, rotation_axis: np.ndarray | None = None) -> Mesh:
    radius = body_radius
    radial, tangent = _radial_basis(azimuth)
    thickness = fin_set.geometry.thickness_m
    polygon = polygon or _fin_polygon(fin_set)
    points = []
    for x, span in polygon:
        center = np.array([x, 0.0, 0.0]) + radial * (radius + span)
        points.extend((center - tangent * thickness / 2, center + tangent * thickness / 2))
    vertices = np.asarray(points)
    if angle_rad:
        axis_point = rotation_axis_point if rotation_axis_point is not None else np.array([fin_set_cp_x(fin_set), 0.0, 0.0]) + radial * radius
        axis = rotation_axis if rotation_axis is not None else radial
        vertices = np.asarray([rotate_about_axis(point, axis_point, axis, angle_rad) for point in vertices])
    faces = []
    count = len(polygon)
    faces.append(tuple(2 * i for i in range(count)))
    faces.append(tuple(2 * i + 1 for i in reversed(range(count))))
    for i in range(count):
        j = (i + 1) % count
        faces.append((2 * i, 2 * j, 2 * j + 1, 2 * i + 1))
    return Mesh(name, vertices, np.asarray(faces, dtype=int), color)


def fin_set_cp_x(fin_set: FinSet) -> float:
    return fin_cp_station(fin_set)


def _control_surface_polygon(fin_set: FinSet) -> list[tuple[float, float]]:
    geometry = fin_set.geometry
    actuation = fin_set.actuation
    assert actuation.span_start_fraction is not None and actuation.hinge_fraction is not None
    start = actuation.span_start_fraction
    def edge(fraction: float) -> tuple[float, float, float]:
        leading = fin_set.station_m + geometry.root_chord_m - geometry.sweep_m * fraction
        chord = geometry.root_chord_m + (geometry.tip_chord_m - geometry.root_chord_m) * fraction
        return leading, leading - chord, geometry.span_m * fraction
    leading_start, trailing_start, span_start = edge(start)
    leading_tip, trailing_tip, span_tip = edge(1.0)
    return [(leading_start - (leading_start - trailing_start) * actuation.hinge_fraction, span_start), (trailing_start, span_start), (trailing_tip, span_tip), (leading_tip - (leading_tip - trailing_tip) * actuation.hinge_fraction, span_tip)]


def build_geometry(definition: RocketDefinition, preview_pose: dict[str, float] | None = None) -> list[Mesh]:
    preview_pose = preview_pose or {}
    rocket = definition.rocket
    body_vertices, body_faces = _cylinder(rocket.body.length_m, rocket.body.diameter_m / 2)
    nose_vertices, nose_faces = _nose(rocket.body.length_m, rocket.nose.length_m, rocket.body.diameter_m / 2)
    meshes = [Mesh("body", body_vertices, body_faces, "#7f8c8d"), Mesh("nose", nose_vertices, nose_faces, "#95a5a6")]
    cp = calculate_barrowman(definition)
    cp_by_id = {item.name: item.cp_station_m for item in cp.fin_sets}
    for fin_set in rocket.fin_sets:
        body_radius = rocket.body.diameter_m / 2
        if fin_set.actuation.type == "fixed":
            color = "#377eb8"
        elif fin_set.actuation.type == "all_moving":
            color = "#f28e2b"
        else:
            color = "#377eb8"
        for index in range(fin_set.count):
            azimuth = fin_set.angular_offset_rad + 2 * math.pi * index / fin_set.count
            value = preview_pose.get(f"{fin_set.id}_fin_{index}", 0.0)
            meshes.append(_extruded_fin(fin_set, body_radius, azimuth, value, f"{fin_set.id}_fin_{index}", color))
            if fin_set.actuation.type == "control_surface":
                surface_angle = preview_pose.get(f"{fin_set.id}_control_surface_{index}", 0.0)
                surface_polygon = _control_surface_polygon(fin_set)
                radial, _ = _radial_basis(azimuth)
                start_x, start_span = surface_polygon[0]
                tip_x, tip_span = surface_polygon[3]
                hinge_start = np.array([start_x, 0.0, 0.0]) + radial * (body_radius + start_span)
                hinge_tip = np.array([tip_x, 0.0, 0.0]) + radial * (body_radius + tip_span)
                surface = _extruded_fin(fin_set, body_radius, azimuth, surface_angle, f"{fin_set.id}_control_surface_{index}", "#e15759", surface_polygon, hinge_start, hinge_tip - hinge_start)
                meshes.append(surface)
    return meshes
