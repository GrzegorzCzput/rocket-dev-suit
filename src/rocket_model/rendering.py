"""Render rocket definitions and aerodynamic markers."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile

import numpy as np
import plotly.graph_objects as go

from .barrowman import calculate_barrowman, fin_cp_span_m
from .geometry import Mesh, build_geometry
from .models import RocketDefinition


def plotly_figure(definition: RocketDefinition, preview_pose: dict[str, float] | None = None) -> go.Figure:
    figure = go.Figure()
    for mesh in build_geometry(definition, preview_pose):
        triangles = np.asarray([(face[0], face[i], face[i + 1]) for face in mesh.faces for i in range(1, len(face) - 1)], dtype=int)
        figure.add_trace(go.Mesh3d(x=mesh.vertices[:, 0], y=mesh.vertices[:, 1], z=mesh.vertices[:, 2], i=triangles[:, 0], j=triangles[:, 1], k=triangles[:, 2], color=mesh.color, opacity=0.9, name=mesh.name, hovertext=mesh.name, hoverinfo="text"))
    result = calculate_barrowman(definition)
    marker_x = []
    marker_y = []
    marker_z = []
    marker_names = []
    for fin_set, item in zip(definition.rocket.fin_sets, result.fin_sets):
        span = fin_cp_span_m(fin_set) + definition.rocket.body.diameter_m / 2
        for index in range(fin_set.count):
            azimuth = fin_set.angular_offset_rad + 2 * np.pi * index / fin_set.count
            marker_x.append(item.cp_station_m)
            marker_y.append(-span * np.sin(azimuth))
            marker_z.append(span * np.cos(azimuth))
            marker_names.append(f"{item.name} fin {index} CP")
    figure.add_trace(go.Scatter3d(x=marker_x, y=marker_y, z=marker_z, mode="markers", marker={"size": 10, "color": "#161616", "line": {"color": "#ffffff", "width": 2}}, text=marker_names, name="component CP", hoverinfo="text"))
    figure.add_trace(go.Scatter3d(x=[result.nose.cp_station_m], y=[0.0], z=[0.0], mode="markers", marker={"size": 13, "color": "#161616", "line": {"color": "#ffffff", "width": 2}}, text=["Nose CP"], name="nose CP", hoverinfo="text"))
    if result.rocket_cp_station_m is not None:
        figure.add_trace(go.Scatter3d(x=[result.rocket_cp_station_m], y=[0.0], z=[0.0], mode="markers", marker={"size": 16, "symbol": "diamond", "color": "#cc00cc", "line": {"color": "#ffffff", "width": 2}}, text=["Rocket CP"], name="rocket CP", hoverinfo="text"))
    cg_station = definition.rocket.mass_properties.center_of_mass_station_m
    figure.add_trace(go.Scatter3d(x=[cg_station], y=[0.0], z=[0.0], mode="markers", marker={"size": 15, "symbol": "cross", "color": "#00a651", "line": {"color": "#ffffff", "width": 2}}, text=["CG / center of mass"], name="CG / center of mass", hoverinfo="text"))
    figure.update_layout(scene={"xaxis_title": "+x forward (m)", "yaxis_title": "+y left (m)", "zaxis_title": "+z up (m)", "aspectmode": "data", "camera": {"eye": {"x": 0, "y": 2.5, "z": 0}, "up": {"x": 0, "y": 0, "z": 1}}}, legend={"orientation": "h"}, margin={"l": 0, "r": 0, "t": 30, "b": 0}, title="Rocket geometry")
    return figure


def _set_equal_3d_axes(axis):
    axis.set_box_aspect((1, 1, 1))


def render_png(definition: RocketDefinition, output: str | Path, preview_pose: dict[str, float] | None = None) -> None:
    mpl_config = Path(tempfile.gettempdir()) / "rocket_geometry_mplconfig"
    mpl_config.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(mpl_config))
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    figure = plt.figure(figsize=(16, 10), dpi=150)
    axes = [figure.add_subplot(2, 2, index, projection="3d") for index in range(1, 5)]
    meshes = build_geometry(definition, preview_pose)
    result = calculate_barrowman(definition)
    views = [(25, -55, "Isometric"), (0, 90, "Side"), (90, -90, "Top"), (0, 0, "Rear")]
    for axis, (elevation, azimuth, title) in zip(axes, views):
        for mesh in meshes:
            polygons = [mesh.vertices[list(face)] for face in mesh.faces]
            axis.add_collection3d(Poly3DCollection(polygons, facecolors=mesh.color, edgecolors="#303030", linewidths=0.2, alpha=0.9))
        axis.scatter([result.nose.cp_station_m], [0.0], [0.0], color="#161616", edgecolors="#ffffff", linewidths=1.2, s=75)
        for fin_set, item in zip(definition.rocket.fin_sets, result.fin_sets):
            span = fin_cp_span_m(fin_set) + definition.rocket.body.diameter_m / 2
            for index in range(fin_set.count):
                angle = fin_set.angular_offset_rad + 2 * np.pi * index / fin_set.count
                axis.scatter([item.cp_station_m], [-span * np.sin(angle)], [span * np.cos(angle)], color="#161616", edgecolors="#ffffff", linewidths=1.0, s=38)
        if result.rocket_cp_station_m is not None:
            axis.scatter([result.rocket_cp_station_m], [0.0], [0.0], color="#cc00cc", edgecolors="#ffffff", linewidths=1.2, marker="D", s=100)
        cg_station = definition.rocket.mass_properties.center_of_mass_station_m
        axis.scatter([cg_station], [0.0], [0.0], color="#00a651", marker="x", s=100, linewidths=2.5)
        axis.set_title(title)
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.set_zlabel("z (m)")
        axis.view_init(elev=elevation, azim=azimuth)
        axis.autoscale_view()
        _set_equal_3d_axes(axis)
    figure.suptitle(f"{definition.rocket.name} — neutral Barrowman CP: {result.rocket_cp_station_m:.4f} m" if result.rocket_cp_station_m is not None else definition.rocket.name)
    figure.tight_layout()
    figure.savefig(output, facecolor="white")
    plt.close(figure)
