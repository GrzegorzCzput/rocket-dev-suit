from __future__ import annotations

import copy
import math
from pathlib import Path

import streamlit as st

from rocket_model.models import RocketDefinition
from rocket_model.rendering import plotly_figure
from rocket_model.yaml_io import dump_definition, load_definition


def _example_path() -> Path:
    return Path(__file__).parents[2] / "examples" / "models" / "control_surface" / "rocket.yaml"


def load_definition_from_bytes(data: bytes) -> RocketDefinition:
    import yaml

    return RocketDefinition.model_validate(yaml.safe_load(data))


def constrain_draft_center_of_mass(draft: dict) -> bool:
    """Keep a transient editor draft's CG on the rocket after a resize.

    Persisted rocket definitions remain strictly validated by ``Rocket``.  This
    helper only makes dimension edits usable when their previous CG station is
    beyond the newly moved nose tip.
    """
    rocket = draft["rocket"]
    maximum_station_m = rocket["body"]["length_m"] + rocket["nose"]["length_m"]
    mass = rocket["mass_properties"]
    if mass["center_of_mass_station_m"] > maximum_station_m:
        mass["center_of_mass_station_m"] = maximum_station_m
        return True
    return False


def _new_fin_set(index: int) -> dict:
    return {
        "id": f"fin_set_{index}", "station_m": 0.0, "count": 4, "angular_offset_rad": 0.0,
        "geometry": {"root_chord_m": 0.18, "tip_chord_m": 0.08, "span_m": 0.1, "sweep_m": 0.05, "thickness_m": 0.003},
        "actuation": {"type": "fixed"},
        "aerodynamics": {"skin_friction_coefficient": 0.0, "pressure_drag_coefficient": 0.0},
        "control_effectiveness": 0.7,
    }


def _number(container, label: str, value: float, key: str, minimum: float | None = None, maximum: float | None = None) -> float:
    options = {"value": float(value), "key": key, "format": "%.6f"}
    if minimum is not None:
        options["min_value"] = float(minimum)
    if maximum is not None:
        options["max_value"] = float(maximum)
    return float(container.number_input(label, **options))


def _edit_fin_set(container, original: dict, index: int) -> tuple[dict, str | None]:
    value = copy.deepcopy(original)
    prefix = f"fin_{index}"
    with container.expander(f"Fin Set: {original.get('id', index + 1)}", expanded=True):
        value["id"] = st.text_input("Identifier", str(original.get("id", "")), key=f"{prefix}_id")
        value["station_m"] = _number(st, "Station (m)", original.get("station_m", 0), f"{prefix}_station", 0)
        counts = (3, 4, 6)
        value["count"] = st.selectbox("Fin count", counts, counts.index(original.get("count", 4)), key=f"{prefix}_count")
        angular_offset_degrees = _number(
            st,
            "Angular offset (degrees)",
            math.degrees(original.get("angular_offset_rad", 0)),
            f"{prefix}_offset",
            0,
            360 - 1e-9,
        )
        value["angular_offset_rad"] = math.radians(angular_offset_degrees)
        geometry = value.setdefault("geometry", {})
        source_geometry = original.get("geometry", {})
        for field, label, minimum in (("root_chord_m", "Root chord (m)", 1e-9), ("tip_chord_m", "Tip chord (m)", 1e-9), ("span_m", "Span (m)", 1e-9), ("sweep_m", "Aft sweep (m)", 0), ("thickness_m", "Thickness (m)", 1e-9)):
            geometry[field] = _number(st, label, source_geometry.get(field, 0.1), f"{prefix}_{field}", minimum)
        source_actuation = original.get("actuation", {"type": "fixed"})
        modes = ("fixed", "all_moving", "control_surface")
        mode = st.selectbox("Actuation", modes, modes.index(source_actuation.get("type", "fixed")), key=f"{prefix}_type")
        actuation: dict[str, float | str] = {"type": mode}
        if mode != "fixed":
            actuation["minimum_deflection_rad"] = _number(st, "Minimum deflection (rad)", source_actuation.get("minimum_deflection_rad", -0.35), f"{prefix}_min", -math.tau, 0)
            actuation["maximum_deflection_rad"] = _number(st, "Maximum deflection (rad)", source_actuation.get("maximum_deflection_rad", 0.35), f"{prefix}_max", 0, math.tau)
        if mode == "control_surface":
            actuation["span_start_fraction"] = _number(st, "Control-surface span start", source_actuation.get("span_start_fraction", 0.2), f"{prefix}_span_start", 0, 0.999999)
            actuation["hinge_fraction"] = _number(st, "Hinge fraction", source_actuation.get("hinge_fraction", 0.7), f"{prefix}_hinge", 1e-6, 0.999999)
        value["actuation"] = actuation
        aero = value.setdefault("aerodynamics", {})
        source_aero = original.get("aerodynamics", {})
        st.caption("Aerodynamics: coefficients default to zero; force model is subsonic and low-angle only.")
        aero["skin_friction_coefficient"] = _number(st, "Skin-friction coefficient Cf", source_aero.get("skin_friction_coefficient", 0), f"{prefix}_skin_friction", 0)
        aero["pressure_drag_coefficient"] = _number(st, "Pressure-drag coefficient Cdp", source_aero.get("pressure_drag_coefficient", 0), f"{prefix}_pressure_drag", 0)
        value["control_effectiveness"] = _number(st, "Control effectiveness", original.get("control_effectiveness", 0.7), f"{prefix}_control_effectiveness", 0, 1)
        controls = container.columns(4)
        action = "remove" if controls[0].button("Remove", key=f"{prefix}_remove") else None
        action = action or ("duplicate" if controls[1].button("Duplicate", key=f"{prefix}_duplicate") else None)
        action = action or ("up" if index > 0 and controls[2].button("Move up", key=f"{prefix}_up") else None)
        action = action or ("down" if controls[3].button("Move down", key=f"{prefix}_down") else None)
    return value, action


def _editor(draft: dict) -> tuple[dict, bool]:
    result = copy.deepcopy(draft)
    rocket = result["rocket"]
    st.sidebar.header("Rocket definition")
    rocket["id"] = st.sidebar.text_input("Rocket identifier", rocket["id"], key="rocket_id")
    rocket["name"] = st.sidebar.text_input("Rocket name", rocket["name"], key="rocket_name")
    st.sidebar.subheader("Body Tube")
    rocket["body"]["length_m"] = _number(st.sidebar, "Body length (m)", rocket["body"]["length_m"], "body_length", 1e-9)
    rocket["body"]["diameter_m"] = _number(st.sidebar, "Body diameter (m)", rocket["body"]["diameter_m"], "body_diameter", 1e-9)
    st.sidebar.subheader("Nose")
    rocket["nose"]["length_m"] = _number(st.sidebar, "Nose length (m)", rocket["nose"]["length_m"], "nose_length", 1e-9)
    st.sidebar.caption("Half-prolate ellipsoid; its base matches the Body Tube.")
    st.sidebar.subheader("Aerodynamics")
    for component_name, component in (("Body Tube", rocket["body"]), ("Nose", rocket["nose"])):
        aero = component.setdefault("aerodynamics", {})
        st.sidebar.caption(f"{component_name} drag coefficients (default: 0)")
        aero["skin_friction_coefficient"] = _number(st.sidebar, f"{component_name} skin-friction Cf", aero.get("skin_friction_coefficient", 0), f"{component_name}_skin_friction", 0)
        aero["pressure_drag_coefficient"] = _number(st.sidebar, f"{component_name} pressure-drag Cdp", aero.get("pressure_drag_coefficient", 0), f"{component_name}_pressure_drag", 0)
    st.sidebar.subheader("Mass Properties")
    mass = rocket["mass_properties"]
    mass["mass_kg"] = _number(st.sidebar, "Mass (kg)", mass["mass_kg"], "mass_kg", 1e-9)
    maximum_cg_station_m = rocket["body"]["length_m"] + rocket["nose"]["length_m"]
    if constrain_draft_center_of_mass(result):
        st.sidebar.info("Center of mass was moved to the new nose tip after resizing the rocket.")
    if st.session_state.get("center_of_mass_station_m", 0.0) > maximum_cg_station_m:
        st.session_state["center_of_mass_station_m"] = maximum_cg_station_m
    mass["center_of_mass_station_m"] = _number(
        st.sidebar,
        "Center of mass station (m)",
        mass["center_of_mass_station_m"],
        "center_of_mass_station_m",
        0,
        maximum_cg_station_m,
    )
    st.sidebar.caption("Principal moments about the center of mass (kg·m²).")
    mass["inertia_xx_kg_m2"] = _number(st.sidebar, "Ixx (kg·m²)", mass["inertia_xx_kg_m2"], "inertia_xx", 1e-12)
    mass["inertia_yy_kg_m2"] = _number(st.sidebar, "Iyy (kg·m²)", mass["inertia_yy_kg_m2"], "inertia_yy", 1e-12)
    mass["inertia_zz_kg_m2"] = _number(st.sidebar, "Izz (kg·m²)", mass["inertia_zz_kg_m2"], "inertia_zz", 1e-12)
    st.sidebar.subheader("Fin Sets")
    fin_sets: list[dict] = []
    requested: tuple[str, int] | None = None
    for index, fin_set in enumerate(rocket["fin_sets"]):
        updated, action = _edit_fin_set(st.sidebar, fin_set, index)
        fin_sets.append(updated)
        if action and requested is None:
            requested = (action, index)
    rocket["fin_sets"] = fin_sets
    if st.sidebar.button("Add Fin Set"):
        fin_sets.append(_new_fin_set(len(fin_sets) + 1))
        return result, True
    if requested:
        action, index = requested
        if action == "remove":
            fin_sets.pop(index)
        elif action == "duplicate":
            clone = copy.deepcopy(fin_sets[index]); clone["id"] = f"{clone['id']}_copy"; fin_sets.insert(index + 1, clone)
        elif action == "up":
            fin_sets[index - 1], fin_sets[index] = fin_sets[index], fin_sets[index - 1]
        elif action == "down" and index < len(fin_sets) - 1:
            fin_sets[index + 1], fin_sets[index] = fin_sets[index], fin_sets[index + 1]
        return result, True
    return result, False


def _preview_pose(definition: RocketDefinition, reset: bool) -> dict[str, float]:
    if reset:
        for key in list(st.session_state):
            if key.startswith("pose_"):
                st.session_state[key] = 0.0
    pose: dict[str, float] = {}
    st.sidebar.subheader("Preview Pose")
    for fin_set in definition.rocket.fin_sets:
        if fin_set.actuation.type == "fixed":
            continue
        noun = "fin" if fin_set.actuation.type == "all_moving" else "control_surface"
        minimum = math.degrees(fin_set.actuation.minimum_deflection_rad or 0)
        maximum = math.degrees(fin_set.actuation.maximum_deflection_rad or 0)
        for index in range(fin_set.count):
            actuator = f"{fin_set.id}_{noun}_{index}"
            degrees = st.sidebar.slider(actuator, minimum, maximum, 0.0, key=f"pose_{actuator}", format="%.2f°")
            pose[actuator] = math.radians(degrees)
    return pose


def run() -> None:
    st.set_page_config(page_title="Rocket Geometry", layout="wide")
    st.title("Rocket Geometry Editor")
    if "draft" not in st.session_state:
        initial = load_definition(_example_path())
        st.session_state.draft, st.session_state.last_valid = initial.model_dump(), initial
    uploaded = st.sidebar.file_uploader("Load rocket.yaml", type="yaml")
    signature = (uploaded.name, uploaded.size) if uploaded is not None else None
    if uploaded is not None and signature != st.session_state.get("uploaded_signature"):
        try:
            loaded = load_definition_from_bytes(uploaded.getvalue())
            st.session_state.draft, st.session_state.last_valid = loaded.model_dump(), loaded
            st.session_state.uploaded_signature = signature
            st.sidebar.success("Rocket loaded")
        except Exception as error:
            st.sidebar.error(str(error))
    draft, topology_changed = _editor(st.session_state.draft)
    st.session_state.draft = draft
    if constrain_draft_center_of_mass(draft):
        st.sidebar.info("Center of mass was moved to the new nose tip after resizing the rocket.")
    try:
        definition, error = RocketDefinition.model_validate(draft), None
        st.session_state.last_valid = definition
    except Exception as validation_error:
        definition, error = st.session_state.last_valid, validation_error
    reset = topology_changed or st.sidebar.button("Reset all preview deflections")
    pose = _preview_pose(definition, reset)
    if error:
        st.error(f"Invalid geometry; showing the last valid rocket. {error}")
    figure = plotly_figure(definition, pose)
    figure.update_layout(height=980)
    st.plotly_chart(figure, use_container_width=True)

    from rocket_model.barrowman import calculate_barrowman, static_margin_calibers

    result = calculate_barrowman(definition)
    mass = definition.rocket.mass_properties
    st.subheader("Neutral Barrowman CP and Mass Properties")
    cp_column, cm_column, margin_column, mass_column = st.columns(4)
    cp_column.metric("Rocket CP station", f"{result.rocket_cp_station_m:.4f} m")
    cm_column.metric("Center of mass station", f"{mass.center_of_mass_station_m:.4f} m")
    margin = static_margin_calibers(definition, result)
    margin_column.metric("Static margin", f"{margin:.2f} calibers" if margin is not None else "N/A")
    mass_column.metric("Mass", f"{mass.mass_kg:.3f} kg")
    st.caption("Near-zero angle, steady subsonic, neutral controls only. Moments of inertia are user-configured about the center of mass.")
    with st.expander("How CP location is calculated"):
        st.markdown(r"""
For the half-ellipsoid nose, the neutral CP station is `body_length_m + 2 × nose_length_m / 3`.

For each trapezoidal Fin Set, the neutral CP uses the simplified Barrowman mean-aerodynamic-chord result. It depends on its Fin Station, root and tip chords, and aft sweep. The fin normal-force slope also accounts for fin count, span, body diameter, and body-fin interference.

The Rocket CP station is the normal-force-slope-weighted average of the Nose and all Fin Set CP stations:

`Rocket CP = Σ(CNαᵢ × CPᵢ) / Σ(CNαᵢ)`.

The green cross is the configured CG / center of mass. Static margin is `(CG − CP) / body diameter`; a positive result means CP is aft of CG in this Aft-Datum coordinate system.
        """)
    st.json({"mass_properties": mass.model_dump(), "barrowman": {"nose": result.nose.__dict__, "fin_sets": [item.__dict__ for item in result.fin_sets]}})
    if error is None:
        st.download_button("Download rocket.yaml", dump_definition(definition), "rocket.yaml", "application/yaml")


if __name__ == "__main__":
    run()
