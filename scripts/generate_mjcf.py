#!/usr/bin/env python3
"""Generate the Lite MuJoCo model from the xacro-derived URDF.

The MuJoCo compiler is the source of truth for URDF joint/body transforms.
This script then adds visual meshes, preserved task-frame sites, gripper
mimic constraints, and named actuators required by the ROS 2 sim stack.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import trimesh
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
CONTROLLED_JOINTS = (
    ("left_shoulder_pitch", "-17 17", "0.009", "0.1", "0.1"),
    ("left_shoulder_roll", "-14 14", "0.009", "0.1", "0.1"),
    ("left_shoulder_yaw", "-14 14", "0.009", "0.1", "0.1"),
    ("left_elbow_pitch", "-14 14", "0.009", "0.1", "0.1"),
    ("left_wrist_yaw", "-4 4", "0.005", "0.1", "0.1"),
    ("left_wrist_roll", "-4 4", "0.005", "0.1", "0.1"),
    ("left_wrist_pitch", "-4 4", "0.005", "0.1", "0.1"),
    ("right_shoulder_pitch", "-17 17", "0.009", "0.1", "0.1"),
    ("right_shoulder_roll", "-14 14", "0.009", "0.1", "0.1"),
    ("right_shoulder_yaw", "-14 14", "0.009", "0.1", "0.1"),
    ("right_elbow_pitch", "-14 14", "0.009", "0.1", "0.1"),
    ("right_wrist_yaw", "-4 4", "0.005", "0.1", "0.1"),
    ("right_wrist_roll", "-4 4", "0.005", "0.1", "0.1"),
    ("right_wrist_pitch", "-4 4", "0.005", "0.1", "0.1"),
    ("left_gripper", "-4 4", "0.001", "0.02", "0.01"),
    ("right_gripper", "-4 4", "0.001", "0.02", "0.01"),
)
PASSIVE_JOINTS = {
    "left_gripper_passive_joint": ("0.001", "0.02", "0.01"),
    "right_gripper_passive_joint": ("0.001", "0.02", "0.01"),
}
PRESERVED_SITE_LINKS = {
    "head_camera_link",
    "left_wrist_camera_link",
    "right_wrist_camera_link",
    "left_gripper_tip_middle_link",
    "right_gripper_tip_middle_link",
}
OVERSIZED_VISUAL_MESHES = {
    "world_root_visual.stl": "world_root_visual_mujoco.stl",
    "left_wrist_yaw_motor_link_visual.stl": "left_wrist_yaw_motor_link_visual_mujoco.stl",
    "right_wrist_yaw_motor_link_visual.stl": "right_wrist_yaw_motor_link_visual_mujoco.stl",
}
MAX_MUJOCO_STL_FACES = 190_000


def run(cmd: list[str], **kwargs) -> None:
    print("+", " ".join(cmd))
    subprocess.run(cmd, check=True, **kwargs)


def parse_xyz(value: str | None) -> list[float]:
    if not value:
        return [0.0, 0.0, 0.0]
    return [float(part) for part in value.split()]


def fmt(values: list[float]) -> str:
    return " ".join(f"{value:.10g}" for value in values)


def quat_from_rpy(rpy: list[float]) -> list[float]:
    quat_xyzw = Rotation.from_euler("xyz", rpy).as_quat()
    return [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]


def origin_attrs(origin: ET.Element | None) -> dict[str, str]:
    attrs: dict[str, str] = {}
    if origin is None:
        return attrs
    xyz = parse_xyz(origin.get("xyz"))
    rpy = parse_xyz(origin.get("rpy"))
    if any(abs(value) > 1e-12 for value in xyz):
        attrs["pos"] = fmt(xyz)
    if any(abs(value) > 1e-12 for value in rpy):
        attrs["quat"] = fmt(quat_from_rpy(rpy))
    return attrs


def ensure_decimated_meshes() -> None:
    for source_name, target_name in OVERSIZED_VISUAL_MESHES.items():
        source = ROOT / "meshes" / source_name
        target = ROOT / "meshes" / target_name
        mesh = trimesh.load_mesh(source, process=False)
        if len(mesh.faces) <= MAX_MUJOCO_STL_FACES:
            shutil.copy2(source, target)
            continue
        simplified = mesh.simplify_quadric_decimation(face_count=MAX_MUJOCO_STL_FACES)
        simplified.export(target)
        print(f"decimated {source_name}: {len(mesh.faces)} -> {len(simplified.faces)} faces")


def write_temp_urdf(urdf_path: Path, tmp_dir: Path) -> Path:
    source_urdf = tmp_dir / "lite_source.urdf"
    with source_urdf.open("w", encoding="utf-8") as handle:
        run(
            [
                "xacro",
                str(urdf_path),
                "emit_ros2_control:=false",
            ],
            stdout=handle,
        )

    tree = ET.parse(source_urdf)
    root = tree.getroot()
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            inertial = ET.SubElement(link, "inertial")
        mass = inertial.find("mass")
        if mass is None:
            mass = ET.SubElement(inertial, "mass")
        try:
            mass_value = float(mass.get("value", "0"))
        except ValueError:
            mass_value = 0.0
        if mass_value <= 0:
            mass.set("value", "0.001")
        inertia = inertial.find("inertia")
        if inertia is None:
            inertia = ET.SubElement(inertial, "inertia")
        inertia.set("ixx", "0.0001")
        inertia.set("iyy", "0.0001")
        inertia.set("izz", "0.0001")
        inertia.set("ixy", "0")
        inertia.set("ixz", "0")
        inertia.set("iyz", "0")

    tree.write(source_urdf, encoding="utf-8", xml_declaration=True)
    return source_urdf


def symlink_meshes(tmp_dir: Path) -> None:
    for mesh in (ROOT / "meshes").glob("*.stl"):
        target = tmp_dir / mesh.name
        if not target.exists():
            target.symlink_to(mesh)


def compile_urdf(compile_bin: str, source_urdf: Path, tmp_dir: Path) -> Path:
    compiled_xml = tmp_dir / "lite_compiled.xml"
    run([compile_bin, str(source_urdf), str(compiled_xml)])
    return compiled_xml


def body_map(worldbody: ET.Element) -> dict[str, ET.Element]:
    bodies: dict[str, ET.Element] = {"world_root": worldbody}

    def visit(body: ET.Element) -> None:
        name = body.get("name")
        if name:
            bodies[name] = body
        for child in body.findall("body"):
            visit(child)

    for body in worldbody.findall("body"):
        visit(body)
    return bodies


def insert_before_child_body(parent: ET.Element, child: ET.Element) -> None:
    for index, existing in enumerate(list(parent)):
        if existing.tag == "body":
            parent.insert(index, child)
            return
    parent.append(child)


def asset_name_for_mesh(filename: str) -> tuple[str, str]:
    basename = Path(filename).name
    file_name = OVERSIZED_VISUAL_MESHES.get(basename, basename)
    return Path(file_name).stem, file_name


def material_rgba(visual: ET.Element) -> str | None:
    color = visual.find("material/color")
    if color is None:
        return None
    return color.get("rgba")


def add_visuals(
    urdf_root: ET.Element,
    mjcf_root: ET.Element,
    bodies: dict[str, ET.Element],
) -> None:
    asset = mjcf_root.find("asset")
    if asset is None:
        asset = ET.Element("asset")
        compiler = mjcf_root.find("compiler")
        insert_at = list(mjcf_root).index(compiler) + 1 if compiler is not None else 0
        mjcf_root.insert(insert_at, asset)
    existing_assets = {mesh.get("name") for mesh in asset.findall("mesh")}

    for link in urdf_root.findall("link"):
        link_name = link.get("name")
        body = bodies.get(link_name or "")
        if body is None:
            continue
        for visual_index, visual in enumerate(link.findall("visual")):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            asset_name, file_name = asset_name_for_mesh(mesh.get("filename", ""))
            if asset_name not in existing_assets:
                ET.SubElement(asset, "mesh", {"name": asset_name, "file": file_name})
                existing_assets.add(asset_name)

            geom_attrs = {
                "name": f"{link_name}_visual_{visual_index}",
                "type": "mesh",
                "mesh": asset_name,
                "contype": "0",
                "conaffinity": "0",
                "density": "0",
                "group": "1",
            }
            rgba = material_rgba(visual)
            if rgba:
                geom_attrs["rgba"] = rgba
            geom_attrs.update(origin_attrs(visual.find("origin")))
            insert_before_child_body(body, ET.Element("geom", geom_attrs))


def mark_collision_geoms(worldbody: ET.Element) -> None:
    for geom in worldbody.iter("geom"):
        geom.set("group", "3")
        geom.set("rgba", "0.35 0.35 0.35 0.25")


def add_sites(urdf_root: ET.Element, bodies: dict[str, ET.Element]) -> None:
    for joint in urdf_root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        child = joint.find("child")
        parent = joint.find("parent")
        if child is None or parent is None:
            continue
        child_name = child.get("link", "")
        if child_name not in PRESERVED_SITE_LINKS:
            continue
        body = bodies.get(parent.get("link", ""))
        if body is None:
            continue
        attrs = {
            "name": child_name,
            "size": "0.01",
            "rgba": "0.1 0.4 1 0.6",
        }
        attrs.update(origin_attrs(joint.find("origin")))
        insert_before_child_body(body, ET.Element("site", attrs))


def tune_joints(worldbody: ET.Element) -> None:
    controlled = {
        name: (force_range, armature, damping, friction)
        for name, force_range, armature, damping, friction in CONTROLLED_JOINTS
    }
    for joint in worldbody.iter("joint"):
        name = joint.get("name", "")
        joint.attrib.pop("actuatorfrcrange", None)
        if name in controlled:
            force_range, armature, damping, friction = controlled[name]
            joint.set("actuatorfrcrange", force_range)
            joint.set("armature", armature)
            joint.set("damping", damping)
            joint.set("frictionloss", friction)
        elif name in PASSIVE_JOINTS:
            armature, damping, friction = PASSIVE_JOINTS[name]
            joint.set("armature", armature)
            joint.set("damping", damping)
            joint.set("frictionloss", friction)


def add_constraints_and_actuators(
    urdf_root: ET.Element,
    mjcf_root: ET.Element,
    bodies: dict[str, ET.Element],
) -> None:
    contact = ET.Element("contact")
    for joint in urdf_root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name = parent.get("link", "")
        child_name = child.get("link", "")
        if parent_name == "world_root" or parent_name not in bodies or child_name not in bodies:
            continue
        ET.SubElement(contact, "exclude", {"body1": parent_name, "body2": child_name})
    if list(contact):
        mjcf_root.append(contact)

    equality = ET.Element("equality")
    for joint in urdf_root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        ET.SubElement(
            equality,
            "joint",
            {
                "joint1": joint.get("name", ""),
                "joint2": mimic.get("joint", ""),
                "polycoef": f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0",
            },
        )
    if list(equality):
        mjcf_root.append(equality)

    actuator = ET.Element("actuator")
    for name, force_range, _armature, _damping, _friction in CONTROLLED_JOINTS:
        ET.SubElement(actuator, "motor", {"name": name, "joint": name, "forcerange": force_range})
    mjcf_root.append(actuator)


def add_sim_options(mjcf_root: ET.Element) -> None:
    mjcf_root.set("model", "bar_lite_arms_grippers")
    compiler = mjcf_root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        mjcf_root.insert(0, compiler)
    compiler.set("angle", "radian")
    compiler.set("meshdir", "../meshes/")

    option = ET.Element(
        "option",
        {
            "timestep": "0.001",
            "integrator": "implicitfast",
            "solver": "Newton",
            "cone": "elliptic",
            "impratio": "10",
        },
    )
    visual = ET.Element("visual")
    ET.SubElement(
        visual,
        "headlight",
        {
            "ambient": "0.7 0.7 0.7",
            "diffuse": "0.25 0.25 0.25",
            "specular": "0.1 0.1 0.1",
        },
    )
    ET.SubElement(visual, "map", {"znear": "0.01"})
    insert_at = list(mjcf_root).index(compiler) + 1
    mjcf_root.insert(insert_at, option)
    mjcf_root.insert(insert_at + 1, visual)


def postprocess(compiled_xml: Path, source_urdf: Path, output_xml: Path) -> None:
    urdf_root = ET.parse(source_urdf).getroot()
    mjcf_tree = ET.parse(compiled_xml)
    mjcf_root = mjcf_tree.getroot()
    worldbody = mjcf_root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("compiled MJCF has no worldbody")

    bodies = body_map(worldbody)
    mark_collision_geoms(worldbody)
    add_sim_options(mjcf_root)
    add_visuals(urdf_root, mjcf_root, bodies)
    add_sites(urdf_root, bodies)
    tune_joints(worldbody)
    add_constraints_and_actuators(urdf_root, mjcf_root, bodies)

    ET.indent(mjcf_tree, space="  ")
    mjcf_tree.write(output_xml, encoding="unicode", xml_declaration=True)
    output_xml.write_text(output_xml.read_text(encoding="utf-8") + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xacro", default=str(ROOT / "urdf" / "lite.urdf.xacro"))
    parser.add_argument("--output", default=str(ROOT / "mjcf" / "lite.xml"))
    parser.add_argument(
        "--compile-bin",
        default=os.environ.get("MUJOCO_COMPILE", shutil.which("compile") or "compile"),
    )
    args = parser.parse_args()

    ensure_decimated_meshes()
    with tempfile.TemporaryDirectory(prefix="lite_mjcf_") as tmp:
        tmp_dir = Path(tmp)
        symlink_meshes(tmp_dir)
        source_urdf = write_temp_urdf(Path(args.xacro), tmp_dir)
        compiled_xml = compile_urdf(args.compile_bin, source_urdf, tmp_dir)
        postprocess(compiled_xml, source_urdf, Path(args.output))


if __name__ == "__main__":
    main()
