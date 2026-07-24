#!/usr/bin/env python3
"""Export para_Labrefer identification results as physically loadable URDF candidates."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PARAM_ORDER = ("mass", "mx", "my", "mz", "Ixx", "Ixy", "Ixz", "Iyy", "Iyz", "Izz")
DEFAULT_SOURCES = (
    "lite_flash_arm_gripper.urdf",
    "lite_flash_arm_gripper_headless.urdf",
    "lite_pro_arm_gripper.urdf",
    "lite_pro_arm_gripper_headless.urdf",
)

# The identification xacro uses the opposite coordinate for this joint. Dynamic
# damping and Coulomb magnitudes are invariant; a constant effort term is not.
TARGET_COORDINATE_SIGN = {"right_elbow_pitch": -1.0}
REFERENCE_PARAMS = REPO_ROOT / "para_Labrefer" / "identified_params.json"
AXIS_ALIGNED_NEGLIGIBLE_MASS_BODIES = {
    "left_shoulder_pitch",
    "right_shoulder_pitch",
}
NEGLIGIBLE_MASS_THRESHOLD_KG = 1e-8


@dataclass(frozen=True)
class ProjectedInertia:
    body: str
    mass: float
    com: np.ndarray
    inertia_com: np.ndarray
    raw_mass: float
    raw_pseudo_min_eigenvalue: float


@dataclass(frozen=True)
class ExportRow:
    source: str
    body: str
    target_joint: str
    target_link: str
    action: str
    raw_mass: float
    output_mass: float
    output_com: tuple[float, float, float]
    raw_pseudo_min_eigenvalue: float
    output_inertia_min_eigenvalue: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Project para_Labrefer inertias and export gripper URDF candidates."
    )
    parser.add_argument(
        "--params",
        type=Path,
        default=REFERENCE_PARAMS,
    )
    parser.add_argument("--urdf-dir", type=Path, default=REPO_ROOT / "urdf")
    parser.add_argument("--output-dir", type=Path, default=REPO_ROOT / "urdf")
    parser.add_argument(
        "--report",
        type=Path,
        default=REPO_ROOT / "urdf" / "PARA_LABREFER_EXPORT_REPORT.md",
    )
    parser.add_argument("--projection-epsilon", type=float, default=1e-8)
    parser.add_argument("--max-com-norm", type=float, default=0.30)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.projection_epsilon <= 0.0 or not math.isfinite(args.projection_epsilon):
        raise SystemExit("--projection-epsilon must be finite and positive")
    if args.max_com_norm <= 0.0 or not math.isfinite(args.max_com_norm):
        raise SystemExit("--max-com-norm must be finite and positive")

    payload = json.loads(args.params.read_text(encoding="utf-8"))
    inertial = payload.get("parameters", {}).get("inertial", {})
    drive = payload.get("parameters", {}).get("drive", {})
    if not inertial or not drive:
        raise SystemExit(f"missing parameters.inertial or parameters.drive in {args.params}")

    projected = project_all_inertias(inertial, epsilon=args.projection_epsilon)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[ExportRow] = []
    outputs: list[Path] = []
    for source_name in DEFAULT_SOURCES:
        source = args.urdf_dir / source_name
        output = args.output_dir / source_name.replace(".urdf", "_para_Labrefer.urdf")
        rows.extend(
            export_one(
                source=source,
                output=output,
                params_path=args.params,
                projected=projected,
                drive=drive,
                max_com_norm=args.max_com_norm,
                inertia_epsilon=args.projection_epsilon,
            )
        )
        outputs.append(output)

    write_report(
        args.report,
        params_path=args.params,
        payload=payload,
        outputs=outputs,
        rows=rows,
        drive=drive,
        max_com_norm=args.max_com_norm,
        projection_epsilon=args.projection_epsilon,
    )
    print(f"Wrote {len(outputs)} URDF candidates")
    print(f"Wrote {args.report}")


def project_all_inertias(
    inertial: dict[str, float], *, epsilon: float
) -> dict[str, ProjectedInertia]:
    bodies = sorted({key.split("/")[1] for key in inertial if key.startswith("inertial/")})
    result: dict[str, ProjectedInertia] = {}
    for body in bodies:
        prefix = f"inertial/{body}/"
        if not all(prefix + name in inertial for name in PARAM_ORDER):
            continue
        values = {name: float(inertial[prefix + name]) for name in PARAM_ORDER}
        raw_mass = values["mass"]
        h = np.asarray([values["mx"], values["my"], values["mz"]], dtype=float)
        inertia_origin = np.asarray(
            [
                [values["Ixx"], values["Ixy"], values["Ixz"]],
                [values["Ixy"], values["Iyy"], values["Iyz"]],
                [values["Ixz"], values["Iyz"], values["Izz"]],
            ],
            dtype=float,
        )
        pseudo = dynamic_to_pseudo_inertia(raw_mass, h, inertia_origin)
        eigenvalues, eigenvectors = np.linalg.eigh(pseudo)
        projected_pseudo = (eigenvectors * np.maximum(eigenvalues, epsilon)) @ eigenvectors.T
        projected_pseudo = symmetrize(projected_pseudo)
        sigma = projected_pseudo[:3, :3]
        projected_h = projected_pseudo[:3, 3]
        mass = max(float(projected_pseudo[3, 3]), epsilon)
        inertia_origin = np.trace(sigma) * np.eye(3) - sigma
        com = projected_h / mass
        inertia_com = inertia_origin - mass * (
            (com @ com) * np.eye(3) - np.outer(com, com)
        )
        inertia_com = regularize_com_inertia(inertia_com, epsilon=epsilon)
        result[body] = ProjectedInertia(
            body=body,
            mass=mass,
            com=com,
            inertia_com=inertia_com,
            raw_mass=raw_mass,
            raw_pseudo_min_eigenvalue=float(np.min(eigenvalues)),
        )
    return result


def dynamic_to_pseudo_inertia(
    mass: float, first_moment: np.ndarray, inertia_origin: np.ndarray
) -> np.ndarray:
    sigma = 0.5 * np.trace(inertia_origin) * np.eye(3) - inertia_origin
    pseudo = np.zeros((4, 4), dtype=float)
    pseudo[:3, :3] = sigma
    pseudo[:3, 3] = first_moment
    pseudo[3, :3] = first_moment
    pseudo[3, 3] = mass
    return symmetrize(pseudo)


def regularize_com_inertia(inertia: np.ndarray, *, epsilon: float) -> np.ndarray:
    inertia = symmetrize(inertia)
    eigenvalues, eigenvectors = np.linalg.eigh(inertia)
    inertia = (eigenvectors * np.maximum(eigenvalues, epsilon)) @ eigenvectors.T
    return symmetrize(inertia)


def export_one(
    *,
    source: Path,
    output: Path,
    params_path: Path,
    projected: dict[str, ProjectedInertia],
    drive: dict[str, dict[str, float]],
    max_com_norm: float,
    inertia_epsilon: float,
) -> list[ExportRow]:
    tree = ET.parse(source)
    root = tree.getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    links = {link.attrib["name"]: link for link in root.findall("link")}
    rows: list[ExportRow] = []

    for body, candidate in sorted(projected.items()):
        target_joint = target_joint_name(body)
        joint = joints.get(target_joint)
        if joint is None:
            continue
        child = joint.find("child")
        if child is None:
            raise RuntimeError(f"joint {target_joint} in {source} has no child")
        target_link = child.attrib["link"]
        link = links[target_link]
        projected_com_norm = float(np.linalg.norm(candidate.com))
        if (
            body in AXIS_ALIGNED_NEGLIGIBLE_MASS_BODIES
            and abs(candidate.raw_mass) <= NEGLIGIBLE_MASS_THRESHOLD_KG
        ):
            regularize_existing_inertia(link, epsilon=inertia_epsilon)
            align_existing_inertial_origin_to_joint_axis(link, joint)
            action = "keep_nominal_axis_aligned_regularized"
        elif projected_com_norm <= max_com_norm:
            set_link_inertial(link, candidate.mass, candidate.com, candidate.inertia_com)
            action = "write_projected"
        else:
            regularize_existing_inertia(link, epsilon=inertia_epsilon)
            action = "keep_nominal_far_com_regularized"
        mass, com, inertia_com = read_link_inertial(link)
        rows.append(
            ExportRow(
                source=source.name,
                body=body,
                target_joint=target_joint,
                target_link=target_link,
                action=action,
                raw_mass=candidate.raw_mass,
                output_mass=mass,
                output_com=tuple(float(value) for value in com),
                raw_pseudo_min_eigenvalue=candidate.raw_pseudo_min_eigenvalue,
                output_inertia_min_eigenvalue=float(np.min(np.linalg.eigvalsh(inertia_com))),
            )
        )

    for identified_joint, terms in sorted(drive.items()):
        target_joint = target_joint_name(identified_joint)
        joint = joints.get(target_joint)
        if joint is None:
            raise RuntimeError(f"identified drive joint {target_joint} missing from {source}")
        dynamics = joint.find("dynamics")
        if dynamics is None:
            axis = joint.find("axis")
            insert_at = list(joint).index(axis) + 1 if axis is not None else len(joint)
            dynamics = ET.Element("dynamics")
            joint.insert(insert_at, dynamics)
        dynamics.attrib["damping"] = number(max(0.0, float(terms["viscous"])))
        dynamics.attrib["friction"] = number(max(0.0, float(terms["coulomb"])))

    root.insert(
        0,
        ET.Comment(
            " para_Labrefer dynamics candidate; inertias are PSD-projected, safety-gated, "
            "and axis-constrained for negligible shoulder-pitch groups. "
            f"Source parameters: {params_path.name}. Review urdf/PARA_LABREFER_EXPORT_REPORT.md. "
        ),
    )
    validate_candidate(root, rows=rows, projected=projected, drive=drive, source=source)
    ET.indent(tree, space="    ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return rows


def target_joint_name(identified_name: str) -> str:
    if identified_name.endswith("_passive_joint"):
        return identified_name
    return f"{identified_name}_joint"


def set_link_inertial(
    link: ET.Element, mass: float, com: np.ndarray, inertia_com: np.ndarray
) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        inertial = ET.Element("inertial")
        link.insert(0, inertial)
    origin = inertial.find("origin")
    if origin is None:
        origin = ET.SubElement(inertial, "origin")
    origin.attrib.update({"xyz": vector(com), "rpy": "0 0 0"})
    mass_element = inertial.find("mass")
    if mass_element is None:
        mass_element = ET.SubElement(inertial, "mass")
    mass_element.attrib["value"] = number(mass)
    inertia = inertial.find("inertia")
    if inertia is None:
        inertia = ET.SubElement(inertial, "inertia")
    inertia.attrib.update(
        {
            "ixx": number(inertia_com[0, 0]),
            "ixy": number(inertia_com[0, 1]),
            "ixz": number(inertia_com[0, 2]),
            "iyy": number(inertia_com[1, 1]),
            "iyz": number(inertia_com[1, 2]),
            "izz": number(inertia_com[2, 2]),
        }
    )


def regularize_existing_inertia(link: ET.Element, *, epsilon: float) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        raise RuntimeError(f"fallback link {link.attrib['name']} has no nominal inertia")
    inertia = inertial.find("inertia")
    if inertia is None:
        raise RuntimeError(f"fallback link {link.attrib['name']} has no inertia tensor")
    matrix = inertia_matrix(inertia)
    matrix = regularize_com_inertia(matrix, epsilon=epsilon)
    inertia.attrib.update(
        {
            "ixx": number(matrix[0, 0]),
            "ixy": number(matrix[0, 1]),
            "ixz": number(matrix[0, 2]),
            "iyy": number(matrix[1, 1]),
            "iyz": number(matrix[1, 2]),
            "izz": number(matrix[2, 2]),
        }
    )


def align_existing_inertial_origin_to_joint_axis(
    link: ET.Element, joint: ET.Element
) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        raise RuntimeError(f"axis-aligned link {link.attrib['name']} has no nominal inertia")
    origin = inertial.find("origin")
    axis_element = joint.find("axis")
    if origin is None or axis_element is None:
        raise RuntimeError(f"axis-aligned joint {joint.attrib['name']} is missing inertial origin or axis")
    origin_xyz = parse_vector(origin.attrib.get("xyz", "0 0 0"))
    axis = parse_vector(axis_element.attrib.get("xyz", "1 0 0"))
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm <= 0.0 or not math.isfinite(axis_norm):
        raise RuntimeError(f"axis-aligned joint {joint.attrib['name']} has invalid axis")
    axis /= axis_norm
    aligned_origin = axis * float(origin_xyz @ axis)
    aligned_origin[np.abs(aligned_origin) < 1e-15] = 0.0
    origin.attrib["xyz"] = vector(aligned_origin)


def read_link_inertial(link: ET.Element) -> tuple[float, np.ndarray, np.ndarray]:
    inertial = link.find("inertial")
    if inertial is None:
        raise RuntimeError(f"link {link.attrib['name']} has no inertia after export")
    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if origin is None or mass is None or inertia is None:
        raise RuntimeError(f"link {link.attrib['name']} has incomplete inertia after export")
    return (
        float(mass.attrib["value"]),
        parse_vector(origin.attrib.get("xyz", "0 0 0")),
        inertia_matrix(inertia),
    )


def validate_candidate(
    root: ET.Element,
    *,
    rows: list[ExportRow],
    projected: dict[str, ProjectedInertia],
    drive: dict[str, dict[str, float]],
    source: Path,
) -> None:
    exported_bodies = {row.body for row in rows}
    missing_bodies = sorted(set(projected) - exported_bodies)
    if missing_bodies:
        raise RuntimeError(f"{source.name} is missing projected bodies: {missing_bodies}")

    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    links = {link.attrib["name"]: link for link in root.findall("link")}
    for row in rows:
        if row.action != "keep_nominal_axis_aligned_regularized":
            continue
        joint = joints[row.target_joint]
        _, com, _ = read_link_inertial(links[row.target_link])
        axis_element = joint.find("axis")
        if axis_element is None:
            raise RuntimeError(f"{source.name} joint {row.target_joint} has no axis")
        axis = parse_vector(axis_element.attrib.get("xyz", "1 0 0"))
        axis /= np.linalg.norm(axis)
        perpendicular_com = com - axis * float(com @ axis)
        if np.linalg.norm(perpendicular_com) > 1e-12:
            raise RuntimeError(f"{source.name} link {row.target_link} COM is not on the joint axis")

    for identified_joint in drive:
        target_joint = target_joint_name(identified_joint)
        dynamics = joints[target_joint].find("dynamics")
        if dynamics is None:
            raise RuntimeError(f"{source.name} joint {target_joint} has no dynamics element")
        for attr in ("damping", "friction"):
            value = float(dynamics.attrib[attr])
            if value < 0.0 or not math.isfinite(value):
                raise RuntimeError(f"{source.name} joint {target_joint} has invalid {attr}")

    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass_element = inertial.find("mass")
        inertia_element = inertial.find("inertia")
        if mass_element is None or inertia_element is None:
            raise RuntimeError(f"{source.name} link {link.attrib['name']} has incomplete inertial")
        mass = float(mass_element.attrib["value"])
        if mass <= 0.0 or not math.isfinite(mass):
            raise RuntimeError(f"{source.name} link {link.attrib['name']} has invalid mass")
        eigenvalues = np.linalg.eigvalsh(inertia_matrix(inertia_element))
        if not np.all(np.isfinite(eigenvalues)) or np.min(eigenvalues) <= 0.0:
            raise RuntimeError(f"{source.name} link {link.attrib['name']} has non-positive inertia")
        moments = np.sort(eigenvalues)
        if moments[0] + moments[1] + 1e-12 < moments[2]:
            raise RuntimeError(f"{source.name} link {link.attrib['name']} violates inertia triangle")


def display_path(path: Path) -> str:
    resolved = path.resolve()
    try:
        return str(resolved.relative_to(REPO_ROOT))
    except ValueError:
        return str(resolved)


def inertia_matrix(inertia: ET.Element) -> np.ndarray:
    return np.asarray(
        [
            [float(inertia.attrib["ixx"]), float(inertia.attrib["ixy"]), float(inertia.attrib["ixz"])],
            [float(inertia.attrib["ixy"]), float(inertia.attrib["iyy"]), float(inertia.attrib["iyz"])],
            [float(inertia.attrib["ixz"]), float(inertia.attrib["iyz"]), float(inertia.attrib["izz"])],
        ]
    )


def parse_vector(value: str) -> np.ndarray:
    result = np.fromstring(value, sep=" ", dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise RuntimeError(f"expected a finite 3-vector, got: {value!r}")
    return result


def write_report(
    path: Path,
    *,
    params_path: Path,
    payload: dict[str, Any],
    outputs: list[Path],
    rows: list[ExportRow],
    drive: dict[str, dict[str, float]],
    max_com_norm: float,
    projection_epsilon: float,
) -> None:
    solver = payload.get("solver", {})
    representative = [row for row in rows if row.source == DEFAULT_SOURCES[0]]
    raw_nonphysical = sum(row.raw_pseudo_min_eigenvalue < 0.0 for row in representative)
    projected = sum(row.action == "write_projected" for row in representative)
    axis_aligned = sum(row.action == "keep_nominal_axis_aligned_regularized" for row in representative)
    fallback = len(representative) - projected - axis_aligned
    lines = [
        "# para_Labrefer URDF export report",
        "",
        "These files are physically loadable candidates, not a guarantee of the true",
        "per-link inertial distribution. The source least-squares problem is rank",
        "deficient, so its raw individual inertial values are not physically unique.",
        "",
        "The `para_Labrefer` suffix means the URDF is intended for the laboratory",
        "reference setup and specified assembly. It contains identified dynamics",
        "fields: center-of-mass position, mass, rotational inertia matrix, viscous",
        "damping, and dry Coulomb friction.",
        "",
        f"- Source parameters: `{params_path.name}`",
        f"- Fit RMSE: `{solver.get('fit_rmse_nm', 'unknown')} Nm`",
        f"- Regressor rank: `{solver.get('rank', 'unknown')} / {solver.get('columns', 'unknown')}`",
        f"- Projection epsilon: `{projection_epsilon}`",
        f"- Maximum accepted projected COM norm: `{max_com_norm} m`",
        f"- Raw non-PSD inertia groups: `{raw_nonphysical} / {len(representative)}`",
        f"- Projected groups written per URDF: `{projected}`",
        f"- Axis-aligned negligible-mass groups per URDF: `{axis_aligned}`",
        f"- Other nominal fallback groups per URDF: `{fallback}`",
        "",
        "## Outputs",
        "",
    ]
    lines.extend(f"- `{display_path(output)}`" for output in outputs)
    lines.extend(
        [
            "",
            "Only the gripper variants are exported because para_Labrefer was identified with",
            "the gripper model. Applying its distal inertia to the O6 hand variants would",
            "describe a different mechanism.",
            "",
            "## Representative inertial export",
            "",
            "All four outputs share the same moving-link inertia and drive values.",
            "",
            "| identified body | target link | action | raw mass (kg) | output mass (kg) | output COM norm (m) | raw pseudo min eig | output inertia min eig |",
            "|---|---|---|---:|---:|---:|---:|---:|",
        ]
    )
    for row in representative:
        lines.append(
            f"| {row.body} | {row.target_link} | {row.action} | {row.raw_mass:.8g} | "
            f"{row.output_mass:.8g} | {np.linalg.norm(row.output_com):.8g} | "
            f"{row.raw_pseudo_min_eigenvalue:.8g} | {row.output_inertia_min_eigenvalue:.8g} |"
        )
    lines.extend(
        [
            "",
            "The shoulder-pitch links are nearly unobservable in this identification because",
            "their physical mass distribution lies predominantly along the pitch rotation",
            "axes. The JSON `mx`, `my`, and `mz` values are first moments, not COM",
            "coordinates. Dividing them by the near-zero identified mass is ill-conditioned;",
            "the approximately 0.92 m values produced by unconstrained PSD projection are not",
            "physical COM estimates. The exporter therefore retains the small nominal",
            "mass/inertia needed for simulator stability and projects each nominal inertial",
            "origin onto its joint axis.",
            "",
            "| identified body | output COM xyz (m) | output first moment xyz (kg m) | output inertia diagonal (kg m^2) |",
            "|---|---:|---:|---:|",
        ]
    )
    for row in representative:
        if row.action != "keep_nominal_axis_aligned_regularized":
            continue
        output_com = np.asarray(row.output_com)
        first_moment = row.output_mass * output_com
        diagonal = " ".join([f"{row.output_inertia_min_eigenvalue:.12g}"] * 3)
        lines.append(
            f"| {row.body} | `{vector(output_com)}` | `{vector(first_moment)}` | "
            f"`{diagonal}` |"
        )
    lines.extend(
        [
            "",
            "## Drive export",
            "",
            "| target joint | damping | raw Coulomb | URDF friction | effort offset |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for identified_joint, terms in sorted(drive.items()):
        sign = TARGET_COORDINATE_SIGN.get(identified_joint, 1.0)
        raw_coulomb = float(terms["coulomb"])
        lines.append(
            f"| {target_joint_name(identified_joint)} | {float(terms['viscous']):.8g} | "
            f"{raw_coulomb:.8g} | {max(0.0, raw_coulomb):.8g} | "
            f"{sign * float(terms['offset']):.8g} |"
        )
    lines.extend(
        [
            "",
            "`damping` and non-negative Coulomb friction are written to standard URDF",
            "`<dynamics>` fields. Constant effort offsets are reported in the table",
            "above for reference only; standard URDF has no equivalent field, and an",
            "offset may include torque-sensor bias rather than passive mechanical",
            "friction.",
            "",
            "## Required validation",
            "",
            "Before selecting a generated URDF as the runtime default, rerun torque",
            "prediction on the held-out trajectories using the projected URDF parameters.",
            "The existing validation metrics apply to the raw unconstrained parameter",
            "vector, not to this PSD-projected candidate.",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def symmetrize(matrix: np.ndarray) -> np.ndarray:
    return 0.5 * (matrix + matrix.T)


def number(value: float) -> str:
    return f"{float(value):.12g}"


def vector(values: np.ndarray) -> str:
    return " ".join(number(float(value)) for value in values)


if __name__ == "__main__":
    main()
