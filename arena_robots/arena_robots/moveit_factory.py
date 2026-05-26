"""Build a MoveIt config dict for a robot, shared between move_group launch
and the RViz parameter-injection path."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import arena_robots.Robot


def _prefix_urdf_links(urdf_xml: str, tf_prefix: str) -> str:
    """Prepend ``tf_prefix`` to every link reference in ``urdf_xml``.

    Joint names stay untouched, so the controller / joint_states wiring
    keeps working; only link/frame names get namespaced to match the TF
    tree published by robot_state_publisher's own ``frame_prefix``.
    """
    if not tf_prefix:
        return urdf_xml
    root = ET.fromstring(urdf_xml)
    for link in root.iter("link"):
        name = link.get("name")
        if name:
            link.set("name", tf_prefix + name)
    for joint in root.iter("joint"):
        for endpoint in joint:
            if endpoint.tag in ("parent", "child"):
                link_attr = endpoint.get("link")
                if link_attr:
                    endpoint.set("link", tf_prefix + link_attr)
    for gz in root.iter("gazebo"):
        ref = gz.get("reference")
        if ref:
            gz.set("reference", tf_prefix + ref)
    return ET.tostring(root, encoding="unicode")


def _prefix_srdf_links(srdf_xml: str, tf_prefix: str) -> str:
    """Same prefixing rules as the URDF, applied to the SRDF link refs."""
    if not tf_prefix:
        return srdf_xml
    root = ET.fromstring(srdf_xml)
    for group in root.iter("group"):
        for child in group:
            if child.tag == "chain":
                for attr in ("base_link", "tip_link"):
                    v = child.get(attr)
                    if v:
                        child.set(attr, tf_prefix + v)
            elif child.tag == "link":
                v = child.get("name")
                if v:
                    child.set("name", tf_prefix + v)
    for dc in root.iter("disable_collisions"):
        for attr in ("link1", "link2"):
            v = dc.get(attr)
            if v:
                dc.set(attr, tf_prefix + v)
    for vj in root.iter("virtual_joint"):
        v = vj.get("child_link")
        if v:
            vj.set("child_link", tf_prefix + v)
    return ET.tostring(root, encoding="unicode")


def build_moveit_params(robot_name: str, tf_prefix: str = "") -> dict[str, object] | None:
    """Return a flat parameter dict (URDF, SRDF, kinematics, joint limits)
    for ``robot_name`` if it advertises ``arm``; ``None`` otherwise.

    When ``tf_prefix`` is non-empty (e.g. ``"env_0/ridgeback_plus/"``) every
    link reference in the URDF/SRDF gets the prefix prepended, so MoveIt's
    planning-scene monitor looks up the same frame ids that
    robot_state_publisher publishes.
    """
    robot = arena_robots.Robot.RobotIdentifier(robot_name).resolve_sync()
    if "arm" not in robot.caps.available:
        return None

    arms = robot.caps.arm
    if arms is None:
        raise ValueError(f"{robot_name}: arm cap required but absent")
    if len(arms) != 1:
        raise NotImplementedError(f"{robot_name}: multi-arm not yet supported")
    (arm,) = arms.values()

    mv = arm.raw.get("moveit") or {}
    pkg = mv.get("package")
    if not pkg:
        return None

    args_dict = mv.get("args") or {}
    mappings = {k: (str(v).lower() if isinstance(v, bool) else str(v)) for k, v in args_dict.items()}
    mappings.setdefault("name", mappings.get("ur_type", "ur5e"))

    urdf_abs = Path(get_package_share_directory("arena_robots")) / "robots" / robot_name / "urdf" / f"{robot_name}.urdf.xacro"

    srdf_ref = mv.get("srdf") or {}
    srdf_pkg = srdf_ref.get("package", pkg)
    srdf_rel = srdf_ref.get("path", "srdf/ur.srdf.xacro")
    srdf_abs = Path(get_package_share_directory(srdf_pkg)) / srdf_rel

    jl_ref = mv.get("joint_limits") or {}
    jl_pkg = jl_ref.get("package", pkg)
    jl_rel = jl_ref.get("path", "config/joint_limits.yaml")
    jl_abs = Path(get_package_share_directory(jl_pkg)) / jl_rel

    moveit_config = MoveItConfigsBuilder(robot_name="ur", package_name=pkg).robot_description(file_path=str(urdf_abs), mappings=mappings).robot_description_semantic(srdf_abs, mappings=mappings).joint_limits(jl_abs).to_moveit_configs()
    params = moveit_config.to_dict()

    if tf_prefix:
        rd = params.get("robot_description")
        if isinstance(rd, str):
            params["robot_description"] = _prefix_urdf_links(rd, tf_prefix)
        rds = params.get("robot_description_semantic")
        if isinstance(rds, str):
            params["robot_description_semantic"] = _prefix_srdf_links(rds, tf_prefix)
    return params
