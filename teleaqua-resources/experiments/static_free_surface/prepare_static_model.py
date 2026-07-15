#!/usr/bin/env python3
"""Create a static Teleh4z SDF with both air-propeller arms deployed."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


TARGET_TOTAL_MASS_KG = 14.608978170 / 9.81
TARGET_FULL_BUOYANCY_N = 15.377871758
ARM_MASS_KG = 0.03686
MIN_LINK_MASS_KG = 0.0001


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    tree = ET.parse(args.input)
    model = tree.getroot().find("model")
    if model is None:
        raise ValueError(f"No <model> in {args.input}")
    static = model.find("static")
    if static is None:
        static = ET.SubElement(model, "static")
    static.text = "true"
    model.set("name", "teleh4z_zaxis_static_deployed")
    remove_names = {
        "z_axis_carriage_link",
        "z_axis_slider_joint",
        "z_axis_attitude_joint",
    }
    for child in list(model):
        if child.get("name") in remove_names:
            model.remove(child)

    deployed_poses = {
        "arm_left_link": "0.0219 0.0445 0.0705 1.57079632679 0 0",
        "arm_right_link": "0.0219 -0.0445 0.0705 -1.57079632679 0 0",
    }
    for link_name, pose_text in deployed_poses.items():
        link = model.find(f"./link[@name='{link_name}']")
        if link is None:
            raise ValueError(f"Missing link {link_name}")
        pose = link.find("pose")
        if pose is None:
            pose = ET.SubElement(link, "pose")
        pose.set("relative_to", "base_link")
        pose.text = pose_text

    for joint_name in ("arm_left_joint", "arm_right_joint"):
        joint = model.find(f"./joint[@name='{joint_name}']")
        if joint is None:
            raise ValueError(f"Missing joint {joint_name}")
        joint.set("type", "fixed")
        axis = joint.find("axis")
        if axis is not None:
            joint.remove(axis)

    for plugin in list(model.findall("plugin")):
        if plugin.get("name") == "gz::sim::systems::JointTrajectoryController":
            model.remove(plugin)

    links = model.findall("link")
    base = model.find("./link[@name='base_link']")
    if base is None:
        raise ValueError("Missing base_link")
    non_base_non_arm = [
        link for link in links
        if link.get("name") not in {"base_link", "arm_left_link", "arm_right_link"}
    ]
    base_mass = (
        TARGET_TOTAL_MASS_KG
        - 2.0 * ARM_MASS_KG
        - len(non_base_non_arm) * MIN_LINK_MASS_KG
    )
    mass_by_link = {
        "base_link": base_mass,
        "arm_left_link": ARM_MASS_KG,
        "arm_right_link": ARM_MASS_KG,
    }
    for link in links:
        mass = link.find("./inertial/mass")
        if mass is None:
            continue
        mass.text = f"{mass_by_link.get(link.get('name'), MIN_LINK_MASS_KG):.12g}"

    hydrodynamics = next(
        (
            plugin.find("hydrodynamics")
            for plugin in model.findall("plugin")
            if plugin.find("hydrodynamics") is not None
        ),
        None,
    )
    if hydrodynamics is None:
        raise ValueError("Missing HybridHydrodynamics configuration")
    buoyancy = hydrodynamics.find("buoyancy_force_N")
    if buoyancy is None:
        buoyancy = ET.SubElement(hydrodynamics, "buoyancy_force_N")
    buoyancy.text = f"{TARGET_FULL_BUOYANCY_N:.12g}"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
