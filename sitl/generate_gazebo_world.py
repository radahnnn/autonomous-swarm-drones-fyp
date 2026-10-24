"""Generate a 6–10 vehicle Gazebo world without duplicating checked-in models."""

import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

from sitl.formation_library import slots_for


REPO_ROOT = Path(__file__).resolve().parents[1]
TEMPLATE_WORLD = REPO_ROOT / "simulator/gazebo/worlds/cinewhoop_5drones.sdf"
TEMPLATE_MODEL = REPO_ROOT / "simulator/gazebo/models/cinewhoop_1"


def _replace_one(pattern: str, replacement: str, content: str, description: str) -> str:
    updated, count = re.subn(pattern, replacement, content, count=1)
    if count != 1:
        raise ValueError(f"template must contain exactly one {description}")
    return updated


def generate_gazebo_world(count: int, output_dir: Path) -> Path:
    """Create a matching world and per-instance ArduPilot FDM model configs."""
    if not 6 <= count <= 10:
        raise ValueError("generated Gazebo worlds support 6–10 drones; use checked-in worlds for 3 or 5")
    output_dir = Path(output_dir).resolve()
    models_dir = output_dir / "models"
    worlds_dir = output_dir / "worlds"
    worlds_dir.mkdir(parents=True, exist_ok=True)
    template_sdf = (TEMPLATE_MODEL / "model.sdf").read_text(encoding="utf-8")
    template_config = (TEMPLATE_MODEL / "model.config").read_text(encoding="utf-8")

    for index in range(5, count):
        drone_number = index + 1
        port_in = 9002 + 10 * index
        port_out = port_in + 1
        model_name = f"cinewhoop_{drone_number}"
        model_dir = models_dir / model_name
        model_dir.mkdir(parents=True, exist_ok=True)

        sdf = _replace_one(r'(<model\s+name=")[^"]+(">)',
                           rf"\g<1>{model_name}\g<2>", template_sdf, "root model name")
        sdf = _replace_one(r"(<fdm_port_in>\s*)\d+(\s*</fdm_port_in>)",
                           rf"\g<1>{port_in}\g<2>", sdf, "FDM input port")
        sdf = _replace_one(r"(<fdm_port_out>\s*)\d+(\s*</fdm_port_out>)",
                           rf"\g<1>{port_out}\g<2>", sdf, "FDM output port")
        (model_dir / "model.sdf").write_text(sdf, encoding="utf-8")

        config = template_config.replace("cinewhoop_1", model_name)
        config = config.replace("port 9002", f"port {port_in}")
        (model_dir / "model.config").write_text(config, encoding="utf-8")

    tree = ET.parse(TEMPLATE_WORLD)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("the Gazebo template does not contain a world element")
    world.set("name", f"cinewhoop_{count}drones")
    for include in list(world.findall("include")):
        model_name = include.findtext("name", "")
        uri = include.findtext("uri", "")
        if model_name.startswith("cinewhoop_") or uri.startswith("model://cinewhoop_"):
            world.remove(include)

    for index, slot in enumerate(slots_for("v", count)):
        north, east = float(slot[0]), float(slot[1])
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "name").text = f"cinewhoop_{index + 1}"
        ET.SubElement(include, "uri").text = f"model://cinewhoop_{index + 1}"
        ET.SubElement(include, "pose", {"degrees": "true"}).text = (
            f"{north:.3f} {east:.3f} 0.2 0 0 90"
        )

    world_path = worlds_dir / f"cinewhoop_{count}drones.sdf"
    tree.write(world_path, encoding="utf-8", xml_declaration=True)
    return world_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drones", type=int, choices=range(6, 11), default=10)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    print(generate_gazebo_world(args.drones, args.output_dir))


if __name__ == "__main__":
    main()
