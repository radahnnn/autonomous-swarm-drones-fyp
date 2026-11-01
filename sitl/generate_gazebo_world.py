"""Generate a 6–10 vehicle Gazebo world without duplicating checked-in models."""

import argparse
import math
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


OBSTACLE_HEIGHT_M = 14.0     # tall enough to reach above every flight altitude used in the demos
CAMERA_SIZE = (320, 240)
CAMERA_HZ = 5

_CAMERA_LINK = """
    <!-- downward-looking camera (added by sitl/generate_gazebo_world.py) -->
    <link name="camera_link">
      <pose relative_to="iris_with_standoffs::base_link">0 0 -0.06 0 1.5708 0</pose>
      <inertial>
        <mass>0.005</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="camera" type="camera">
        <always_on>1</always_on>
        <update_rate>%(hz)d</update_rate>
        <topic>%(topic)s</topic>
        <camera>
          <horizontal_fov>1.3</horizontal_fov>
          <image><width>%(w)d</width><height>%(h)d</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>120</far></clip>
        </camera>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>camera_link</child>
    </joint>

"""


def camera_topic(drone_number: int) -> str:
    """Gazebo transport topic of drone ``drone_number``'s (1-based) camera when the world was generated with cameras."""
    return f"/drone{drone_number}/camera"


_MARKER_COLOURS = ("0.15 0.45 0.95 1", "0.95 0.85 0.1 1", "0.7 0.2 0.85 1", "0.1 0.8 0.75 1")


def _visual_box(name: str, x: float, y: float, z: float, size, colour: str) -> ET.Element:
    """A static, collision-free coloured box (camera scenery only; it cannot change the flight physics)."""
    model = ET.Element("model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{x:.3f} {y:.3f} {z:.3f} 0 0 0"
    visual = ET.SubElement(ET.SubElement(model, "link", {"name": "link"}), "visual", {"name": "visual"})
    ET.SubElement(ET.SubElement(ET.SubElement(visual, "geometry"), "box"), "size").text = " ".join(f"{v:g}" for v in size)
    material = ET.SubElement(visual, "material")
    for tag in ("ambient", "diffuse"):
        ET.SubElement(material, tag).text = colour
    return model


def _ground_visual() -> ET.Element:
    """A visible ground (plane, visual only: the flight physics keeps using the world's own ground)."""
    model = ET.Element("model", {"name": "ground_visual"})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "0 0 0.005 0 0 0"   # just above the world's own ground
    visual = ET.SubElement(ET.SubElement(model, "link", {"name": "link"}), "visual", {"name": "visual"})
    plane = ET.SubElement(ET.SubElement(visual, "geometry"), "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "400 400"
    material = ET.SubElement(visual, "material")
    for tag in ("ambient", "diffuse"):
        ET.SubElement(material, tag).text = "0.32 0.46 0.28 1"
    return model


def _tree_model(index: int, north: float, east: float) -> ET.Element:
    """Scenery tree (trunk + canopy). Visual only: no collision, so it does not affect flight or the sensors."""
    model = ET.Element("model", {"name": f"tree_{index + 1}"})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{east:.3f} {north:.3f} 0 0 0 0"
    link = ET.SubElement(model, "link", {"name": "link"})
    for name, shape, size_tag, size, z, colour in (
            ("trunk", "cylinder", None, None, 1.25, "0.4 0.26 0.13 1"),
            ("canopy", "sphere", "radius", 1.7, 3.6, "0.12 0.42 0.16 1")):
        visual = ET.SubElement(link, "visual", {"name": name})
        ET.SubElement(visual, "pose").text = f"0 0 {z} 0 0 0"
        geometry = ET.SubElement(ET.SubElement(visual, "geometry"), shape)
        if shape == "cylinder":
            ET.SubElement(geometry, "radius").text = "0.25"
            ET.SubElement(geometry, "length").text = "2.5"
        else:
            ET.SubElement(geometry, size_tag).text = f"{size:g}"
        material = ET.SubElement(visual, "material")
        for tag in ("ambient", "diffuse"):
            ET.SubElement(material, tag).text = colour
    return model


def _obstacle_model(index: int, north: float, east: float, radius: float) -> ET.Element:
    model = ET.Element("model", {"name": f"obstacle_{index + 1}"})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = f"{east:.3f} {north:.3f} {OBSTACLE_HEIGHT_M / 2:.3f} 0 0 0"   # x = East, y = North
    link = ET.SubElement(model, "link", {"name": "link"})
    for kind in ("visual", "collision"):
        part = ET.SubElement(link, kind, {"name": kind})
        cylinder = ET.SubElement(ET.SubElement(part, "geometry"), "cylinder")
        ET.SubElement(cylinder, "radius").text = f"{radius:g}"
        ET.SubElement(cylinder, "length").text = f"{OBSTACLE_HEIGHT_M:g}"
        if kind == "visual":
            material = ET.SubElement(part, "material")
            for tag in ("ambient", "diffuse"):
                ET.SubElement(material, tag).text = "0.95 0.25 0.1 1"
    return model


def generate_gazebo_world(count: int, output_dir: Path, step_ms: float = 1.0, camera: bool = False,
                          obstacles=(), markers=(), trees=()) -> Path:
    """Create a matching world and per-instance ArduPilot FDM model configs."""
    if not 6 <= count <= 10:
        raise ValueError("generated Gazebo worlds support 6–10 drones; use checked-in worlds for 3 or 5")
    output_dir = Path(output_dir).resolve()
    models_dir = output_dir / "models"
    worlds_dir = output_dir / "worlds"
    worlds_dir.mkdir(parents=True, exist_ok=True)
    template_sdf = (TEMPLATE_MODEL / "model.sdf").read_text(encoding="utf-8")
    template_config = (TEMPLATE_MODEL / "model.config").read_text(encoding="utf-8")

    # With cameras every model is regenerated (the checked-in ones have none); the generated copies shadow them.
    for index in range(0 if camera else 5, count):
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
        if camera:
            marker = '    <plugin name="ArduPilotPlugin"'
            if sdf.count(marker) != 1:
                raise ValueError("template must contain exactly one ArduPilot plugin")
            sdf = sdf.replace(marker, _CAMERA_LINK % {"hz": CAMERA_HZ, "topic": camera_topic(drone_number),
                                                      "w": CAMERA_SIZE[0], "h": CAMERA_SIZE[1]} + marker)
        (model_dir / "model.sdf").write_text(sdf, encoding="utf-8")

        config = template_config.replace("cinewhoop_1", model_name)
        config = config.replace("port 9002", f"port {port_in}")
        (model_dir / "model.config").write_text(config, encoding="utf-8")

    tree = ET.parse(TEMPLATE_WORLD)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("the Gazebo template does not contain a world element")
    world.set("name", f"cinewhoop_{count}drones")
    if not 0.5 <= step_ms <= 4.0:
        raise ValueError("physics step must be between 0.5 and 4 ms (ArduCopter's main loop runs at 400 Hz = 2.5 ms)")
    step = world.find("physics/max_step_size")
    if step is None:
        raise ValueError("the Gazebo template does not set a physics step")
    step.text = f"{step_ms / 1000.0:g}"
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
            f"{east:.3f} {north:.3f} 0.2 0 0 90"   # Gazebo world: x = East, y = North
        )

    for index, (north, east, radius) in enumerate(obstacles):
        if not all(math.isfinite(v) for v in (north, east, radius)) or radius <= 0:
            raise ValueError("obstacles must be finite north,east,radius with a positive radius")
        world.append(_obstacle_model(index, north, east, radius))

    if camera:      # something to look at: a visible ground and flat markers at points of interest
        world.append(_ground_visual())
        for index, (north, east) in enumerate(trees):
            if not all(math.isfinite(v) for v in (north, east)):
                raise ValueError("trees must be finite north,east pairs")
            world.append(_tree_model(index, north, east))
        for index, (north, east) in enumerate(markers):
            if not all(math.isfinite(v) for v in (north, east)):
                raise ValueError("markers must be finite north,east pairs")
            world.append(_visual_box(f"marker_{index + 1}", east, north, 0.03, (3.0, 3.0, 0.04),
                                     _MARKER_COLOURS[index % len(_MARKER_COLOURS)]))

    world_path = worlds_dir / f"cinewhoop_{count}drones.sdf"
    tree.write(world_path, encoding="utf-8", xml_declaration=True)
    return world_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drones", type=int, choices=range(6, 11), default=10)
    parser.add_argument("--step-ms", type=float, default=1.0,
                        help="physics step in ms; every step is a lockstep exchange with every SITL, so larger is faster")
    parser.add_argument("--camera", action="store_true", help="add a downward camera to every drone")
    parser.add_argument("--obstacle", action="append", default=[], metavar="N,E,R",
                        help="add a pole at north,east with radius (m); repeatable")
    parser.add_argument("--marker", action="append", default=[], metavar="N,E",
                        help="flat coloured 3 m marker on the ground (with --camera); repeatable")
    parser.add_argument("--tree", action="append", default=[], metavar="N,E",
                        help="scenery tree (visual only, with --camera); repeatable")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    obstacles = []
    for raw in args.obstacle:
        try:
            values = tuple(float(v) for v in raw.split(","))
        except ValueError:
            parser.error("--obstacle must be north,east,radius")
        if len(values) != 3:
            parser.error("--obstacle must be north,east,radius")
        obstacles.append(values)
    markers = []
    for raw in args.marker:
        try:
            pair = tuple(float(v) for v in raw.split(","))
        except ValueError:
            parser.error("--marker must be north,east")
        if len(pair) != 2:
            parser.error("--marker must be north,east")
        markers.append(pair)
    trees = []
    for raw in args.tree:
        try:
            pair = tuple(float(v) for v in raw.split(","))
        except ValueError:
            parser.error("--tree must be north,east")
        if len(pair) != 2:
            parser.error("--tree must be north,east")
        trees.append(pair)
    print(generate_gazebo_world(args.drones, args.output_dir, args.step_ms, args.camera, obstacles, markers, trees))


if __name__ == "__main__":
    main()
