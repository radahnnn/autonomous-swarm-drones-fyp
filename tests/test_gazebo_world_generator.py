import re
import xml.etree.ElementTree as ET

import pytest

from sitl.formation_library import slots_for
from sitl.generate_gazebo_world import generate_gazebo_world


def test_generates_ten_drone_world_with_unique_fdm_ports_and_v_slots(tmp_path):
    world_path = generate_gazebo_world(10, tmp_path)
    world = ET.parse(world_path).getroot().find("world")
    assert world is not None
    assert world.get("name") == "cinewhoop_10drones"

    includes = [node for node in world.findall("include")
                if node.findtext("uri", "").startswith("model://cinewhoop_")]
    assert len(includes) == 10
    slots = slots_for("v", 10)
    for index, include in enumerate(includes):
        number = index + 1
        assert include.findtext("name") == f"cinewhoop_{number}"
        pose = include.findtext("pose").split()
        assert float(pose[0]) == pytest.approx(slots[index, 0])
        assert float(pose[1]) == pytest.approx(slots[index, 1])
        assert float(pose[2]) == pytest.approx(0.2)

    ports = []
    for number in range(1, 11):
        model = tmp_path / "models" / f"cinewhoop_{number}" / "model.sdf"
        if number <= 5:
            from sitl.generate_gazebo_world import REPO_ROOT
            model = REPO_ROOT / "simulator/gazebo/models" / f"cinewhoop_{number}" / "model.sdf"
        text = model.read_text(encoding="utf-8")
        match = re.search(r"<fdm_port_in>\s*(\d+)\s*</fdm_port_in>", text)
        assert match is not None
        ports.append(int(match.group(1)))
    assert ports == [9002 + 10 * index for index in range(10)]


def test_generated_gazebo_world_rejects_sizes_outside_six_to_ten(tmp_path):
    with pytest.raises(ValueError, match="6–10"):
        generate_gazebo_world(5, tmp_path)
    with pytest.raises(ValueError, match="6–10"):
        generate_gazebo_world(11, tmp_path)
