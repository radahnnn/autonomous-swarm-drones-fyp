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
        assert float(pose[0]) == pytest.approx(slots[index, 1])      # Gazebo x = East
        assert float(pose[1]) == pytest.approx(slots[index, 0])      # Gazebo y = North
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


def test_generated_spawn_poses_match_the_checked_in_five_drone_world(tmp_path):
    """The checked-in world is the reference (live-flown); the generator must lay the first five drones out
    identically, otherwise the V is mirrored and the swarm has to cross itself after take-off."""
    import xml.etree.ElementTree as ET
    from sitl.generate_gazebo_world import TEMPLATE_WORLD, generate_gazebo_world

    def poses(path):
        world = ET.parse(path).getroot().find("world")
        return {inc.findtext("name"): [float(v) for v in inc.findtext("pose").split()[:2]]
                for inc in world.findall("include") if inc.findtext("name", "").startswith("cinewhoop_")}

    reference, generated = poses(TEMPLATE_WORLD), poses(generate_gazebo_world(6, tmp_path))
    for name, xy in reference.items():
        assert generated[name] == pytest.approx(xy), name


def test_physics_step_is_configurable_and_bounded(tmp_path):
    import xml.etree.ElementTree as ET
    from sitl.generate_gazebo_world import generate_gazebo_world
    default = ET.parse(generate_gazebo_world(6, tmp_path / "a")).getroot().find("world/physics/max_step_size").text
    fast = ET.parse(generate_gazebo_world(6, tmp_path / "b", step_ms=1.25)).getroot().find("world/physics/max_step_size").text
    assert float(default) == pytest.approx(0.001) and float(fast) == pytest.approx(0.00125)
    for bad in (0.0, 0.1, 1.5, 2.5, float("nan")):
        with pytest.raises(ValueError):
            generate_gazebo_world(6, tmp_path / "c", step_ms=bad)


def test_camera_models_and_obstacles(tmp_path):
    import xml.etree.ElementTree as ET
    from sitl.generate_gazebo_world import camera_topic, generate_gazebo_world
    world = generate_gazebo_world(6, tmp_path, camera=True, obstacles=[(12.0, -4.0, 0.6), (0.0, 9.0, 0.4)])
    for number in range(1, 7):
        sdf = (tmp_path / "models" / f"cinewhoop_{number}" / "model.sdf").read_text()
        assert sdf.count('type="camera"') == 1 and camera_topic(number) in sdf
        assert f"<fdm_port_in>{9002 + 10 * (number - 1)}</fdm_port_in>" in sdf        # ports stay per-drone
        ET.fromstring(sdf)                                                          # still well-formed XML
    root = ET.parse(world).getroot().find("world")
    poles = {m.get("name"): m for m in root.findall("model") if m.get("name", "").startswith("obstacle_")}
    assert set(poles) == {"obstacle_1", "obstacle_2"}
    x, y, z = (float(v) for v in poles["obstacle_1"].findtext("pose").split()[:3])
    assert (x, y) == pytest.approx((-4.0, 12.0)) and z > 1.0                          # x = East, y = North
    assert float(poles["obstacle_1"].find("link/visual/geometry/cylinder/radius").text) == pytest.approx(0.6)


def test_no_camera_by_default_and_bad_obstacle_rejected(tmp_path):
    from sitl.generate_gazebo_world import generate_gazebo_world
    generate_gazebo_world(6, tmp_path / "a")
    assert not (tmp_path / "a" / "models" / "cinewhoop_1").exists()                    # checked-in models are used
    for bad in ((1.0, 2.0, 0.0), (float("nan"), 0.0, 1.0)):
        with pytest.raises(ValueError):
            generate_gazebo_world(6, tmp_path / "b", obstacles=[bad])


def test_visible_ground_and_markers_have_no_collision(tmp_path):
    import xml.etree.ElementTree as ET
    from sitl.generate_gazebo_world import generate_gazebo_world
    world = ET.parse(generate_gazebo_world(6, tmp_path, camera=True, markers=[(10.0, 0.0), (-5.0, 8.0)])).getroot().find("world")
    scenery = {m.get("name"): m for m in world.findall("model") if m.get("name") in ("ground_visual", "marker_1", "marker_2")}
    assert set(scenery) == {"ground_visual", "marker_1", "marker_2"}
    for model in scenery.values():
        assert model.find("link/collision") is None and model.findtext("static") == "true"
    x, y = (float(v) for v in scenery["marker_1"].findtext("pose").split()[:2])
    assert (x, y) == pytest.approx((0.0, 10.0))
    with pytest.raises(ValueError):
        generate_gazebo_world(6, tmp_path / "bad", camera=True, markers=[(float("inf"), 0.0)])


def test_trees_are_scenery_only(tmp_path):
    import xml.etree.ElementTree as ET
    from sitl.generate_gazebo_world import generate_gazebo_world
    world = ET.parse(generate_gazebo_world(6, tmp_path, camera=True, trees=[(8.0, 14.0), (-12.0, -9.0)])).getroot().find("world")
    trees = [m for m in world.findall("model") if m.get("name", "").startswith("tree_")]
    assert len(trees) == 2
    for tree in trees:
        assert tree.find("link/collision") is None and tree.findtext("static") == "true"
        assert {v.get("name") for v in tree.findall("link/visual")} == {"trunk", "canopy"}
    x, y = (float(v) for v in trees[0].findtext("pose").split()[:2])
    assert (x, y) == pytest.approx((14.0, 8.0))
    with pytest.raises(ValueError):
        generate_gazebo_world(6, tmp_path / "bad", camera=True, trees=[(float("nan"), 0.0)])
