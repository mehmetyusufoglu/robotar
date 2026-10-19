#!/usr/bin/env python3
"""Generate the Robotar orchard Gazebo world and ros_gz bridge configuration.

``config/orchard.yaml`` is the single source of truth for the orchard: row
geometry, tree poses, the target branch, olives, catch net, physics tuning
and harvest topic patterns. This script derives everything else from it and
writes two build artifacts:

* an SDFormat 1.11 world (Gazebo Harmonic) with two tree rows, one
  instrumented target branch, six dynamically detachable olives, a catch
  net, ground and lighting;
* a ros_gz_bridge YAML configuration covering /clock, the robot sensor
  topics fixed by the interface contract, and one detach/state topic pair
  per target olive.

Both outputs are generated files and must never be hand-edited. The script
only uses the Python standard library; if PyYAML happens to be available it
is used to parse the configuration, otherwise a built-in parser for the
restricted YAML subset used by orchard.yaml takes over.
"""

import argparse
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

SDF_VERSION = "1.11"
GENERATED_NOTICE = (
    "GENERATED FILE - do not edit. Rebuild from config/orchard.yaml "
    "with tools/generate_orchard.py."
)

# Robot sensor and clock topics fixed by the project interface contract
# (see robotar_description/config/physical.yaml for the same values).
STATIC_BRIDGES = [
    ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
    ("/camera/mast/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"),
    ("/camera/mast/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "GZ_TO_ROS"),
    ("/camera/mast/depth/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"),
    ("/camera/mast/depth/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo", "GZ_TO_ROS"),
    ("/imu/data", "sensor_msgs/msg/Imu", "gz.msgs.IMU", "GZ_TO_ROS"),
    ("/comb/contact", "ros_gz_interfaces/msg/Contacts", "gz.msgs.Contacts", "GZ_TO_ROS"),
]

WORLD_PLUGINS = [
    ("gz-sim-physics-system", "gz::sim::systems::Physics", {}),
    ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands", {}),
    ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster", {}),
    ("gz-sim-sensors-system", "gz::sim::systems::Sensors", {"render_engine": "ogre2"}),
    ("gz-sim-imu-system", "gz::sim::systems::Imu", {}),
    ("gz-sim-contact-system", "gz::sim::systems::Contact", {}),
]

TRUNK_COLOR = (0.36, 0.27, 0.18)
FOLIAGE_BASE_COLOR = (0.33, 0.39, 0.24)
OLIVE_GREEN = (0.30, 0.36, 0.12)
OLIVE_PURPLE = (0.26, 0.13, 0.26)
SOIL_COLOR = (0.45, 0.36, 0.25)
NET_COLOR = (0.12, 0.25, 0.15)
BRANCH_COLOR = (0.40, 0.31, 0.20)


# ---------------------------------------------------------------------------
# Configuration loading
# ---------------------------------------------------------------------------

def _parse_scalar(text):
    text = text.strip()
    if len(text) >= 2 and text[0] == text[-1] and text[0] in "\"'":
        return text[1:-1]
    lowered = text.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    if lowered in ("null", "~"):
        return None
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text


def _mini_yaml_load(text):
    """Parse the restricted YAML subset used by orchard.yaml.

    Supports nested block mappings by indentation, block sequences
    (``- item``), flow sequences (``[a, b]``) and plain scalars. It exists so
    the generator stays standard-library only; PyYAML is preferred when
    installed.
    """

    lines = []
    for raw in text.splitlines():
        if not raw.strip() or raw.strip().startswith("#"):
            continue
        content = raw.split(" #", 1)[0].rstrip()
        indent = len(raw) - len(raw.lstrip(" "))
        lines.append((indent, content.strip()))

    pos = [0]

    def parse_value(token):
        token = token.strip()
        if token.startswith("["):
            if not token.endswith("]"):
                raise ValueError(f"unterminated flow sequence: {token!r}")
            inner = token[1:-1].strip()
            return [_parse_scalar(part) for part in inner.split(",")] if inner else []
        return _parse_scalar(token)

    def parse_block(indent):
        if pos[0] >= len(lines):
            return None
        first_indent, first_content = lines[pos[0]]
        if first_indent < indent:
            return None
        if first_content == "-" or first_content.startswith("- "):
            result = []
            while pos[0] < len(lines):
                line_indent, content = lines[pos[0]]
                if line_indent != indent or not (
                    content == "-" or content.startswith("- ")
                ):
                    break
                item = content[1:].strip()
                pos[0] += 1
                if item:
                    result.append(parse_value(item))
                else:
                    result.append(parse_block(indent + 2))
            return result
        result = {}
        while pos[0] < len(lines):
            line_indent, content = lines[pos[0]]
            if line_indent != indent or content == "-" or content.startswith("- "):
                break
            key, sep, value = content.partition(":")
            if not sep:
                raise ValueError(f"invalid mapping line: {content!r}")
            key = key.strip()
            value = value.strip()
            pos[0] += 1
            if value:
                result[key] = parse_value(value)
            elif pos[0] < len(lines) and lines[pos[0]][0] > indent:
                result[key] = parse_block(lines[pos[0]][0])
            else:
                result[key] = None
        return result

    document = parse_block(lines[0][0] if lines else 0)
    if pos[0] != len(lines):
        raise ValueError("could not parse the complete configuration")
    return document


def load_config(path):
    text = Path(path).read_text(encoding="utf-8")
    try:
        import yaml
    except ImportError:
        return _mini_yaml_load(text)
    return yaml.safe_load(text)


# ---------------------------------------------------------------------------
# SDF helpers
# ---------------------------------------------------------------------------

def _n(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if value == int(value) and abs(value) < 1e12:
            return str(int(value))
        return f"{value:.9g}"
    return str(value)


def _vec(values):
    return " ".join(_n(v) for v in values)


def _sub(parent, tag, text=None, **attrs):
    element = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        element.text = text if isinstance(text, str) else _n(text)
    return element


def _pose_text(xyz, rpy=(0.0, 0.0, 0.0)):
    return _vec(tuple(xyz) + tuple(rpy))


def _add_geometry(parent, kind, **dims):
    geometry = _sub(parent, "geometry")
    shape = _sub(geometry, kind)
    for key, value in dims.items():
        _sub(shape, key, _n(value) if not isinstance(value, (list, tuple)) else _vec(value))
    return geometry


def _add_material(visual, color):
    material = _sub(visual, "material")
    ambient = tuple(c * 0.6 for c in color)
    _sub(material, "ambient", _vec(ambient) + " 1")
    _sub(material, "diffuse", _vec(color) + " 1")
    _sub(material, "specular", "0.05 0.05 0.05 1")


def _add_visual(link, name, pose, kind, color, **dims):
    visual = _sub(link, "visual", name=name)
    _sub(visual, "pose", pose)
    _add_geometry(visual, kind, **dims)
    _add_material(visual, color)
    return visual


def _add_collision(link, name, pose, kind, **dims):
    collision = _sub(link, "collision", name=name)
    _sub(collision, "pose", pose)
    _add_geometry(collision, kind, **dims)
    return collision


def _add_inertial(link, mass, ixx, iyy, izz):
    inertial = _sub(link, "inertial")
    _sub(inertial, "mass", _n(mass))
    inertia = _sub(inertial, "inertia")
    for tag, value in (("ixx", ixx), ("iyy", iyy), ("izz", izz)):
        _sub(inertia, tag, _n(value))
    for tag in ("ixy", "ixz", "iyz"):
        _sub(inertia, tag, "0")


# ---------------------------------------------------------------------------
# Orchard geometry derivation
# ---------------------------------------------------------------------------

def tree_grid(orchard):
    rows = orchard["rows"]
    poses = []
    for row in range(rows["count"]):
        y = rows["row_y"][row]
        for index in range(rows["trees_per_row"]):
            x = rows["start_x"] + index * rows["tree_spacing"]
            poses.append((row * rows["trees_per_row"] + index, x, y))
    return poses


def _canopy_blobs(rng, tree_cfg):
    half_w = tree_cfg["canopy_width"] / 2.0
    half_d = tree_cfg["canopy_depth"] / 2.0
    bottom = tree_cfg["canopy_bottom"]
    span = tree_cfg["height"] - bottom
    blobs = [(0.0, 0.0, bottom + 0.70 * span, 0.58 * half_w)]
    for k in range(5):
        angle = 2.0 * math.pi * k / 5.0 + rng.uniform(-0.25, 0.25)
        radial = rng.uniform(0.45, 0.62)
        blobs.append(
            (
                math.cos(angle) * radial * half_w,
                math.sin(angle) * radial * half_d,
                bottom + span * rng.uniform(0.38, 0.62),
                rng.uniform(0.34, 0.46) * half_w,
            )
        )
    blobs.append(
        (
            rng.uniform(-0.12, 0.12),
            rng.uniform(-0.12, 0.12),
            bottom + 0.93 * span,
            0.36 * half_w,
        )
    )
    return blobs


def _jittered_color(rng, base, spread=0.045):
    return tuple(min(1.0, max(0.0, c + rng.uniform(-spread, spread))) for c in base)


def branch_geometry(tree_xyz, branch_cfg):
    """Return (base_xyz, midpoint_xyz, yaw) of the target branch cylinder."""

    tip = branch_cfg["tip_xyz"]
    length = branch_cfg["length"]
    dx = tip[0] - tree_xyz[0]
    dy = tip[1] - tree_xyz[1]
    horizontal = math.hypot(dx, dy)
    if horizontal < 1e-6:
        raise ValueError("harvest branch tip must be offset from the trunk axis")
    ux, uy = dx / horizontal, dy / horizontal
    base = (tip[0] - ux * length, tip[1] - uy * length, tip[2])
    midpoint = tuple((a + b) / 2.0 for a, b in zip(base, tip))
    yaw = math.atan2(dy, dx)
    return base, midpoint, yaw


def _point_segment_distance(point, seg_a, seg_b):
    ax, ay, az = seg_a
    bx, by, bz = seg_b
    px, py, pz = point
    abx, aby, abz = bx - ax, by - ay, bz - az
    length_sq = abx * abx + aby * aby + abz * abz
    if length_sq < 1e-12:
        return math.dist(point, seg_a)
    t = ((px - ax) * abx + (py - ay) * aby + (pz - az) * abz) / length_sq
    t = min(1.0, max(0.0, t))
    closest = (ax + t * abx, ay + t * aby, az + t * abz)
    return math.dist(point, closest)


def validate_layout(orchard):
    """Fail the build early if configured bodies would overlap at spawn."""

    target = orchard["target_tree"]
    grid = {index: (x, y) for index, x, y in tree_grid(orchard)}
    matched = None
    for index, xy in grid.items():
        if f"olive_tree_{index}" == target["name"]:
            matched = (index, xy)
    if matched is None:
        raise ValueError(f"target tree {target['name']!r} is not on the derived grid")
    index, (tx, ty) = matched
    if abs(target["xyz"][0] - tx) > 1e-9 or abs(target["xyz"][1] - ty) > 1e-9:
        raise ValueError(
            f"target_tree.xyz {target['xyz']} contradicts the grid pose "
            f"({tx}, {ty}) of {target['name']}"
        )

    branch_cfg = target["harvest_branch"]
    base, _, _ = branch_geometry(target["xyz"], branch_cfg)
    tip = tuple(branch_cfg["tip_xyz"])
    branch_radius = branch_cfg["radius"]

    tree_cfg = orchard["tree"]
    span = tree_cfg["height"] - tree_cfg["canopy_bottom"]
    canopy_collision_bottom = tree_cfg["canopy_bottom"] + 0.37 * span

    olives_cfg = target["olives"]
    positions = [tuple(p) for p in olives_cfg["positions"]]
    if len(positions) != target["olive_count"]:
        raise ValueError("olive_count does not match the configured olive positions")
    olive_bound = max(olives_cfg["size_xyz"]) / 2.0

    for pos in positions:
        if pos[2] + olive_bound > canopy_collision_bottom:
            raise ValueError(
                f"olive at {pos} reaches into the canopy collision "
                f"(bottom z={canopy_collision_bottom:.3f})"
            )
        clearance = _point_segment_distance(pos, base, tip) - branch_radius - olive_bound
        if clearance < 0.01:
            raise ValueError(
                f"olive at {pos} overlaps the target branch "
                f"(clearance {clearance:.4f} m)"
            )
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            if math.dist(positions[i], positions[j]) < 2.0 * olive_bound + 0.005:
                raise ValueError(
                    f"olives {i} and {j} overlap at spawn "
                    f"({positions[i]} vs {positions[j]})"
                )


# ---------------------------------------------------------------------------
# World construction
# ---------------------------------------------------------------------------

def _build_ground(world, ground_cfg):
    model = _sub(world, "model", name="orchard_ground")
    _sub(model, "static", "true")
    link = _sub(model, "link", name="ground_link")
    size = (ground_cfg["size"][0], ground_cfg["size"][1], 0.2)
    pose = _pose_text((0.0, 0.0, -0.1))
    collision = _add_collision(link, "ground_collision", pose, "box", size=size)
    surface = _sub(collision, "surface")
    friction = _sub(surface, "friction")
    ode = _sub(friction, "ode")
    _sub(ode, "mu", "1.0")
    _sub(ode, "mu2", "1.0")
    _add_visual(link, "ground_visual", pose, "box", SOIL_COLOR, size=size)


def _build_tree(world, orchard, index, x, y, rng):
    tree_cfg = orchard["tree"]
    target = orchard["target_tree"]
    is_target = target["name"] == f"olive_tree_{index}"

    model = _sub(world, "model", name=f"olive_tree_{index}")
    _sub(model, "static", "true")
    _sub(model, "pose", _pose_text((x, y, 0.0)))

    trunk_radius = tree_cfg["trunk_diameter"] / 2.0
    bottom = tree_cfg["canopy_bottom"]
    span = tree_cfg["height"] - bottom

    trunk_link = _sub(model, "link", name="trunk_link")
    trunk_visual_length = bottom + 1.0
    _add_visual(
        trunk_link,
        "trunk_visual",
        _pose_text((0.0, 0.0, trunk_visual_length / 2.0)),
        "cylinder",
        TRUNK_COLOR,
        radius=trunk_radius,
        length=trunk_visual_length,
    )
    trunk_collision_length = bottom + 0.5
    _add_collision(
        trunk_link,
        "trunk_collision",
        _pose_text((0.0, 0.0, trunk_collision_length / 2.0)),
        "cylinder",
        radius=trunk_radius,
        length=trunk_collision_length,
    )

    # Coarse canopy collision: a single cylinder over the upper canopy, kept
    # clear of the instrumented branch and olives (checked in
    # validate_layout).
    canopy_collision_bottom = bottom + 0.37 * span
    canopy_collision_top = bottom + 0.87 * span
    _add_collision(
        trunk_link,
        "canopy_collision",
        _pose_text((0.0, 0.0, (canopy_collision_bottom + canopy_collision_top) / 2.0)),
        "cylinder",
        radius=tree_cfg["canopy_width"] / 2.0,
        length=canopy_collision_top - canopy_collision_bottom,
    )

    for blob_index, (bx, by, bz, radius) in enumerate(_canopy_blobs(rng, tree_cfg)):
        _add_visual(
            trunk_link,
            f"canopy_visual_{blob_index}",
            _pose_text((bx, by, bz)),
            "sphere",
            _jittered_color(rng, FOLIAGE_BASE_COLOR),
            radius=radius,
        )

    if not is_target:
        return

    branch_cfg = target["harvest_branch"]
    _, midpoint, yaw = branch_geometry(target["xyz"], branch_cfg)
    relative_mid = tuple(m - t for m, t in zip(midpoint, target["xyz"]))

    branch_link = _sub(model, "link", name=branch_cfg["name"])
    _add_visual(
        branch_link,
        f"{branch_cfg['name']}_visual",
        _pose_text(relative_mid, (0.0, -math.pi / 2.0, yaw)),
        "cylinder",
        BRANCH_COLOR,
        radius=branch_cfg["radius"],
        length=branch_cfg["length"],
    )
    # The branch collision is a box (square cross-section circumscribing the
    # visual cylinder): DART/FCL produces reliable contact points for the
    # comb-bank-cylinder vs box pair, while grazing cylinder-cylinder
    # contacts publish no contact points (verified with an isolated test
    # world). The contact sensor and the mission gating depend on this.
    _add_collision(
        branch_link,
        branch_cfg["collision"],
        _pose_text(relative_mid, (0.0, 0.0, yaw)),
        "box",
        size=(branch_cfg["length"], 2.0 * branch_cfg["radius"], 2.0 * branch_cfg["radius"]),
    )

    joint = _sub(model, "joint", name=f"{branch_cfg['name']}_fixed", type="fixed")
    _sub(joint, "parent", "trunk_link")
    _sub(joint, "child", branch_cfg["name"])

    # Contact sensor on the instrumented branch. The tool-side sensors on the
    # comb banks (robotar_description) monitor the same contact pair, but for
    # the URDF-spawned robot DART does not publish contact points for the
    # grazing bank/branch contact, while the static-tree side is reported
    # reliably (verified against isolated test worlds). Both publish to the
    # contract topic physical.yaml:sensors.comb_contact.topic (/comb/contact);
    # the mission filters by collision pair, so either side opens the gate.
    sensor = _sub(branch_link, "sensor", name="target_branch_contact_sensor", type="contact")
    _sub(sensor, "always_on", "true")
    _sub(sensor, "update_rate", "50")
    contact = _sub(sensor, "contact")
    _sub(contact, "collision", branch_cfg["collision"])
    _sub(contact, "topic", "/comb/contact")

    olives_cfg = target["olives"]
    detach_pattern = orchard["topics"]["detach_pattern"]
    state_pattern = orchard["topics"]["state_pattern"]
    for i in range(target["olive_count"]):
        olive_name = f"{olives_cfg['prefix']}_{i}"
        plugin = _sub(
            model,
            "plugin",
            filename="gz-sim-detachable-joint-system",
            name="gz::sim::systems::DetachableJoint",
        )
        _sub(plugin, "parent_link", branch_cfg["name"])
        _sub(plugin, "child_model", olive_name)
        _sub(plugin, "child_link", "link")
        _sub(plugin, "detach_topic", detach_pattern.format(olive=olive_name))
        _sub(plugin, "output_topic", state_pattern.format(olive=olive_name))


def _build_olive(world, orchard, index, xyz):
    target = orchard["target_tree"]
    olives_cfg = target["olives"]
    name = f"{olives_cfg['prefix']}_{index}"

    size = olives_cfg["size_xyz"]
    radius = size[0] / 2.0
    length = size[2] - 2.0 * radius
    if length < 0.0:
        raise ValueError("olive z size must be at least its diameter")
    mass = olives_cfg["mass"]

    # Capsule inertia approximated by a solid cylinder of equal bounding box.
    height = size[2]
    ixx = mass * (3.0 * radius * radius + height * height) / 12.0
    izz = mass * radius * radius / 2.0

    model = _sub(world, "model", name=name)
    _sub(model, "static", "false")
    _sub(model, "pose", _pose_text(xyz))

    link = _sub(model, "link", name="link")
    _add_inertial(link, mass, ixx, ixx, izz)
    decay = _sub(link, "velocity_decay")
    _sub(decay, "linear", "0.05")
    _sub(decay, "angular", "0.2")

    capsule_pose = _pose_text((0.0, 0.0, 0.0))
    _add_collision(link, "olive_collision", capsule_pose, "capsule", radius=radius, length=length)
    color = OLIVE_PURPLE if index % 2 else OLIVE_GREEN
    _add_visual(link, "olive_visual", capsule_pose, "capsule", color, radius=radius, length=length)


def _build_catch_net(world, orchard):
    net_cfg = orchard["catch_net"]
    sx, sy = net_cfg["size"]
    rim = net_cfg["rim_height"]
    wall = 0.02

    model = _sub(world, "model", name=net_cfg["name"])
    _sub(model, "static", "true")
    _sub(model, "pose", _pose_text(net_cfg["center_xyz"]))
    link = _sub(model, "link", name="net_link")

    _add_visual(link, "net_floor_visual", _pose_text((0.0, 0.0, -0.01)), "box",
                NET_COLOR, size=(sx, sy, 0.02))
    _add_collision(link, "net_floor_collision", _pose_text((0.0, 0.0, -0.01)), "box",
                   size=(sx, sy, 0.02))

    walls = (
        ("north", (0.0, sy / 2.0 - wall / 2.0, rim / 2.0), (sx, wall, rim)),
        ("south", (0.0, -sy / 2.0 + wall / 2.0, rim / 2.0), (sx, wall, rim)),
        ("east", (sx / 2.0 - wall / 2.0, 0.0, rim / 2.0), (wall, sy - 2.0 * wall, rim)),
        ("west", (-sx / 2.0 + wall / 2.0, 0.0, rim / 2.0), (wall, sy - 2.0 * wall, rim)),
    )
    for side, xyz, size in walls:
        _add_visual(link, f"net_rim_{side}_visual", _pose_text(xyz), "box", NET_COLOR, size=size)
        _add_collision(link, f"net_rim_{side}_collision", _pose_text(xyz), "box", size=size)


def _build_lighting(world):
    scene = _sub(world, "scene")
    _sub(scene, "ambient", "0.4 0.4 0.4 1")
    _sub(scene, "background", "0.55 0.68 0.85 1")
    _sub(scene, "shadows", "true")

    sun = _sub(world, "light", name="sun", type="directional")
    _sub(sun, "cast_shadows", "true")
    _sub(sun, "pose", "0 0 10 0 0 0")
    _sub(sun, "diffuse", "0.85 0.82 0.75 1")
    _sub(sun, "specular", "0.2 0.2 0.2 1")
    attenuation = _sub(sun, "attenuation")
    _sub(attenuation, "range", "1000")
    _sub(attenuation, "constant", "0.9")
    _sub(attenuation, "linear", "0.01")
    _sub(attenuation, "quadratic", "0.001")
    _sub(sun, "direction", "-0.5 0.1 -0.9")


def build_world(orchard):
    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    sdf.append(ET.Comment(GENERATED_NOTICE))
    world = _sub(sdf, "world", name=orchard["name"])

    _sub(world, "gravity", "0 0 -9.8")
    _sub(world, "magnetic_field", "6.0e-6 2.3e-5 -4.2e-5")
    _sub(world, "atmosphere", type="adiabatic")

    physics = _sub(world, "physics", name="default_physics", type="ignored")
    _sub(physics, "max_step_size", _n(float(orchard["physics"]["max_step_size"])))
    _sub(physics, "real_time_update_rate", _n(float(orchard["physics"]["real_time_update_rate"])))

    for filename, name, children in WORLD_PLUGINS:
        plugin = _sub(world, "plugin", filename=filename, name=name)
        for tag, value in children.items():
            _sub(plugin, tag, value)

    _build_lighting(world)
    _build_ground(world, orchard["ground"])

    rng = random.Random(orchard["seed"])
    target_index = None
    for index, x, y in tree_grid(orchard):
        tree_rng = random.Random(rng.random())
        _build_tree(world, orchard, index, x, y, tree_rng)
        if orchard["target_tree"]["name"] == f"olive_tree_{index}":
            target_index = index
    if target_index is None:
        raise ValueError("target tree is not part of the orchard grid")

    olives_cfg = orchard["target_tree"]["olives"]
    for i, xyz in enumerate(olives_cfg["positions"]):
        _build_olive(world, orchard, i, xyz)

    _build_catch_net(world, orchard)
    return sdf


# ---------------------------------------------------------------------------
# Bridge configuration
# ---------------------------------------------------------------------------

def build_bridge_config(orchard):
    entries = list(STATIC_BRIDGES)
    olives_cfg = orchard["target_tree"]["olives"]
    for i in range(orchard["target_tree"]["olive_count"]):
        olive_name = f"{olives_cfg['prefix']}_{i}"
        entries.append(
            (
                orchard["topics"]["detach_pattern"].format(olive=olive_name),
                "std_msgs/msg/Empty",
                "gz.msgs.Empty",
                "ROS_TO_GZ",
            )
        )
        entries.append(
            (
                orchard["topics"]["state_pattern"].format(olive=olive_name),
                "std_msgs/msg/String",
                "gz.msgs.StringMsg",
                "GZ_TO_ROS",
            )
        )

    lines = [
        "# GENERATED FILE - do not edit. Rebuild from config/orchard.yaml",
        "# with tools/generate_orchard.py.",
    ]
    for topic, ros_type, gz_type, direction in entries:
        lines.extend(
            [
                f'- ros_topic_name: "{topic}"',
                f'  gz_topic_name: "{topic}"',
                f'  ros_type_name: "{ros_type}"',
                f'  gz_type_name: "{gz_type}"',
                "  subscriber_queue: 10",
                "  publisher_queue: 10",
                "  lazy: false",
                f"  direction: {direction}",
            ]
        )
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="Path to orchard.yaml (SSOT)")
    parser.add_argument("--world-out", required=True, help="Output path for the SDF world")
    parser.add_argument("--bridge-out", required=True, help="Output path for bridge.yaml")
    args = parser.parse_args(argv)

    config = load_config(args.config)
    orchard = config["orchard"]

    validate_layout(orchard)

    world = build_world(orchard)
    ET.indent(world, space="  ")
    world_out = Path(args.world_out)
    world_out.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(world)
    tree.write(world_out, encoding="utf-8", xml_declaration=True)

    bridge_out = Path(args.bridge_out)
    bridge_out.parent.mkdir(parents=True, exist_ok=True)
    bridge_out.write_text(build_bridge_config(orchard), encoding="utf-8")

    olive_count = orchard["target_tree"]["olive_count"]
    print(
        f"Generated {world_out} (world '{orchard['name']}', "
        f"{len(tree_grid(orchard))} trees, {olive_count} detachable olives) "
        f"and {bridge_out}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
