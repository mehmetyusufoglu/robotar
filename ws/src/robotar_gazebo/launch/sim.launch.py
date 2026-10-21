"""Launch the Robotar Gazebo Harmonic orchard simulation.

Bringup order:

1. Gazebo Harmonic with the generated orchard world (``world`` argument).
2. Optionally ``robotar_description``'s ``description.launch.py`` so that
   ``robot_state_publisher`` publishes ``robot_description``.
3. Spawn the robot into the world from the ``robot_description`` topic.
4. The generated ros_gz bridge configuration (clock, RGB-D, IMU, tool
   contact/IMU topics, per-olive detach/state topics).
5. ros2_control controller spawners. The spawners wait for the Gazebo
   ``controller_manager`` service themselves, so no fixed startup timers
   are needed. The tool controllers (``comb_controller``,
   ``gripper_controller`` or the shaker clamp/rotor pair) are spawned
   according to the ``tool`` argument.

Wheel geometry is not duplicated anywhere: this launch file reads
``robotar_description/config/physical.yaml`` and merges ``wheel_track``,
``wheel_radius`` and ``cmd_vel_timeout`` into a runtime copy of
``config/controllers.yaml`` that is handed to the spawners.
"""

import os
import tempfile

import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

GAZEBO_PACKAGE = "robotar_gazebo"
DESCRIPTION_PACKAGE = "robotar_description"


def _source_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _package_file(package, *parts):
    """Resolve a file inside a package share directory.

    Falls back to the source tree (``ws/src`` layout) so the launch file can
    be evaluated before an install space exists.
    """

    try:
        return os.path.join(get_package_share_directory(package), *parts)
    except PackageNotFoundError:
        return os.path.join(os.path.dirname(_source_root()), package, *parts)


def _gazebo_file(*parts):
    try:
        return os.path.join(get_package_share_directory(GAZEBO_PACKAGE), *parts)
    except PackageNotFoundError:
        return os.path.join(_source_root(), *parts)


def _load_yaml(path):
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _load_project_configs():
    """Read orchard.yaml and physical.yaml; tolerate a missing description."""

    try:
        orchard = _load_yaml(_gazebo_file("config", "orchard.yaml"))["orchard"]
    except (FileNotFoundError, KeyError):
        orchard = {}
    try:
        physical = _load_yaml(_package_file(DESCRIPTION_PACKAGE, "config", "physical.yaml"))
    except FileNotFoundError:
        physical = None
    return orchard, physical


def _write_runtime_controller_params():
    """Merge physical.yaml wheel geometry into a runtime controller file.

    Returns the generated file path, or None when the description package
    configuration is unavailable (world-only bringup without controllers).
    """

    try:
        controllers = _load_yaml(_gazebo_file("config", "controllers.yaml"))
        physical = _load_yaml(_package_file(DESCRIPTION_PACKAGE, "config", "physical.yaml"))
        base = physical["robotar"]["base"]
        drive = controllers["drive_controller"]["ros__parameters"]
        drive["wheel_separation"] = float(base["wheel_track"])
        drive["wheel_radius"] = float(base["wheel_radius"])
        drive["cmd_vel_timeout"] = float(base["limits"]["cmd_vel_timeout"])
    except (FileNotFoundError, KeyError, PackageNotFoundError) as error:
        return None, str(error)

    path = os.path.join(
        tempfile.gettempdir(), f"robotar_controllers_runtime_{os.getpid()}.yaml"
    )
    with open(path, "w", encoding="utf-8") as handle:
        yaml.safe_dump(controllers, handle, default_flow_style=False)
    return path, None


def generate_launch_description():
    orchard, physical = _load_project_configs()
    robot_cfg = orchard.get("robot", {})
    spawn_xyz = robot_cfg.get("spawn_xyz", [0.0, -1.3, 0.25])
    spawn_yaw = robot_cfg.get("spawn_yaw", 0.0)
    world_name_default = orchard.get("name", "robotar_olive_row")
    robot_name_default = "robotar"
    if physical:
        robot_name_default = physical.get("robotar", {}).get("name", robot_name_default)

    runtime_params, runtime_params_error = _write_runtime_controller_params()

    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    robot_name = LaunchConfiguration("robot_name")
    gui = LaunchConfiguration("gui")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    tool = LaunchConfiguration("tool")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw_arg = LaunchConfiguration("spawn_yaw")
    start_description = LaunchConfiguration("start_description")
    spawn_robot = LaunchConfiguration("spawn_robot")
    start_bridge = LaunchConfiguration("start_bridge")
    start_controllers = LaunchConfiguration("start_controllers")

    declared_arguments = [
        DeclareLaunchArgument(
            "world",
            default_value=_gazebo_file("worlds", "robotar_orchard.sdf"),
            description="Generated orchard SDF world (built from orchard.yaml).",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value=world_name_default,
            description="Name of the Gazebo world inside the SDF file.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value=robot_name_default,
            description="Gazebo model name for the spawned robot.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Run the Gazebo GUI; 'false' runs the server headless.",
        ),
        DeclareLaunchArgument(
            "gz_verbosity",
            default_value="1",
            description="Gazebo console verbosity (0-4).",
        ),
        DeclareLaunchArgument(
            "tool",
            default_value="comb",
            description="End effector mounted by robotar_description: comb, gripper or shaker.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value=str(spawn_xyz[0])),
        DeclareLaunchArgument("spawn_y", default_value=str(spawn_xyz[1])),
        DeclareLaunchArgument("spawn_z", default_value=str(spawn_xyz[2])),
        DeclareLaunchArgument("spawn_yaw", default_value=str(spawn_yaw)),
        DeclareLaunchArgument(
            "start_description",
            default_value="true",
            description="Include robotar_description/launch/description.launch.py.",
        ),
        DeclareLaunchArgument(
            "spawn_robot",
            default_value="true",
            description="Spawn the robot from the robot_description topic.",
        ),
        DeclareLaunchArgument(
            "start_bridge",
            default_value="true",
            description="Start the ros_gz bridge with the generated config.",
        ),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=_gazebo_file("config", "bridge.yaml"),
            description="Generated ros_gz_bridge YAML configuration.",
        ),
        DeclareLaunchArgument(
            "start_controllers",
            default_value="true",
            description="Spawn the ros2_control controllers.",
        ),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                "-r -v",
                gz_verbosity,
                " ",
                PythonExpression(["'-s ' if '", gui, "' == 'false' else ''"]),
                world,
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(DESCRIPTION_PACKAGE), "launch", "description.launch.py"]
            )
        ),
        # controllers_file bakes the runtime controller parameters (including
        # physical.yaml wheel geometry) into robot_description so the
        # gz_ros2_control Gazebo plugin starts a controller_manager on spawn.
        launch_arguments={
            "tool": tool,
            "use_sim_time": use_sim_time,
            "controllers_file": runtime_params if runtime_params else "",
            "rviz": gui,
        }.items(),
        condition=IfCondition(start_description),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", world_name,
            "-topic", "robot_description",
            "-name", robot_name,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw_arg,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(spawn_robot),
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_bridge"), "launch", "ros_gz_bridge.launch.py"]
            )
        ),
        launch_arguments={
            "bridge_name": "robotar_gz_bridge",
            "config_file": LaunchConfiguration("bridge_config"),
        }.items(),
        condition=IfCondition(start_bridge),
    )

    def spawner(name, tool_condition=None):
        arguments = [
            name,
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "30.0",
        ]
        if runtime_params:
            arguments += ["--param-file", runtime_params]
        if tool_condition is None:
            condition = IfCondition(start_controllers)
        else:
            condition = IfCondition(
                PythonExpression(
                    ["'", start_controllers, "' == 'true' and '", tool, "' == '", tool_condition, "'"]
                )
            )
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=arguments,
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            condition=condition,
        )

    entities = list(declared_arguments)
    if runtime_params is None:
        entities.append(
            LogInfo(
                msg=(
                    "physical.yaml/controllers.yaml unavailable "
                    f"({runtime_params_error}); controllers rely on the "
                    "parameters baked into robot_description."
                )
            )
        )
    entities.extend(
        [
            gazebo,
            description,
            spawn,
            bridge,
            spawner("joint_state_broadcaster"),
            spawner("drive_controller"),
            spawner("arm_controller"),
            spawner("comb_controller", tool_condition="comb"),
            spawner("gripper_controller", tool_condition="gripper"),
            spawner("shaker_clamp_controller", tool_condition="shaker"),
            spawner("shaker_rotor_controller", tool_condition="shaker"),
        ]
    )
    return LaunchDescription(entities)
