"""End-to-end branch clamp-shaker demo bringup.

Includes robotar_gazebo's sim.launch.py (always with the shaker tool — the
shaker mission requires it) and starts the two mission nodes:
``odom_driver`` (closed-loop row drive) and ``shaker_demo`` (mission state
machine). Mission parameters come from config/shaker_demo.yaml; the SSOT
config paths are injected here.

Launch arguments:
  gui          : 'true'/'false' (default 'false' — headless)
  miss_contact : 'true' runs the no-clamp-contact negative verification:
                 the arm uses the deliberate miss pose, the clamp never
                 verifies, no detach topics may be published and the
                 mission fails cleanly after releasing the jaws.
  gz_verbosity : forwarded to Gazebo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission_share = FindPackageShare("robotar_mission")
    gazebo_share = FindPackageShare("robotar_gazebo")
    description_share = FindPackageShare("robotar_description")

    params_file = PathJoinSubstitution(
        [mission_share, "config", "shaker_demo.yaml"]
    )
    orchard_path = PathJoinSubstitution(
        [gazebo_share, "config", "orchard.yaml"]
    )
    physical_path = PathJoinSubstitution(
        [description_share, "config", "physical.yaml"]
    )

    gui = LaunchConfiguration("gui")
    miss_contact = LaunchConfiguration("miss_contact")
    gz_verbosity = LaunchConfiguration("gz_verbosity")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_share, "launch", "sim.launch.py"])
        ),
        launch_arguments={
            "gui": gui,
            "tool": "shaker",
            "gz_verbosity": gz_verbosity,
        }.items(),
    )

    odom_driver = Node(
        package="robotar_mission",
        executable="odom_driver",
        name="odom_driver",
        output="screen",
        parameters=[
            params_file,
            {
                "orchard_config_path": orchard_path,
                "physical_config_path": physical_path,
            },
        ],
    )

    shaker_demo = Node(
        package="robotar_mission",
        executable="shaker_demo",
        name="shaker_demo",
        output="screen",
        parameters=[
            params_file,
            {
                "orchard_config_path": orchard_path,
                "physical_config_path": physical_path,
                "miss_contact": ParameterValue(miss_contact, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="false",
                description="Run the Gazebo GUI; 'false' runs headless.",
            ),
            DeclareLaunchArgument(
                "miss_contact",
                default_value="false",
                description="'true' selects the deliberate miss pose for the "
                            "no-clamp-contact negative verification run.",
            ),
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value="1",
                description="Gazebo console verbosity (0-4).",
            ),
            sim,
            odom_driver,
            shaker_demo,
        ]
    )
