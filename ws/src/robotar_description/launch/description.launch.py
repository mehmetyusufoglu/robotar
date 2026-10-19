"""TF/RViz bringup for the robotar description.

Publishes robot_description through robot_state_publisher and optionally
starts RViz. This entry point intentionally does not touch Gazebo or
controllers; simulation bringup lives in robotar_gazebo. The
controllers_file argument is forwarded to the xacro so a Gazebo launch
can reuse the same command line when spawning.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("robotar_description")
    default_model = PathJoinSubstitution([pkg_share, "urdf", "robotar.urdf.xacro"])
    default_rviz_config = PathJoinSubstitution([pkg_share, "rviz", "robotar.rviz"])

    tool = LaunchConfiguration("tool")
    controllers_file = LaunchConfiguration("controllers_file")
    rviz = LaunchConfiguration("rviz")

    robot_description = ParameterValue(
        Command([
            "xacro ", default_model,
            " tool:=", tool,
            " controllers_file:=", controllers_file,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "tool",
            default_value="comb",
            choices=["comb", "gripper"],
            description="Tool mounted on the tool0 flange (launch-time interchangeability).",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="",
            description="ros2_control parameter YAML consumed by the gz_ros2_control "
                        "Gazebo plugin; empty omits the plugin (TF/RViz-only use).",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz with the robotar config.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", default_rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
