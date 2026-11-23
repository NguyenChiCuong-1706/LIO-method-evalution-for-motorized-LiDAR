"""Launch the read-only MHF9025-V3 joint-state publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("direction_sign", default_value="1"),
        Node(
            package="mhf9025_joint_state",
            executable="joint_state_node",
            name="mhf9025_joint_state",
            output="screen",
            arguments=[
                "--device", LaunchConfiguration("device"),
                "--can-bitrate", "500000",
                "--motor-id", "1",
                "--direction-sign", LaunchConfiguration("direction_sign"),
            ],
        ),
    ])
