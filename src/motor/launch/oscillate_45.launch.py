"""Automatically oscillate theta1 between +45 and -45 degrees around HOME."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("can_bitrate", default_value="500000"),
        DeclareLaunchArgument("direction_sign", default_value="1"),
        DeclareLaunchArgument("sweep_rpm", default_value="10"),
        DeclareLaunchArgument("max_runtime_s", default_value="300"),
        Node(
            package="mhf9025_joint_state",
            executable="oscillating_motor_node",
            name="mhf9025_oscillating_motor",
            output="screen",
            arguments=[
                "--device", LaunchConfiguration("device"),
                "--can-bitrate", LaunchConfiguration("can_bitrate"),
                "--motor-id", "1",
                "--direction-sign", LaunchConfiguration("direction_sign"),
                "--sweep-rpm", LaunchConfiguration("sweep_rpm"),
                "--max-runtime-s", LaunchConfiguration("max_runtime_s"),
            ],
        ),
    ])
