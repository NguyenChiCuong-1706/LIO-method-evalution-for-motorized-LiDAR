#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('voxel_slam')
    config_file = os.path.join(package_share_dir, 'config', 'mid360.yaml')
    rviz_config = os.path.join(package_share_dir, 'rviz_cfg', 'back.rviz')
    save_path = os.path.join(os.path.expanduser('~'), 'save_datas')
    os.makedirs(save_path, exist_ok=True)

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Whether to launch RViz2'
    )

    voxelslam_node = Node(
        package='voxel_slam',
        executable='voxelslam',
        name='voxelslam',
        output='screen',
        parameters=[config_file, {
            'finish': False,
            'General.odometry_only': True,
            'General.is_save_map': 0,
            'General.save_path': save_path + os.sep,
        }],
    )

    odometry_adapter_node = Node(
        package='voxel_slam',
        executable='tf_to_odometry',
        name='voxel_tf_to_odometry',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([rviz_arg, voxelslam_node, odometry_adapter_node, rviz_node])
