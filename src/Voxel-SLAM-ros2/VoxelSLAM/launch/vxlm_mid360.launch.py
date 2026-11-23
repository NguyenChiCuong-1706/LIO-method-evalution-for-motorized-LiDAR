#!/usr/bin/env python3

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory('voxel_slam')
    save_path = os.path.join(os.path.expanduser('~'), 'save_datas')
    os.makedirs(save_path, exist_ok=True)

    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )
    bagname_arg = DeclareLaunchArgument(
        'bagname',
        default_value='mid360_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'),
        description='Name of this mapping session and its save directory',
    )
    previous_map_arg = DeclareLaunchArgument(
        'previous_map',
        default_value='',
        description='Saved session and threshold to load, e.g. mid360_session:0.45',
    )

    config_file = os.path.join(package_share_dir, 'config', 'mid360.yaml')
    rviz_config = os.path.join(package_share_dir, 'rviz_cfg', 'back.rviz')

    # VoxelSLAM node
    voxelslam_node = Node(
        package='voxel_slam',
        executable='voxelslam',
        name='voxelslam',
        output='screen',
        parameters=[config_file, {
            'finish': False,
            'General.bagname': LaunchConfiguration('bagname'),
            'General.save_path': save_path + os.sep,
            'General.previous_map': LaunchConfiguration('previous_map'),
        }],
    )

    # RViz2 node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        rviz_arg,
        bagname_arg,
        previous_map_arg,
        voxelslam_node,
        rviz_node,
    ])
