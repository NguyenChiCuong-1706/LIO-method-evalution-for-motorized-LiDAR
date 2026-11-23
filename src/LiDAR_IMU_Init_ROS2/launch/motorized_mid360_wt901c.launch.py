import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lidar_imu_init')
    config = os.path.join(package_share, 'config', 'motorized_mid360_wt901c.yaml')
    default_result_dir = os.path.join(os.path.expanduser('~'), 'lm_calibr', 'data', 'li_init_results')
    return LaunchDescription([
        DeclareLaunchArgument('result_dir', default_value=default_result_dir),
        Node(
            package='lidar_imu_init',
            executable='li_init',
            name='laser_mapping',
            output='screen',
            parameters=[config, {'result_dir': LaunchConfiguration('result_dir')}],
        ),
    ])
