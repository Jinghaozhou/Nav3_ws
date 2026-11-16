from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory('nav3_navigation_manager'))
        / 'config' / 'navigation_manager.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='nav3_navigation_manager',
            executable='navigation_manager',
            name='nav3_navigation_manager',
            output='screen',
            parameters=[LaunchConfiguration('config_file'), {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
