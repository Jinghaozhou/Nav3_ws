"""Show the lightweight global map, local map window and navigation paths."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workspace = Path(get_package_prefix("nav3_visualization")).parent
    rviz_file = Path(get_package_share_directory("nav3_visualization")) / "rviz/nav3_navigation.rviz"
    default_map = workspace / "data/maps/building_split_space_viz_space10_open.pcd"
    return LaunchDescription([
        DeclareLaunchArgument("map_pcd", default_value=str(default_map)),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        Node(package="nav3_visualization", executable="map_publisher.py",
             name="nav3_visual_map_publisher", output="screen",
             parameters=[{"map_pcd": LaunchConfiguration("map_pcd")}]),
        Node(package="rviz2", executable="rviz2", name="nav3_navigation_rviz",
             arguments=["-d", str(rviz_file)], output="screen",
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
