import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    navigation_params = get_package_share_directory('otherNodes') + '/config/navigation.yaml'
    navigation = Node(
        package="nav2_bringup",
        executable="navigation_launch",
        name="nav2_bringup",
        parameters=[navigation_params],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]     
    )

    explorer = Node(
        package="otherNodes",
        executable="explorer"
    )

    waypoint_follower = Node(
        package="otherNodes",
        executable="waypoint_follower"
    )
    
    return LaunchDescription([
        navigation,
        explorer,
        waypoint_follower
    ])