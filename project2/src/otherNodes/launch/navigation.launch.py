import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'), '/launch/navigation_launch.py']),
        launch_arguments={
            'params_file': get_package_share_directory('otherNodes') + '/config/navigation.yaml'
        }.items()
    )

    robot_position = Node(
        package="otherNodes",
        executable="robot_position"
    )

    explorer = Node(
        package="otherNodes",
        executable="explorer"
    )

    # waypoint_follower = Node(
    #     package="otherNodes",
    #     executable="waypoint_follower"
    # )
    
    return LaunchDescription([
        navigation,
        explorer, 
        robot_position
        # waypoint_follower
    ])