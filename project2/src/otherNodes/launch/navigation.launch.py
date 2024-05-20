import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    navigation_config_file = get_package_share_directory('otherNodes') + '/config/navigation.yaml'
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': navigation_config_file
        }.items(),
    )

    # robot_position = Node(
    #     package="otherNodes",
    #     executable='robot_position'
    # )
    
    return LaunchDescription([
        nav2_launch
        # robot_position
    ])