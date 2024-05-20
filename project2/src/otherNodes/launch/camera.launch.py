import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')
        ),
        launch_arguments={
            'usb_device_path': '/dev/bus/usb/003/009'
        }.items()
    )

    cone_detector = Node(
        package="otherNodes",
        executable="cone_detector"
    )
    
    # number_detector = Node(
    #     package="otherNodes",
    #     executable="number_detector"
    # )

    return LaunchDescription([
        camera,
        cone_detector,
        # number_detector
    ])
