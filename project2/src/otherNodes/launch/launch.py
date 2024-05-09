import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Joy/teleop twist joy node creation
    joy_params = os.path.join(get_package_share_directory('otherNodes'), 'config', 'joystick.yaml')
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[joy_params]
    )

    # Joy tele-op twist node creation
    joy_teleop_node = Node(
        package="teleop_twist_joy",
        executable = "teleop_node",
        name='teleop_node',
        parameters=[joy_params]
    )

    lidar_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sick_scan_xd'),
                         'launch/sick_tim_7xx.launch.py')
        )
    )

    # camera = Node(
    #         package='depthai_ros',  # Replace with actual package name
    #         executable='oak_node',  # Replace with actual executable name
    #         name='oak_camera_node',
    #         parameters=[{
    #             'usb_device_path': '/dev/bus/usb/003/009'  # Device USB path
    #         }],
    #         output='screen',
    # )
    
    #controller = Node(
    	#package='controller',
    	#executable="controller",
    #)

    return LaunchDescription([
        joy_node,
        joy_teleop_node,
        lidar_sensor
    ])
