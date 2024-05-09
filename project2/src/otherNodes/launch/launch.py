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

    # Lidar connection 
    lidar_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sick_scan_xd'),
                         'launch/sick_tim_7xx.launch.py')
        )
    )

    # IMU connection 
    imu_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('phidgets_spatial'),
                         'launch/spatial-launch.py')
        )
    )

    # Slam toolbox connection 
    slam_toolbox = Node( 
        package='slam_toolbox', 
        executable='async_slam_toolbox_node', 
        parameters=[
                get_package_share_directory('otherNodes') + '/config/mapping.yaml'
        ], 
        output='screen',
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('depthai_ros_driver'),
                         'launch/camera.launch.py')),
            launch_arguments={
                'usb_device_path': '/dev/bus/usb/003/009'
            }.items()
    ) 

    #controller = Node(
    	#package='controller',
    	#executable="controller",
    #)

    return LaunchDescription([
        joy_node,
        joy_teleop_node,
        # lidar_sensor,
        # imu_sensor,
        # slam_toolbox,
        camera
    ])
