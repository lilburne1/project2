import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # # Joy/teleop twist joy node creation
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
        executable="teleop_node",
        name='teleop_node',
        parameters=[joy_params],
        remappings = [
            ('cmd_vel', 'cmd_vel_joy')
        ]
    )

    # Lidar connection 
    lidar_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sick_scan_xd'), 'launch', 'sick_tim_7xx.launch.py')
        ),
    )

    robot_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("otherNodes"), 'config', 'ekf.yaml')],
            remappings=[
                ('/odometry/filtered', '/odom')
            ]
    )
    
    urdf_file = os.path.join(get_package_share_directory('otherNodes'), 'models', 'pioneer.urdf')
    urdf_file = get_package_share_directory("otherNodes") + "/models/pioneer.urdf"
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # TF Transform for joint publisher
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': False
        }]
    )

    slam_toolbox = Node( 
        package='slam_toolbox', 
        executable='async_slam_toolbox_node', 
        parameters=[
                get_package_share_directory('otherNodes') + '/config/mapping.yaml'
        ], 
        output='screen',
    )

    dead_man_switch = Node(
        package="otherNodes",
        executable='dead_switch'
    )

    # launch.LaunchDescription([
    #     launch.actions.ExecuteProcess(
    #         cmd=['ros2', 'bag', 'record', '-a'],
    #         output='screen'
    #     )
    # ])


    # camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('depthai_examples'), 'launch', 'tracker_yolov4_spatial_node.launch.py')
    #     ),
    #     launch_arguments={
    #         'usb_device_path': '/dev/bus/usb/003/009'
    #     }.items()
    # )

    return LaunchDescription([
        joy_node,
        joy_teleop_node,
        lidar_sensor,
        slam_toolbox,
        joint_state_pub,
        robot_state_pub,
        robot_localization,
        dead_man_switch,
    ])
