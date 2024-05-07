# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Connection with controller
    controller_connection = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params]
    )

    # Joy tele-op node creation
    controller_converter = Node(
        package="teleop_twist_joy",
        executable = "teleop_node",
        name='teleop_node',
        params=[joy_params],
    )

    camera = Node(
            package='depthai_ros',  # Replace with actual package name
            executable='oak_node',  # Replace with actual executable name
            name='oak_camera_node',
            parameters=[{
                'usb_device_path': '/dev/bus/usb/003/009'  # Device USB path
            }],
            output='screen',
    )
    
    #controller = Node(
    	#package='controller',
    	#executable="controller",
    #)

    return LaunchDescription([
        controller_connection,
        controller_converter,
        camera
    ])
