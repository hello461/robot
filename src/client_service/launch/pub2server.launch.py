

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ros_bridge = get_package_share_directory('rosbridge_server')
    client_service = get_package_share_directory('client_service')

   
    return LaunchDescription([
       
        Node(
            package='client_service',
            executable='image_compressor',
            name='image_compressor',
            parameters=[{'bandwidth_mode':'medium','target_fps':10,'motion_detection':False,'grayscale':False}],
            output='screen',
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[
                {"port": 9090},         
                {"address": "0.0.0.0"}, 
                {"retry_startup_delay": 5.0}
            ]
        )
    ])