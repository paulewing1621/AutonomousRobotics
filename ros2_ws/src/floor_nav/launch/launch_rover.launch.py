import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import launch_ros.actions
import launch_ros.descriptions

base_dir=str(Path(get_package_share_directory("floor_nav")).parents[1])

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='floor_nav', 
            executable='floornav_task_server', 
            name='floor_tasks',
            parameters=[
                {'lib_path': os.path.join(base_dir, "lib", "floor_nav"),
                'base_frame': 'rover_body',      
                'reference_frame': 'odom'        
                },
            ],
            remappings=[
                ('~/cloud3d', '/points'),
                ('~/scan', '/vrep/hokuyo'),
            ],
            output='screen'),
    ])