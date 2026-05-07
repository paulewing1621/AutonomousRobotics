import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[
                {'autorepeat_rate': 10.},
                {'dev': "/dev/input/js0"},
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='floor_nav', executable='floornav_task_server', name='floor_tasks',
            parameters=[
                {'lib_path': os.path.join(os.getenv("HOME"),"Autonomous-Robotics/ros2_ws/install/floor_nav/lib/floor_nav")},
                {'base_frame': 'base_link'},
                {'reference_frame': 'map'},
                ],
            remappings=[
                #('~/clouds3d', '/points'),
                ('~/scan', '/kinect/scan'),
                ],
            output='screen'),

    ])
