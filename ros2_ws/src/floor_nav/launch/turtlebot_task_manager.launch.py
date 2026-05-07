# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_ros.actions


def generate_launch_description():
    ca_launch_dir = get_package_share_directory('collision_avoidance_base')
    ca_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ca_launch_dir, 'ca.launch.py')
        )
    )

    floor_nav_launch_dir = get_package_share_directory('floor_nav')
    floor_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(floor_nav_launch_dir, 'launch_server_tb.launch.py')
        )
    )

    return LaunchDescription([
        ca_launch,
        floor_nav_launch,

        launch_ros.actions.Node(
            package='floor_nav', executable='dock_toggle_button.py', name='dock_toggle_button',
            parameters=[
                {'~/joy_topic': '/joy'},
                {'~/toggle_button': 2},
                {'~/start_docked': True},
                {'~/undock_reverse_duration': 1.2},
                {'~/undock_reverse_speed': 0.06},
                {'~/undock_turn_angle': 3.141592653589793},
            ],
            output='screen'),

    ])
