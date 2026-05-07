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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True') 
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo, Bag) clock if true'),
        launch_ros.actions.Node(
            package='ar_loc_base', executable='ar_tbloc_node', name='ar_loc_base',
            parameters=[
                {'~/filter_name': 'particle'}, # in ['odo','kalman','particle']
                {'~/use_ar': True},
                {'~/target_frame': 'totalstation'},
                {'~/odom_frame': 'odom'}, 
                {'~/ar_precision': 0.5},
                {'~/initial_x': -5.2},
                {'~/initial_y': 8.3},
                {'~/initial_theta': -1.57},
                {'~/x_precision': 5e-3},
                {'~/y_precision': 1e-3},
                {'~/theta_precision': 1e-2},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='tf_recorder', executable='tf_recorder_node', name='tf_recorder',
            parameters=[
                {'reference_frame': 'totalstation'},
                {'target_frame': 'base_link'},
                {'recording_period': 0.5},
                {'output_file': '/tmp/traj.csv'},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            output='screen'),
    ])
