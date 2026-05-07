import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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
            package='topic_tools', executable='mux', name='cmd_mux',
            arguments=['/vrep/safeCommand','/teleop/twistCommand','/mux/autoCommand'], # /commands/velocity
            parameters=[
                {'output_topic': '/vrep/safeCommand'}, # /commands/velocity
                {'input_topics': ['/teleop/twistCommand','/mux/autoCommand']},
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_node', name='teleop',
            parameters=[
                {'~/axis_linear_x': 1},
                {'~/axis_angular': 0},
                {'~/scale_linear_x': 1.0},
                {'~/scale_angular': 1.0},
                {'~/slow_scale_linear_x': 0.2},
                {'~/slow_scale_linear_y': 0.0},
                {'~/slow_scale_angular': 0.5},
                {'~/timeout': 1.0}
                ],
            remappings=[
                ('twistCommand', '/teleop/twistCommand'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_mux',
            parameters=[
                {'~/joystick_button': 0},
                {'~/joystick_topic': '/teleop/twistCommand'},
                {'~/auto_button': 1},
                {'~/auto_topic': '/mux/autoCommand'}
                ],
            remappings=[
                ('select', '/cmd_mux/select'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='collision_avoidance_base', executable='collision_avoidance_base', name='collision_avoidance',
            parameters=[
                {'~/safety_diameter': 0.3},
                {'~/buffer_diameter': 0.4},
                {'~/ignore_diameter': 1.5},
                {'~/only_forward': False},
                {'~/bogus_point_distance': 0.2},
                {'~/cone_half_angle_deg': 10.0},
                ],
            remappings=[
                ('~/scans', '/scan'), #/vrep/hokuyo
                ('~/vel_input', '/vrep/safeCommand'),
                ('~/vel_output', '/commands/velocity'), # /vrep/twistCommand
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner_base', executable='occgrid_planner_base', name='occgrid_planner',
            parameters=[
                {'~/neighbourhood': 8},
                {'~/base_frame': 'base_link'}, # bubbleRob
                {'~/robot_radius': 0.2},
                {'~/heading_bins': 8},
                {'~/rotation_cost': 0.25},
                {'~/distance_weight': 1.0},
                {'~/exploration_period': 7.0},
                {'~/exploration_button': 3},
                {'~/exploration_enabled': False},
                {'~/joy_topic': '/joy'},
                {'~/debug': False},
                {'~/headless': False},
                {'~/display_min_size_px': 900},
                {'~/display_max_size_px': 1600},
                ],
            remappings=[
                ('~/occ_grid', '/map'),
                ('~/goal', '/goal_pose'),
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner_base', executable='path_optimizer_base', name='path_optimizer',
            parameters=[
                {'~/max_acceleration': 0.2},
                {'~/max_braking': 0.15},
                {'~/velocity': 0.08},
                ],
            remappings=[
                ('~/path', '/occgrid_planner/path'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner_base', executable='path_follower_base', name='path_follower',
            parameters=[
                {'~/Kx': 0.7},
                {'~/Ky': 0.2},
                {'~/Ktheta': 0.6},
                {'~/max_rot_speed': 0.5},
                {'~/max_velocity': 0.08},
                {'~/max_y_error': 0.5},
                {'~/max_error': 0.5},
                {'~/look_ahead': 0.6},
                {'~/base_frame': 'base_link'}, # bubbleRob
                ], 
            remappings=[
                ('~/traj', '/path_optimizer/trajectory'),
                ('~/twistCommand', '/mux/autoCommand'),
                ('~/goal', '/goal_pose'),
                ],
            output='screen'),

    ])
