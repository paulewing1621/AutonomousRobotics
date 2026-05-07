import os

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
            package='topic_tools', executable='mux', name='cmd_mux',
            arguments=['/vrep/twistCommand','/teleop/twistCommand','/mux/autoCommand'],
            parameters=[
                {'output_topic': '/vrep/twistCommand'},
                {'input_topics': ['/teleop/twistCommand','/mux/autoCommand']},
                ],
            output='screen'),


        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_node', name='teleop',
            parameters=[
                {'~/axis_linear_x': 1},
                {'~/axis_angular': 0},
                {'~/scale_linear_x': 0.2},
                {'~/scale_angular': 1.},
                {'~/timeout': 1.0}
                ],
            remappings=[
                ('~/twistCommand', '/teleop/twistCommand'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_mux_node', name='teleop_mux',
            parameters=[
                {'~/joystick_button': 0},
                {'~/joystick_topic': '/teleop/twistCommand'},
                {'~/auto_button': 1},
                {'~/auto_topic': '/vrep/safeCommand'}
                ],
            remappings=[
                ('select', '/cmd_mux/select'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='collision_avoidance_base', executable='collision_avoidance_base', name='collision_avoidance',
            parameters=[
                {'~/safety_diameter': 0.15},
                {'~/ignore_diameter': 1.0},
                {'~/max_velocity': 1.0},
                {'~/only_forward': False},
                ],
            remappings=[
                ('~/clouds', '/points'),
                ('~/scans', '/vrep/hokuyo'),
                ('~/vel_input', '/mux/autoCommand'),
                ('~/vel_output', '/vrep/safeCommand'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner_base', executable='occgrid_planner_base', name='occgrid_planner',
            parameters=[
                {'~/neighbourhood': 8},
                {'~/base_frame': 'bubbleRob'},
                {'~/debug': False},
                {'~/headless': False},
                {'~/robot_radius': 0.30}
                ],
            remappings=[
                ('~/occ_grid', '/map'),
                ('~/goal', '/goal_pose'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner_base', executable='path_optimizer_base', name='path_optimizer',
            parameters=[
                {'~/max_acceleration': 0.3},
                {'~/max_braking': 0.1},
                {'~/velocity': 0.1},
                ],
            remappings=[
                ('~/path', '/occgrid_planner/path'),
                ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner_base', executable='path_follower_base', name='path_follower',
            parameters=[
                {'~/Kx': 1.0},
                {'~/Ky': 0.0},
                {'~/Ktheta': 1.0},
                {'~/max_rot_speed': 1.0},
                {'~/max_velocity': 0.1},
                {'~/max_y_error': 1.0},
                {'~/max_error': 0.5},
                {'~/look_ahead': 1.0},
                {'~/base_frame': 'bubbleRob'},
                ],
            remappings=[
                ('~/traj', '/path_optimizer/trajectory'),
                ('~/twistCommand', '/mux/autoCommand'),
                ('~/goal', '/goal_pose'),
                ],
            output='screen'),

    ])
