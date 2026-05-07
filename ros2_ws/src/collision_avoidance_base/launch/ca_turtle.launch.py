import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='collision_avoidance_base',
            executable='collision_avoidance_base',
            name='collision_avoidance',
            parameters=[
                {'~/safety_diameter': 0.4},
                {'~/ignore_diameter': 1.0},
                {'~/max_velocity': 0.2},
                {'~/only_forward': True},
            ],
            remappings=[
                ('~/scans', '/scan'),
                ('~/vel_input', '/mux/autoCommand'),
                ('~/vel_output', '/commands/velocity'),
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='occgrid_planner_base', executable='occgrid_planner_base', name='occgrid_planner',
            parameters=[
                {'~/neighbourhood': 8},
                {'~/base_frame': 'base_footprint'},
                {'~/robot_radius': 0.3},
                {'~/debug': False},
                {'~/headless': False},
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
                {'~/velocity': 0.2},
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
                {'~/Ktheta': 2.0},
                {'~/max_rot_speed': 1.0},
                {'~/max_velocity': 0.3},
                {'~/max_y_error': 1.0},
                {'~/max_error': 1.5},
                {'~/look_ahead': 1.5},
                {'~/base_frame': 'base_footprint'},
            ],
            remappings=[
                ('~/traj', '/path_optimizer/trajectory'),
                ('~/twistCommand', '/mux/autoCommand'),
                ('~/goal', '/goal_pose'),
            ],
            output='screen'),
    ])
