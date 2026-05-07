# autonomous_mission.launch.py
# Place this file in: floor_nav/launch/
#
# Launches:
#   1. CA + planner stack   (ca_turtle.launch.py  from collision_avoidance_base)
#   2. Floor nav task server(launch_server_tb.launch.py  from floor_nav)
#   3. SLAM Toolbox sync    (slam_tb_sync.launch.py      from turtlebot_launch)
#   4. Autonomous explore-dock mission node
#
# Run with:
#   ros2 launch floor_nav autonomous_mission.launch.py use_sim_time:=True
# On real robot:
#   ros2 launch floor_nav autonomous_mission.launch.py use_sim_time:=False

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

import launch_ros.actions


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Launch argument: use_sim_time                                        #
    # ------------------------------------------------------------------ #
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use /clock from simulation (set False on real robot)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------------ #
    # 1. CA + planner stack                                               #
    #    (lives in collision_avoidance_base package)                      #
    #    Includes: joy, cmd_mux, teleop, teleop_mux,                      #
    #              collision_avoidance, occgrid_planner,                  #
    #              path_optimizer, path_follower                          #
    # ------------------------------------------------------------------ #
    ca_launch_dir = get_package_share_directory('collision_avoidance_base')
    ca_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ca_launch_dir, 'ca_turtle.launch.py')
        )
    )

    # ------------------------------------------------------------------ #
    # 2. Floor-nav task server                                             #
    #    (lives in floor_nav package)                                     #
    # ------------------------------------------------------------------ #
    floor_nav_launch_dir = get_package_share_directory('floor_nav')
    floor_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(floor_nav_launch_dir, 'launch_server_tb.launch.py')
        )
    )

    # ------------------------------------------------------------------ #
    # 3. SLAM Toolbox sync                                                 #
    #    Replaces running this manually:                                  #
    #    ros2 launch turtlebot_launch slam_tb_sync.launch.py              #
    #               use_sim_time:=True                                    #
    # ------------------------------------------------------------------ #
    slam_launch_dir = get_package_share_directory('turtlebot_launch')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_launch_dir, 'slam_tb_sync.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ------------------------------------------------------------------ #
    # 4. Autonomous explore-dock mission node                              #
    #                                                                      #
    # Velocity topology (from ca_turtle.launch.py):                       #
    #   teleop        → /teleop/twistCommand                              #
    #   mux auto chan → /mux/autoCommand                                  #
    #   CA vel_input  → /teleop/twistCommand                              #
    #   CA vel_output → /commands/velocity  (final drive command)         #
    # ------------------------------------------------------------------ #
    mission_node = launch_ros.actions.Node(
        package='floor_nav',
        executable='explore_dock_loop.py',
        name='explore_dock_mission',
        parameters=[
            {'use_sim_time': use_sim_time},

            # --- Home pose ---
            # auto_capture_home=True reads pose from SLAM at start-up.
            # Robot must be docked when launched.
            {'~/auto_capture_home': True},
            {'~/home_x': 0.0},
            {'~/home_y': 0.0},
            {'~/home_theta': 0.0},

            # --- Undock motion ---
            {'~/undock_reverse_duration': 1.2},
            {'~/undock_reverse_speed': 0.06},
            {'~/undock_turn_angle': 3.141592653589793},

            # --- Exploration ---
            {'~/explore_seconds': 120.0},

            # --- Pre-dock staging ---
            {'~/predock_offset': 0.40},
            {'~/predock_dist_threshold': 0.20},

            # --- Docking retries ---
            {'~/dock_retries': 3},
            {'~/dock_retry_reverse': 0.80},
            {'~/dock_retry_reverse_speed': 0.05},

            # --- Service names (must match ca_turtle.launch.py) ---
            {'~/mux_service_name': '/cmd_mux/select'},
            {'~/ca_enable_service': '/collision_avoidance/enable'},
        ],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # Assemble                                                             #
    # ------------------------------------------------------------------ #
    return LaunchDescription([
        use_sim_time_arg,
        ca_launch,
        floor_nav_launch,
        slam_launch,
        mission_node,
        TimerAction(period=10.0, actions=[mission_node]),
    ])
