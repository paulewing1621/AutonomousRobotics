import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot_launch'),
                         'slam_tb_sync.launch.py')
        )
    )

    return LaunchDescription([
        slam_launch,
        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[
                {'autorepeat_rate': 10.},
                {'dev': "/dev/input/js0"},
            ],
            output='screen'),

        launch_ros.actions.Node(
            package='vrep_ros_teleop', executable='teleop_node', name='teleop',
            parameters=[
                {'~/axis_linear_x': 1},
                {'~/axis_angular': 0},
                {'~/scale_linear_x': 0.4},
                {'~/scale_angular': 1.0},
                {'~/timeout': 10.0},
            ],
            remappings=[
                ('~/twistCommand', '/commands/velocity'),
            ],
            output='screen'),
        
        launch_ros.actions.Node(
        package='wifi_map_base', executable='wifi_map_node', name='wifi_map',
        parameters=[
            {'~/base_link': 'base_link'},
            {'~/ignore_header': True},
            #{'~/bssid': '5C:A6:E6:34:74:E6'},
            {'~/bssid': 'first'},
            {'~/measurement_radius': 2.0},
            {'use_sim_time': False},
            ],
        remappings=[
            ('~/occ_grid', '/map'),
            ('~/scan', '/wpa_cli/scan'),
            ],
        output='screen')
    ])
