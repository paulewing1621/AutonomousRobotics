from launch import LaunchDescription
import launch_ros.actions


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
            package='vrep_ros_teleop', executable='teleop_node', name='teleop',
            parameters=[
                {'~/axis_linear_x': 1},
                {'~/axis_angular': 0},
                {'~/scale_linear_x': 0.2},
                {'~/scale_angular': 1.0},
                {'~/timeout': 1.0},
            ],
            remappings=[
                ('twistCommand', '/commands/velocity'),
            ],
            output='screen'),
    ])
