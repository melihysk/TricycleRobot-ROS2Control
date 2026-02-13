from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
                'use_sim_time': True
            }],
            output='screen'
        ),
        Node(
            package='offset_tricycle_robot',
            executable='joystick_teleop.py',
            name='joystick_teleop',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'max_linear_velocity': 1.0,
                'max_steering_angle': 1.5708,
                'wheelbase': 0.5,
                'deadzone': 0.1,
                'throttle_axis': 1,
                'steering_axis': 3,
            }]
        ),
    ])
