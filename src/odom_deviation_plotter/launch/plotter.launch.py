"""
Launch file for the odometry deviation plotter.

Usage examples:

  # For tricycle_robot (default):
  ros2 launch odom_deviation_plotter plotter.launch.py

  # For offset_tricycle_robot:
  ros2 launch odom_deviation_plotter plotter.launch.py \
      wheel_odom_topic:=/offset_tricycle_controller/odom \
      cmd_vel_topic:=/offset_tricycle_controller/cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'wheel_odom_topic',
            default_value='/tricycle_controller/odom',
            description='Wheel odometry topic from the controller',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/tricycle_controller/cmd_vel',
            description='cmd_vel topic (TwistStamped)',
        ),
        DeclareLaunchArgument(
            'ground_truth_topic',
            default_value='/model/gebot/odometry',
            description='Gazebo ground truth odometry topic',
        ),
        DeclareLaunchArgument(
            'ekf_odom_topic',
            default_value='/odometry/filtered',
            description='EKF filtered odometry topic',
        ),
        DeclareLaunchArgument(
            'window_size',
            default_value='2500',
            description='Number of data points shown on the plot',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time',
        ),

        Node(
            package='odom_deviation_plotter',
            executable='odom_deviation_plotter.py',
            name='odom_deviation_plotter',
            output='screen',
            parameters=[{
                'wheel_odom_topic': LaunchConfiguration('wheel_odom_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'ground_truth_topic': LaunchConfiguration('ground_truth_topic'),
                'ekf_odom_topic': LaunchConfiguration('ekf_odom_topic'),
                'window_size': LaunchConfiguration('window_size'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
