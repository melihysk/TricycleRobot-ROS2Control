"""
Bag replay launch file for offline odometry experiments.

Starts everything needed to replay a recorded bag and test odometry
algorithms with configurable sensor noise — no Gazebo, no controller
manager, no ROS-Gazebo bridge required.

#Record command:
ros2 bag record /clock /tf /tf_static /joint_states \
  /imu/data /scan \
  /offset_tricycle_controller/odom \
  /offset_tricycle_controller/cmd_vel \
  /model/forklift/odometry \
  -o ~/Desktop/ros_bag_record/warehouse_clean_sensor_data

Usage:
  # Terminal 1: start nodes
  ros2 launch offset_tricycle_robot bag_replay.launch.py \
  scan_stddev:=0.12 \
  imu_gyro_stddev:=0.015 imu_accel_stddev:=0.3 \
  odom_vx_stddev:=0.09 odom_vyaw_stddev:=0.09

  ros2 launch offset_tricycle_robot bag_replay.launch.py \
  odom_vx_stddev:=0.2 odom_vyaw_stddev:=0.2

  ros2 launch offset_tricycle_robot bag_replay.launch.py \
  scan_stddev:=0.12 \
  odom_vx_stddev:=0.09 odom_vyaw_stddev:=0.09

  # Terminal 2: play the bag (clock driven by bag)
  ros2 bag play ~/Desktop/ros_bag_record/warehouse_clean_sensor_data --clock \
    --topics /clock /joint_states /imu/data /scan \
             /offset_tricycle_controller/odom \
             /offset_tricycle_controller/cmd_vel \
             /model/forklift/odometry

  # Terminal 3: record odometry comparison (optional)
  ros2 run offset_tricycle_robot compare_odometry.py record \\
      --output_dir ./results/noise_test --evaluate

Launch arguments:
  imu_gyro_stddev   (default 0.0)  IMU angular velocity noise [rad/s]
  imu_accel_stddev  (default 0.0)  IMU linear acceleration noise [m/s²]
  scan_stddev       (default 0.0)  LiDAR range noise per beam [m]
  odom_vx_stddev    (default 0.0)  Wheel odom forward velocity noise [m/s]  (encoder/slip)
  odom_vyaw_stddev  (default 0.0)  Wheel odom yaw rate noise [rad/s]        (steering encoder)
  rvizconfig        (default gz_sim.rviz from package)
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('offset_tricycle_robot')
    rviz_config_path = os.path.join(pkg_path, 'config', 'gz_sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── Robot description ────────────────────────────────────────────────────
    xacro_file = os.path.join(pkg_path, 'urdf', 'forklift.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time,
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ── Noise injector ───────────────────────────────────────────────────────
    # Reads clean /imu/data and /scan from bag, publishes /imu/data/noisy and /scan/noisy.
    # stddev=0.0 → noise-free pass-through (baseline experiment).
    noise_injector_node = Node(
        package='offset_tricycle_robot',
        executable='noise_injector.py',
        name='noise_injector',
        output='screen',
        parameters=[{
            'use_sim_time':            use_sim_time,
            'imu_gyro_noise_stddev':   LaunchConfiguration('imu_gyro_stddev'),
            'imu_accel_noise_stddev':  LaunchConfiguration('imu_accel_stddev'),
            'scan_range_noise_stddev': LaunchConfiguration('scan_stddev'),
            'odom_vx_stddev':          LaunchConfiguration('odom_vx_stddev'),
            'odom_vyaw_stddev':        LaunchConfiguration('odom_vyaw_stddev'),
        }],
    )

    # ── LaserScan (noisy) → PointCloud2 for KISS-ICP ────────────────────────
    laser_to_pointcloud_node = Node(
        package='offset_tricycle_robot',
        executable='laser_scan_to_pointcloud.py',
        name='laser_scan_to_pointcloud',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic':   '/scan/noisy',
            'output_topic': '/lidar/points',
        }],
    )

    # ── KISS-ICP (uses /lidar/points derived from /scan/noisy) ──────────────
    kiss_icp_launch_args = {
        'topic':            '/lidar/points',
        'base_frame':       'base_link',
        'lidar_odom_frame': 'odom',
        'publish_odom_tf':  'false',
        'visualize':        'false',
        'use_sim_time':     use_sim_time,
    }
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('kiss_icp'),
                'launch', 'odometry.launch.py',
            )
        ]),
        launch_arguments=kiss_icp_launch_args.items(),
    )
    delayed_kiss_icp = TimerAction(period=3.0, actions=[kiss_icp_launch])

    # ── Kinematic-ICP (uses /scan/noisy directly) ────────────────────────────
    kinematic_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('kinematic_icp'),
                'launch', 'online_node.launch.py',
            )
        ]),
        launch_arguments={
            'lidar_topic':      '/scan/noisy',
            'use_2d_lidar':     'true',
            'base_frame':       'base_link',
            'lidar_odom_frame': 'odom',
            'wheel_odom_frame': 'odom_wheel_noisy',
            'publish_odom_tf':  'true',
            'invert_odom_tf':   'true',
            'tf_timeout':       '0.5',
            'visualize':        'false',
            'use_sim_time':     use_sim_time,
        }.items(),
    )
    delayed_kinematic_icp = TimerAction(period=3.0, actions=[kinematic_icp_launch])

    # ── RViz ─────────────────────────────────────────────────────────────────
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use /clock from bag replay',
        ),
        DeclareLaunchArgument(
            'imu_gyro_stddev', default_value='0.0',
            description='IMU angular velocity noise stddev [rad/s]',
        ),
        DeclareLaunchArgument(
            'imu_accel_stddev', default_value='0.0',
            description='IMU linear acceleration noise stddev [m/s²]',
        ),
        DeclareLaunchArgument(
            'scan_stddev', default_value='0.0',
            description='LiDAR range noise stddev per beam [m]',
        ),
        DeclareLaunchArgument(
            'odom_vx_stddev', default_value='0.0',
            description='Wheel odom forward velocity noise stddev [m/s] (models traction encoder error / slip)',
        ),
        DeclareLaunchArgument(
            'odom_vyaw_stddev', default_value='0.0',
            description='Wheel odom yaw rate noise stddev [rad/s] (models steering encoder error)',
        ),
        DeclareLaunchArgument(
            'rvizconfig', default_value=rviz_config_path,
            description='Absolute path to RViz config file',
        ),
        node_robot_state_publisher,
        noise_injector_node,
        laser_to_pointcloud_node,
        delayed_kiss_icp,
        delayed_kinematic_icp,
        rviz2_node,
    ])
