import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _box_sdf(r: float, g: float, b: float) -> str:
    return f"""
<sdf version='1.9'>
  <model name='box_marker'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry>
          <box>
            <size>0.35 0.35 0.35</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
      <collision name='collision'>
        <geometry>
          <box>
            <size>0.35 0.35 0.35</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def _spawn_box(name: str, x: float, y: float, r: float, g: float, b: float) -> Node:
    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', name,
            '-string', _box_sdf(r, g, b),
            '-x', str(x),
            '-y', str(y),
            '-z', '0.175',
        ],
        output='screen',
    )


def generate_launch_description():
    pkg_path = get_package_share_directory('offset_tricycle_robot')
    rviz_config_path = os.path.join(pkg_path, 'config', 'gz_sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description (xacro)
    xacro_file = os.path.join(pkg_path, 'gazebo_sim_model', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Gazebo Sim (gz sim) başlatma
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Robot spawn - ros_gz_sim kullanarak
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'gebot',
            '-z', '0.5',
        ],
        output='screen',
    )

    # Static colored reference boxes for odometry checks
    spawn_x_plus = _spawn_box('marker_x_plus_7', 7.0, 0.0, 1.0, 0.0, 0.0)     # red
    spawn_x_minus = _spawn_box('marker_x_minus_7', -7.0, 0.0, 0.0, 1.0, 0.0)  # green
    spawn_y_plus = _spawn_box('marker_y_plus_7', 0.0, 7.0, 0.0, 0.0, 1.0)     # blue
    spawn_y_minus = _spawn_box('marker_y_minus_7', 0.0, -7.0, 1.0, 1.0, 0.0)  # yellow

    delayed_box_spawners = TimerAction(
        period=2.0,
        actions=[spawn_x_plus, spawn_x_minus, spawn_y_plus, spawn_y_minus],
    )

    # ROS-Gazebo bridge for clock, IMU, and ground truth odometry
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/gebot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # Tricycle Controller - cmd_vel ile kontrol için
    offset_tricycle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['offset_tricycle_controller'],
        output='screen',
    )

    # Controller'ları spawn sonrası başlat (delay ile)
    delayed_controller_spawner = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, offset_tricycle_controller_spawner],
    )

    # EKF node for sensor fusion (wheel odometry + IMU)
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )

    # Delay EKF start to ensure controller and IMU are ready
    delayed_ekf = TimerAction(
        period=7.0,
        actions=[ekf_node],
    )

    # RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use sim time if true',
        ),
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=rviz_config_path,
            description='Absolute path to rviz config file',
        ),
        node_robot_state_publisher,
        gazebo_sim,
        spawn_entity,
        #delayed_box_spawners,
        bridge,
        delayed_controller_spawner,
        delayed_ekf,
        rviz2_node,
    ])
