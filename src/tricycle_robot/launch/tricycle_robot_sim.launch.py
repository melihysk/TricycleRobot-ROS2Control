import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('tricycle_robot')
    rviz_config_path = os.path.join(pkg_path, 'config', 'gz_sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description (xacro) - gz_rsp mantığı burada
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

    # ROS-Gazebo bridge for clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
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
    tricycle_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tricycle_controller'],
        output='screen',
    )

    # Controller'ları spawn sonrası başlat (delay ile)
    delayed_controller_spawner = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner, tricycle_controller_spawner],
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
        bridge,
        delayed_controller_spawner,
        rviz2_node,
    ])
