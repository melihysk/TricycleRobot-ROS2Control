import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('offset_tricycle_robot')
    rviz_config_path = os.path.join(pkg_path, 'config', 'gz_sim.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot description (xacro)
    xacro_file = os.path.join(pkg_path, 'urdf', 'forklift.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Warehouse world path
    world_file = os.path.join(pkg_path, 'worlds', 'warehouse.sdf')
    worlds_dir = os.path.join(pkg_path, 'worlds')

    # GZ_SIM_RESOURCE_PATH: Gazebo'nun lokal modelleri (Warehouse) bulabilmesi için
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=worlds_dir,
    )

    # Gazebo Sim (gz sim) başlatma
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Robot spawn - ros_gz_sim kullanarak
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'forklift',
            '-z', '0.5',
        ],
        output='screen',
    )

    # ROS-Gazebo bridge for clock, IMU, and ground truth odometry
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/forklift/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
    )

    # 2D LaserScan -> 3D PointCloud2 (KISS-ICP için)
    laser_to_pointcloud_node = Node(
        package='offset_tricycle_robot',
        executable='laser_scan_to_pointcloud.py',
        name='laser_scan_to_pointcloud',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
            ('lidar/points', '/lidar/points'),
        ],
    )

    # KISS-ICP LiDAR odometry (3D point cloud ile; 2D scan dönüştürülmüş nokta bulutu kullanır)
    kiss_icp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('kiss_icp'), 'launch', 'odometry.launch.py'),
        ]),
        launch_arguments={
            'topic': '/lidar/points',
            'base_frame': 'base_link',
            'lidar_odom_frame': 'odom',
            'publish_odom_tf': 'true',
            'visualize': 'false',
            'use_sim_time': use_sim_time,
        }.items(),
    )
    delayed_kiss_icp = TimerAction(period=8.0, actions=[kiss_icp_launch])

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

    rf2o_laser_odometry_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : False,
            'base_frame_id' : 'base_link',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 10.0}],
    )

    # RF2O needs /scan; start after bridge and robot are up
    delayed_rf2o = TimerAction(period=10.0, actions=[rf2o_laser_odometry_node])

    laser_scan_matcher_odometry_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[{
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'laser_frame': 'lidar_link',
                'publish_odom': 'odom_scan_matcher',
                'publish_tf': False,
            }],
    )

    # RF2O needs /scan; start after bridge and robot are up
    delayed_scan_matcher = TimerAction(period=10.0, actions=[laser_scan_matcher_odometry_node])

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
        gz_resource_path,
        node_robot_state_publisher,
        gazebo_sim,
        spawn_entity,
        bridge,
        laser_to_pointcloud_node,
        delayed_controller_spawner,
        delayed_ekf,
        delayed_kiss_icp,
        rviz2_node,
        delayed_rf2o,
        delayed_scan_matcher,
    ])
