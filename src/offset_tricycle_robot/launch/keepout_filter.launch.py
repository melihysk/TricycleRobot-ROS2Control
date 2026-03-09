import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('offset_tricycle_robot')
    default_keepout_mask = os.path.join(pkg_path, 'maps', 'keepout', 'mask.yaml')
    nav2_params_path = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    keepout_mask = LaunchConfiguration('keepout_mask')

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params_path, {'use_sim_time': use_sim_time}],
    )

    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params_path, {'use_sim_time': use_sim_time,
                                        'yaml_filename': keepout_mask}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        emulate_tty=True,
        parameters=[{'autostart': True,
                     'use_sim_time': use_sim_time,
                     'node_names': ['costmap_filter_info_server',
                                    'filter_mask_server']}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use sim time if true',
        ),
        DeclareLaunchArgument(
            name='keepout_mask',
            default_value=default_keepout_mask,
            description='Full path to keepout filter mask yaml file',
        ),
        costmap_filter_info_server,
        filter_mask_server,
        lifecycle_manager,
    ])
