from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag_file')],
            output='screen'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/radar/cropped_pointcloud'),
                        ('scan', '/radar/cropped_scan')],
            parameters=[{
                'target_frame': 'radar',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -2.0943951024,  # -M_PI/2
                'angle_max': 2.0943951024,  # M_PI/2
                'angle_increment': 0.003,  # M_PI/360.0
                'scan_time': 1.1,
                'range_min': 20.0,
                'range_max': 120.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='halo_radar_visualize',
            executable='halo_radar_data_cropper',
            name='halo_radar_data_cropper',
            output='screen'
        ),
        # Node(
        #     package='halo_radar_visualize',
        #     executable='halo_radar_merge_scan',
        #     name='halo_radar_merge_scan',
        #     output='screen'
        # )
    ])
