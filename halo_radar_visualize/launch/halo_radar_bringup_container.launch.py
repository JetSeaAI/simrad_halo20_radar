import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='halo_radar_container',
        namespace='halo_radar',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='halo_radar',
                plugin='RadarVisualizeNode',
                name='radar_visualize',
                namespace='halo_radar',
                parameters=[
                    {'frame_id': 'radar'},
                    {'single_shot_pointcloud_topic': 'single_shot_radar_pointcloud'},
                    {'radar_input_topic': '/HaloA/data'},
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='halo_radar',
                plugin='HaloRadarMergeScan',
                name='radar_merge_scan',
                namespace='halo_radar',
                parameters=[
                    {'merged_pointcloud_topic': 'merged_pointcloud'},
                    {'single_shot_pointcloud_topic': 'single_shot_radar_pointcloud'},
                    {'radar_input_topic': '/HaloA/data'},
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='halo_radar',
                plugin='HaloRadarDataCropper',
                name='radar_data_cropper',
                namespace='halo_radar',
                parameters=[
                    {'input_pointcloud_topic': 'merged_pointcloud'},
                    {'cropped_pointcloud_topic': 'cropped_pointcloud'},
                    {'cropped_angle_start': -120.0},
                    {'cropped_angle_end': 120.0},
                    {'cropped_distance_start': 20.0},
                    {'cropped_distance_end': 120.0},
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='both',
    )

    halo_radar_node = Node(
            package='halo_radar',
            executable='halo_radar',
            name='halo_radar_node',
            namespace='halo_radar',
            output='screen',
            parameters=[
                {'range_correction_factor': 1.024},
                {'frame_id': 'radar'}
            ]
        )

    control_panel_node =Node(
            package='halo_radar_visualize',
            executable='halo_radar_control_panel',
            name='radar_control_panel',
            namespace='halo_radar',
            output='screen'
    )
    laserscan_node= Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            namespace='halo_radar',
            remappings=[('cloud_in', 'cropped_pointcloud'),
                        ('scan', 'cropped_scan')],
            parameters=[{
                # 'target_frame': 'radar',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -2.0943951024,  # -M_PI/2 -120deg
                'angle_max': 2.0943951024,  # M_PI/2 120deg
                'angle_increment': 0.003,  # same as angular resolution of radar
                'scan_time': 1.1,
                'range_min': 20.0,
                'range_max': 120.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )

    return launch.LaunchDescription([container, control_panel_node,laserscan_node, halo_radar_node])