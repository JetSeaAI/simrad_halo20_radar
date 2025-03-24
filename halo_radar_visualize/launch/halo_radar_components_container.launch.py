import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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

    return launch.LaunchDescription([container])
