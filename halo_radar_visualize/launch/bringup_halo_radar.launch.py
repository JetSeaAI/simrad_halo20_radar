from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='halo_radar',
            executable='halo_radar',
            name='halo_radar_node',
            output='screen',
            parameters=[
                {'range_correction_factor': 1.024},
                {'frame_id': 'radar'}
            ]
        ),
        Node(
            package='halo_radar_visualize',
            executable='halo_radar_visualize',
            name='radar_visualize_node',
            output='screen'
        ),
        Node(
            package='halo_radar_visualize',
            executable='halo_radar_control_panel',
            name='radar_control_panel',
            output='screen'
        )
    ])
