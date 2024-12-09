import launch
import time

def generate_launch_description():
    topics_to_record = [
        '/HaloA/change_state',
        '/HaloA/data',
        '/HaloA/state',
        '/parameter_events',
        '/halo_radar/radar_pointcloud',
        '/halo_radar/cropped_pointcloud',
        '/halo_radar/cropped_scan',
        '/rosout'
    ]

    timestamp = time.strftime("%Y%m%d-%H%M%S")
    bag_name = f'recorded_rosbag_halo_{timestamp}'
    
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] +
            ['-d','900']+ # 15 minutes
              topics_to_record +
                ['-o', f'./share/rosbag_record/{bag_name}'],
            output='screen'
        )
    ])