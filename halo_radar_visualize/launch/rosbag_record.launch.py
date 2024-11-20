import launch
import time

def generate_launch_description():
    topics_to_record = [
        '/HaloA/change_state',
        '/HaloA/data',
        '/HaloA/state',
        '/parameter_events',
        '/radar_image',
        '/radar_pointcloud',
        '/rosout'
    ]

    timestamp = time.strftime("%Y%m%d-%H%M%S")
    bag_name = f'recorded_rosbag_{timestamp}'
    
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record'] + topics_to_record + ['-o', f'./share/rosbag_record/{bag_name}'],
            output='screen'
        )
    ])