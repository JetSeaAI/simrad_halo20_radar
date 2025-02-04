import launch
import subprocess
import time

def generate_launch_description():

    project_name = 'halo'
    
    topics_to_record = [
        '/HaloA/change_state',
        '/HaloA/data',
        '/HaloA/state',
        '/parameter_events',
        '/halo_radar/single_shot_radar_pointcloud',
        '/halo_radar/merged_pointcloud',
        '/halo_radar/cropped_pointcloud',
        '/halo_radar/cropped_scan',
        '/rosout'
    ]

    timestamp = time.strftime("%Y%m%d-%H%M%S")
    bag_dir = f'./share/rosbag_record/recorded_rosbag_{project_name}_{timestamp}'
    duration = 900  # 15 minutes

    def record_rosbag():
        part=0
        while True:
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            bag_name = f'{timestamp}_{project_name}_part{part}'
            cmd = ['ros2', 'bag', 'record', '-o', f'{bag_dir}/{bag_name}'] + topics_to_record
            process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=None, stderr=None)
            part += 1
            time.sleep(duration)
            process.terminate()
            process.wait()

    return launch.LaunchDescription([
        launch.actions.OpaqueFunction(function=lambda context: record_rosbag())
    ])