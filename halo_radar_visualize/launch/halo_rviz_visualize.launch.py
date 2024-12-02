import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz2_config = os.path.join(
        get_package_share_directory("halo_radar_visualize"), "rviz2", "visualize.rviz"
    )
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz2_config],
            )
        ]
    )
