#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from marine_sensor_msgs.msg import RadarSector
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class HaloRadarDataCropper(Node):
    def __init__(self):
        super().__init__('halo_radar_data_cropper')

        parameters = {
            'input_pointcloud_topic': 'merged_pointcloud',
            'cropped_pointcloud_topic': 'cropped_pointcloud',
            'cropped_angle_start': -120,
            'cropped_angle_end': 120,
            'cropped_distance_start': 20,
            'cropped_distance_end': 120,
        }

        for name, value in parameters.items():
            self.declare_parameter(name, value)

        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("input_pointcloud_topic").value,
            self.listener_callback,
            10)
        
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter("cropped_pointcloud_topic").value,
            10)
        
        self.angle_start = self.get_parameter("cropped_angle_start").value
        self.angle_end = self.get_parameter("cropped_angle_end").value
        self.distance_start = self.get_parameter("cropped_distance_start").value
        self.distance_end = self.get_parameter("cropped_distance_end").value

        self.full_stack_pointcloud = []

    def listener_callback(self, msg):
        full_pointcloud = np.array(list(pc2.read_points(msg, skip_nans=False)))

        # Filter points within the angle range of -120 degrees to 120 degrees
        x = full_pointcloud['x']
        y = full_pointcloud['y']
        z = full_pointcloud['z']
        distances = np.sqrt(x**2 + y**2 + z**2)
        angles = np.degrees(np.arctan2(y, x))
        angle_mask = (angles >= self.angle_start) & (angles <= self.angle_end)
        full_pointcloud = full_pointcloud[angle_mask]
        distances = distances[angle_mask]

        # Filter points within the distance range of 20 meters to 120 meters
        distance_mask = (distances >= self.distance_start) & (distances <= self.distance_end)
        full_pointcloud = full_pointcloud[distance_mask]

        header = msg.header
        full_pointcloud_msg = pc2.create_cloud(header, msg.fields, full_pointcloud)
        self.pointcloud_publisher.publish(full_pointcloud_msg)
        self.full_stack_pointcloud = []  # Clear the data for the next stack

def main(args=None):
    rclpy.init(args=args)
    cropper = HaloRadarDataCropper()
    rclpy.spin(cropper)
    cropper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
