#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class HaloRadarDataCropper(Node):
    def __init__(self):
        super().__init__('halo_radar_data_cropper')
        self.subscription = self.create_subscription(
            PointCloud2,
            'radar_pointcloud',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, 'cropped_pointcloud', 10)
        self.full_stack_pointcloud = []

    def listener_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, skip_nans=False)))
        if points.size == 0:
            self.get_logger().error("No points found in the PointCloud2 message.")
            return
      
        self.full_stack_pointcloud.append(points)
        x = points['x']
        y = points['y']
        # z = points['z']
        angles = np.degrees(np.arctan2(y, x))
        # Publish the full stacked point cloud data when the angle reaches -180 degrees
        if np.any(angles == -180):
            full_pointcloud = np.concatenate(self.full_stack_pointcloud, axis=0)

            # Filter points within the angle range of -120 degrees to 120 degrees
            x = full_pointcloud['x']
            y = full_pointcloud['y']
            z = full_pointcloud['z']
            distances = np.sqrt(x**2 + y**2 + z**2)
            angles = np.degrees(np.arctan2(y, x))
            angle_mask = (angles >= -120) & (angles <= 120)
            full_pointcloud = full_pointcloud[angle_mask]
            distances = distances[angle_mask]

            # Filter points within the distance range of 20 meters to 120 meters
            distance_mask = (distances >= 20) & (distances <= 120)
            full_pointcloud = full_pointcloud[distance_mask]

            header = msg.header
            full_pointcloud_msg = pc2.create_cloud(header, msg.fields, full_pointcloud)
            self.publisher.publish(full_pointcloud_msg)
            self.full_stack_pointcloud = []  # Clear the data for the next stack

def main(args=None):
    rclpy.init(args=args)
    cropper = HaloRadarDataCropper()
    rclpy.spin(cropper)
    cropper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
