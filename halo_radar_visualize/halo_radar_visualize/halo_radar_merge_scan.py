#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from marine_sensor_msgs.msg import RadarSector
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class HaloRadarMergeScan(Node):
    def __init__(self):
        super().__init__('halo_radar_merge_scan')


        parameters = {
            'single_shot_pointcloud_topic': 'single_shot_radar_pointcloud',
            'radar_input_topic': '/HaloA/data',
            'merged_pointcloud_topic': 'merged_pointcloud'
        }

        for name, value in parameters.items():
            self.declare_parameter(name, value)

        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("single_shot_pointcloud_topic").value,
            self.listener_callback,
            10)
        
        self.halo_subscription = self.create_subscription(
            RadarSector,
            self.get_parameter('radar_input_topic').value,
            self.radar_data_callback,
            10)
            
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter("merged_pointcloud_topic").value,
            10)
        
        self.full_stack_pointcloud = []
        self.previous_angle=0
        self.publish_merged_pointcloud = False

    def radar_data_callback(self, msg):
        angle= msg.angle_start
        if angle>self.previous_angle:
            self.get_logger().info(f"Full Scan. Publish the merged pointcloud")
            self.publish_merged_pointcloud = True
        self.previous_angle=angle

    def listener_callback(self, msg):
        points = np.array(list(pc2.read_points(msg, skip_nans=False)))
        if points.size == 0:
            # self.get_logger().info(f"No points found in the PointCloud2 message.")
            return
        self.full_stack_pointcloud.append(points)
        if self.publish_merged_pointcloud:
            self.merge_pointcloud(msg)
            self.publish_merged_pointcloud = False
        

    def merge_pointcloud(self,msg):
            full_pointcloud = np.concatenate(self.full_stack_pointcloud, axis=0)
            header = msg.header
            full_pointcloud_msg = pc2.create_cloud(header, msg.fields, full_pointcloud)
            self.pointcloud_publisher.publish(full_pointcloud_msg)
            self.full_stack_pointcloud = []  # Clear the data for the next stack

def main(args=None):
    rclpy.init(args=args)
    cropper = HaloRadarMergeScan()
    rclpy.spin(cropper)
    cropper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
