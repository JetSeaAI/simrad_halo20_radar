#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from marine_sensor_msgs.msg import RadarSector
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField



class RadarVisualizeNode(Node):
    def __init__(self):
        super().__init__('radar_visualize_node')
        self.subscription = self.create_subscription(
            RadarSector,
            '/HaloA/data',
            self.radar_echo_data_callback,
            10)

        self.timer = None
        self.angle_increment = None
        self.offset = 2 * np.pi
        self.previous_angle = 0.0
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'radar_pointcloud', 10)

        # Initialize static fields for PointCloud2 message
        self.pointcloud = PointCloud2()
        self.pointcloud.header.frame_id = 'radar'
        self.pointcloud.height = 1
        self.pointcloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.pointcloud.is_bigendian = False
        self.pointcloud.point_step = 16
        self.pointcloud.is_dense = True
      
        self.publish_count = 0
        self.timer = self.create_timer(5.0, self.report_publish_rate)

    def radar_echo_data_callback(self, msg):
        # start_time = time.time() # For performance testing
        # Process RadarSector message and convert to image
        angle_start = msg.angle_start
        angle_increment = msg.angle_increment
        if self.angle_increment != angle_increment:
            self.radar_interface.configureAngles(360, angle_increment)
            self.angle_increment = angle_increment
        range_min = msg.range_min
        range_max = msg.range_max
        intensities = msg.intensities
        points = []
        angle_start -= self.offset

        if angle_start - self.previous_angle > 0.001 and not angle_start < 0.05:
            self.get_logger().warn(f"Angle Jump Detected. angle_start: {angle_start}, previous_angle: {self.previous_angle}")
        # Convert radar data to image and generate point cloud
        for i, intensitie in enumerate(intensities):
            angle = angle_start + i * angle_increment
            # self.refreshImage(angle, intensitie.echoes)
            points.extend(self.generate_points(angle, intensitie.echoes, range_min, range_max))
        
        self.previous_angle = angle
        # self.publishImage()
        self.publishPointCloud(points)
        self.publish_count += 1
        # end_time = time.time()
        # self.get_logger().info(f"radar_echo_data_callback took {end_time - start_time:.4f} seconds")


    def generate_points(self, angle, intensities_echoes, range_min, range_max):
        points = []
        intensities_echoes = np.array(intensities_echoes)
        valid_indices = np.where(intensities_echoes > 0)[0]
        r = range_min + valid_indices * (range_max - range_min) / len(intensities_echoes)
        
        # Filter out points less than 3m that relfects the radar it self
        valid_indices = valid_indices[r >= 3]
        r = r[r >= 3]
        
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        z = np.zeros_like(x)
        intensities = intensities_echoes[valid_indices]
        points = np.column_stack((x, y, z, intensities)).tolist()
        return points
   
    def publishPointCloud(self, points):
        # start_time = time.time()
        
        self.pointcloud.header.stamp = self.get_clock().now().to_msg()
        self.pointcloud.width = len(points)

        # Use numpy for faster packing
        points_array = np.array(points, dtype=np.float32)
        self.pointcloud.data = points_array.tobytes()

        self.pointcloud_publisher.publish(self.pointcloud)
        
        # end_time = time.time()
        # self.get_logger().info(f"publishPointCloud took {end_time - start_time:.4f} seconds")

    def report_publish_rate(self):
        self.get_logger().info(f"Publishing rate: {self.publish_count/5} pointclouds per second")
        self.publish_count = 0

def main(args=None):
    rclpy.init(args=args)
    radar_visualize_node = RadarVisualizeNode()
    
    while rclpy.ok():
        rclpy.spin(radar_visualize_node)

    radar_visualize_node.destroy_node()

if __name__ == '__main__':
    main()