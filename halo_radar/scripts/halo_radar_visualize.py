#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from marine_sensor_msgs.msg import RadarSector
from cv_bridge import CvBridge
import cv2
import numpy as np

class RadarVisualizeNode(Node):
    def __init__(self):
        super().__init__('radar_visualize_node')
        self.subscription = self.create_subscription(
            RadarSector,
            '/HaloA/data',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/radar_image', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Process RadarSector message and convert to image
        angle_start = msg.angle_start
        angle_increment = msg.angle_increment
        range_min = msg.range_min
        range_max = msg.range_max
        intensities = msg.intensities

        # Create a blank image
        image_size = 2000
        radar_image = np.zeros((image_size, image_size), dtype=np.uint8)

        # Convert radar data to image
        for i, intensitie in enumerate(intensities):
            angle = angle_start + i * angle_increment
            self.get_logger().info(f"angle={angle} length of intensities.echoes: {len(intensitie.echoes)}")
            self.get_logger().info(f"intensitie.echoes: {intensitie.echoes}")
            # for j, intensity in enumerate(intensitie.echoes):
                

        # Convert the image to ROS Image message and publish
        # image_msg = self.bridge.cv2_to_imgmsg(radar_image, encoding="mono8")
        # self.publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    radar_visualize_node = RadarVisualizeNode()
    rclpy.spin(radar_visualize_node)
    radar_visualize_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()