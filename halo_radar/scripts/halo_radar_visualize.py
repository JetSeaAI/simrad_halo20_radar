#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from marine_sensor_msgs.msg import RadarSector
from halo_radar.radar_interface import RadarInterface, Sector
from cv_bridge import CvBridge
import cv2
import numpy as np


class RadarVisualizeNode(Node):
    def __init__(self):
        super().__init__('radar_visualize_node')
        self.subscription = self.create_subscription(
            RadarSector,
            '/HaloA/data',
            self.radar_echo_data_callback,
            10)
        self.image_publisher = self.create_publisher(Image, '/radar_image', 10)
        self.bridge = CvBridge()
        self.radar_interface = RadarInterface()
        self.sector = Sector()
        self.image = Image()
        self.image.header.frame_id = 'sonar'
        self.image.encoding = 'mono8'
        self.image.is_bigendian = 0
        self.image.step = self.image.width = self.image.height = 1024
        self.image.data = [0 for _ in range(1024 * 1024)]
        self.timer = None
        self.angle_increment = None
        self.offset=2 * np.pi
        
        
        # Configure sector
        self.sector.configure(1024, 1024 // 2)

    def radar_echo_data_callback(self, msg):
        # Process RadarSector message and convert to image
        angle_start = msg.angle_start
        angle_increment = msg.angle_increment
        if self.angle_increment is None:
            self.radar_interface.configureAngles(360, angle_increment, False)
        range_min = msg.range_min
        range_max = msg.range_max
        intensities = msg.intensities
    

        # Convert radar data to image
        for i, intensitie in enumerate(intensities):
            angle = angle_start + i * angle_increment
            if angle > np.pi:
                angle -= self.offset
            self.get_logger().info(f"angle={angle} length of intensities.echoes: {len(intensitie.echoes)}")
            # self.get_logger().info(f"intensitie.echoes: {intensitie.echoes}")
            self.refreshImage(angle, intensitie.echoes)

                
        self.publishImage()

    def refreshImage(self,angle,intensities_echoes):
        result= self.radar_interface.updateAngle(self.radar_interface.rad2grad(angle))
        half_size = 1024//2
        self.sector.init(self.radar_interface.currentAngle(), self.radar_interface.angleStep())
        length = len(intensities_echoes)
        x = 0
        y = 0
        while True:
            more_points, x, y, index = self.sector.nextPoint(x, y)
            if index < length:
                #for mapping the radar data to the image
                #the data values are mapping to 0~255, beacuse the image data is 8 bit,but the radar data range is 0~1
                self.image.data[half_size-y + self.image.step*(half_size-x)] = int(intensities_echoes[index]*255)
            
            if not more_points:
                break
        
       
    def publishImage(self):
        self.image.header.stamp = self.get_clock().now().to_msg()
        self.image_publisher.publish(self.image)

def main(args=None):
    rclpy.init(args=args)
    radar_visualize_node = RadarVisualizeNode()
    
    while rclpy.ok():
        rclpy.spin(radar_visualize_node)

    radar_visualize_node.destroy_node()

if __name__ == '__main__':
    main()