#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from marine_sensor_msgs.msg import RadarSector
from halo_radar_visualize.radar_interface import RadarInterface, Sector
import numpy as np
import time


class RadarVisualizeNode(Node):
    def __init__(self):
        super().__init__('radar_visualize_node')
        self.subscription = self.create_subscription(
            RadarSector,
            '/HaloA/data',
            self.radar_echo_data_callback,
            10)
        self.image_publisher = self.create_publisher(Image, '/radar_image', 10)

        self.radar_interface = RadarInterface()
        self.sector = Sector()
        self.image = Image()
        self.image.header.frame_id = 'radar'
        self.image.encoding = 'mono8'
        self.image.is_bigendian = 0
        echo_length = 1024
        self.image.step = self.image.width = self.image.height = echo_length
        self.image.data = [0 for _ in range(echo_length * echo_length)]
        self.timer = None
        self.angle_increment = None
        self.offset = 2 * np.pi
        self.previous_angle = 0.0
        self.half_size = echo_length // 2

          # Configure sector
        self.sector.configure(echo_length, 1024 // 2)

    def radar_echo_data_callback(self, msg):
        start_time = time.time() # For performance testing
        # Process RadarSector message and convert to image
        angle_start = msg.angle_start
        angle_increment = msg.angle_increment
        if self.angle_increment != angle_increment:
            self.radar_interface.configureAngles(360, angle_increment)
            self.angle_increment = angle_increment
        intensities = msg.intensities

        angle_start -= self.offset

        if angle_start - self.previous_angle > 0.001 and not angle_start < 0.05:
            self.get_logger().warn(f"Angle Jump Detected. angle_start: {angle_start}, previous_angle: {self.previous_angle}")
        # Convert radar data to image
        for i, intensitie in enumerate(intensities):
            angle = angle_start + i * angle_increment
            self.refreshImage(angle, intensitie.echoes)
        
        self.previous_angle = angle
        self.publishImage()
        end_time = time.time()
        self.get_logger().info(f"radar_echo_data_callback took {end_time - start_time:.4f} seconds")

    def refreshImage(self, angle, intensities_echoes):
        self.radar_interface.updateAngle(self.radar_interface.rad2grad(angle))
        self.sector.init(self.radar_interface.currentAngle(), self.radar_interface.angleStep())
        length = len(intensities_echoes)
        intensities_echoes = (np.array(intensities_echoes) * 255).astype(np.uint8) #speed up the mapping process
        x = 0
        y = 0
       
        while True:
            more_points, x, y, index = self.sector.nextPoint(x, y)
            if index < length:
                # TODO: Maybe we can speed up this process by using numpy, it take a lot of time to update the image
                self.image.data[self.half_size - y + self.image.step * (self.half_size - x)] = intensities_echoes[index]
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