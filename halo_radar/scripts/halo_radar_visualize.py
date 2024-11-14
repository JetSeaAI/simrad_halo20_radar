#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from marine_sensor_msgs.msg import RadarSector
from marine_radar_control_msgs.msg import RadarControlValue
from halo_radar.radar_interface import RadarInterface, Sector
from cv_bridge import CvBridge
import cv2
import numpy as np
from PyQt5 import QtWidgets, QtCore
import signal

class RadarVisualizeNode(Node):
    def __init__(self):
        super().__init__('radar_visualize_node')
        self.subscription = self.create_subscription(
            RadarSector,
            '/HaloA/data',
            self.listener_callback,
            10)
        self.image_publisher = self.create_publisher(Image, '/radar_image', 10)
        self.command_publisher = self.create_publisher(RadarControlValue, '/HaloA/change_state', 10)
        
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

    def listener_callback(self, msg):
        # Process RadarSector message and convert to image
        angle_start = msg.angle_start
        angle_increment = msg.angle_increment
        if self.angle_increment is None:
            self.radar_interface.configureAngles(360, angle_increment, False)
        range_min = msg.range_min
        range_max = msg.range_max
        intensities = msg.intensities
    
        # Create a blank image
        # image_size = 2000
        # radar_image = np.zeros((image_size, image_size), dtype=np.uint8)

        # Convert radar data to image
        for i, intensitie in enumerate(intensities):
            angle = angle_start + i * angle_increment
            if angle > np.pi:
                angle -= self.offset
            self.get_logger().info(f"angle={angle} length of intensities.echoes: {len(intensitie.echoes)}")
            # self.get_logger().info(f"intensitie.echoes: {intensitie.echoes}")
            self.refreshImage(angle, intensitie.echoes)
            # for j, intensity in enumerate(intensitie.echoes):
                
        self.publishImage()
        # Convert the image to ROS Image message and publish
        # image_msg = self.bridge.cv2_to_imgmsg(radar_image, encoding="mono8")
        # self.publisher.publish(image_msg)
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
                self.image.data[half_size-y + self.image.step*(half_size-x)] = int(intensities_echoes[index]*255)
            
            if not more_points:
                break
        
       
    def publishImage(self):
        self.image.header.stamp = self.get_clock().now().to_msg()
        self.image_publisher.publish(self.image)

class RadarConfigGUI(QtWidgets.QWidget):
    def __init__(self, radar_node):
        super().__init__()
        self.radar_node = radar_node
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Radar Configuration')
        layout = QtWidgets.QVBoxLayout()

        self.start_button = QtWidgets.QPushButton('Start Radar')
        self.start_button.clicked.connect(self.start_radar)
        layout.addWidget(self.start_button)

        self.stop_button = QtWidgets.QPushButton('Stop Radar')
        self.stop_button.clicked.connect(self.stop_radar)
        layout.addWidget(self.stop_button)

        self.setLayout(layout)

    def start_radar(self):
        self.radar_node.get_logger().info('Starting radar...')
        command = RadarControlValue()
        command.key = 'status'
        command.value = 'transmit'
        self.radar_node.command_publisher.publish(command)


    def stop_radar(self):
        self.radar_node.get_logger().info('Stopping radar...')
        command = RadarControlValue()
        command.key = 'status'
        command.value = 'standby'
        self.radar_node.command_publisher.publish(command)

    def set_range(self):
        # TODO: Implement
        pass

    def set_gain(self):
        # TODO: Implement
        pass

    def set_mode(self):
        # TODO: Implement
        pass

    

def main(args=None):
    rclpy.init(args=args)
    radar_visualize_node = RadarVisualizeNode()

    app = QtWidgets.QApplication([])
    gui = RadarConfigGUI(radar_visualize_node)
    gui.show()


    def on_exit():
        rclpy.shutdown()

    app.aboutToQuit.connect(on_exit)

    def handle_sigint(*args):
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)
    
    while rclpy.ok():
        app.processEvents()
        rclpy.spin_once(radar_visualize_node, timeout_sec=0.1)
    app.exec_()

    radar_visualize_node.destroy_node()

if __name__ == '__main__':
    main()