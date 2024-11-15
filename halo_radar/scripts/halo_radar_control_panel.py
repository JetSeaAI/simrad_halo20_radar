#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from marine_radar_control_msgs.msg import RadarControlValue
from PyQt5 import QtWidgets, QtCore
import signal

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

        self.range_button = QtWidgets.QPushButton('Set Range')
        self.range_button.clicked.connect(self.set_range)
        layout.addWidget(self.range_button)

        self.gain_button = QtWidgets.QPushButton('Set Gain')
        self.gain_button.clicked.connect(self.set_gain)
        layout.addWidget(self.gain_button)

        self.mode_label = QtWidgets.QLabel('Mode')
        layout.addWidget(self.mode_label)
        self.mode_combobox = QtWidgets.QComboBox()
        self.mode_combobox.addItems(['custom', 'harbor', 'offshore', 'weather', 'bird'])
        self.mode_combobox.currentIndexChanged.connect(self.set_mode)
        layout.addWidget(self.mode_combobox)

        self.sea_clutter_button = QtWidgets.QPushButton('Set Sea Clutter')
        self.sea_clutter_button.clicked.connect(self.set_sea_clutter)
        layout.addWidget(self.sea_clutter_button)

        self.auto_sea_clutter_nudge_button = QtWidgets.QPushButton('Set Auto Sea Clutter Nudge')
        self.auto_sea_clutter_nudge_button.clicked.connect(self.set_auto_sea_clutter_nudge)
        layout.addWidget(self.auto_sea_clutter_nudge_button)

        self.sea_state_label = QtWidgets.QLabel('Sea state')
        layout.addWidget(self.sea_state_label)
        self.sea_state_combobox = QtWidgets.QComboBox()
        self.sea_state_combobox.addItems(['calm', 'moderate', 'rough'])
        self.sea_state_combobox.currentIndexChanged.connect(self.set_sea_state)
        layout.addWidget(self.sea_state_combobox)

        self.rain_clutter_button = QtWidgets.QPushButton('Set Rain Clutter')
        self.rain_clutter_button.clicked.connect(self.set_rain_clutter)
        layout.addWidget(self.rain_clutter_button)

        self.noise_rejection_label = QtWidgets.QLabel('Noise rejection')
        layout.addWidget(self.noise_rejection_label)
        self.noise_rejection_combobox = QtWidgets.QComboBox()
        self.noise_rejection_combobox.addItems(['off', 'low', 'medium', 'high'])
        self.noise_rejection_combobox.currentIndexChanged.connect(self.set_noise_rejection)
        layout.addWidget(self.noise_rejection_combobox)

        self.target_expansion_label = QtWidgets.QLabel('Target expansion')
        layout.addWidget(self.target_expansion_label)
        self.target_expansion_combobox = QtWidgets.QComboBox()
        self.target_expansion_combobox.addItems(['off', 'low', 'medium', 'high'])
        self.target_expansion_combobox.currentIndexChanged.connect(self.set_target_expansion)
        layout.addWidget(self.target_expansion_combobox)

        self.interference_rejection_label = QtWidgets.QLabel('Interf. rej')
        layout.addWidget(self.interference_rejection_label)
        self.interference_rejection_combobox = QtWidgets.QComboBox()
        self.interference_rejection_combobox.addItems(['off', 'low', 'medium', 'high'])
        self.interference_rejection_combobox.currentIndexChanged.connect(self.set_interference_rejection)
        layout.addWidget(self.interference_rejection_combobox)

        self.target_separation_label = QtWidgets.QLabel('Target separation')
        layout.addWidget(self.target_separation_label)
        self.target_separation_combobox = QtWidgets.QComboBox()
        self.target_separation_combobox.addItems(['off', 'low', 'medium', 'high'])
        self.target_separation_combobox.currentIndexChanged.connect(self.set_target_separation)
        layout.addWidget(self.target_separation_combobox)

        self.scan_speed_label = QtWidgets.QLabel('Fast scan')
        layout.addWidget(self.scan_speed_label)
        self.scan_speed_combobox = QtWidgets.QComboBox()
        self.scan_speed_combobox.addItems(['off', 'medium', 'high'])
        self.scan_speed_combobox.currentIndexChanged.connect(self.set_scan_speed)
        layout.addWidget(self.scan_speed_combobox)

        self.doppler_mode_label = QtWidgets.QLabel('VelocityTrack')
        layout.addWidget(self.doppler_mode_label)
        self.doppler_mode_combobox = QtWidgets.QComboBox()
        self.doppler_mode_combobox.addItems(['off', 'normal', 'approaching_only'])
        self.doppler_mode_combobox.currentIndexChanged.connect(self.set_doppler_mode)
        layout.addWidget(self.doppler_mode_combobox)

        self.doppler_speed_button = QtWidgets.QPushButton('Set Doppler Speed')
        self.doppler_speed_button.clicked.connect(self.set_doppler_speed)
        layout.addWidget(self.doppler_speed_button)

        self.antenna_height_button = QtWidgets.QPushButton('Set Antenna Height')
        self.antenna_height_button.clicked.connect(self.set_antenna_height)
        layout.addWidget(self.antenna_height_button)

        self.bearing_alignment_button = QtWidgets.QPushButton('Set Bearing Alignment')
        self.bearing_alignment_button.clicked.connect(self.set_bearing_alignment)
        layout.addWidget(self.bearing_alignment_button)

        self.sidelobe_suppression_button = QtWidgets.QPushButton('Set Sidelobe Suppression')
        self.sidelobe_suppression_button.clicked.connect(self.set_sidelobe_suppression)
        layout.addWidget(self.sidelobe_suppression_button)

        self.lights_label = QtWidgets.QLabel('Halo light')
        layout.addWidget(self.lights_label)
        self.lights_combobox = QtWidgets.QComboBox()
        self.lights_combobox.addItems(['off', 'low', 'medium', 'high'])
        self.lights_combobox.currentIndexChanged.connect(self.set_lights)
        layout.addWidget(self.lights_combobox)

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
        self.radar_node.get_logger().info('Setting range...')
        command = RadarControlValue()
        command.key = 'range'
        command.value = '50'  # Example value
        self.radar_node.command_publisher.publish(command)

    def set_gain(self):
        self.radar_node.get_logger().info('Setting gain...')
        command = RadarControlValue()
        command.key = 'gain'
        command.value = '35.686275'  # Example value
        self.radar_node.command_publisher.publish(command)

    def set_mode(self):
        self.radar_node.get_logger().info('Setting mode...')
        command = RadarControlValue()
        command.key = 'mode'
        command.value = self.mode_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_sea_clutter(self):
        self.radar_node.get_logger().info('Setting sea clutter...')
        command = RadarControlValue()
        command.key = 'sea_clutter'
        command.value = 'auto'  # Example value
        self.radar_node.command_publisher.publish(command)

    def set_auto_sea_clutter_nudge(self):
        self.radar_node.get_logger().info('Setting auto sea clutter nudge...')
        command = RadarControlValue()
        command.key = 'auto_sea_clutter_nudge'
        command.value = '0'  # Example value
        self.radar_node.command_publisher.publish(command)

    def set_sea_state(self):
        self.radar_node.get_logger().info('Setting sea state...')
        command = RadarControlValue()
        command.key = 'sea_state'
        command.value = self.sea_state_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_rain_clutter(self):
        self.radar_node.get_logger().info('Setting rain clutter...')
        command = RadarControlValue()
        command.key = 'rain_clutter'
        command.value = '0.784314'  # Example value
        self.radar_node.command_publisher.publish(command)

    def set_noise_rejection(self):
        self.radar_node.get_logger().info('Setting noise rejection...')
        command = RadarControlValue()
        command.key = 'noise_rejection'
        command.value = self.noise_rejection_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_target_expansion(self):
        self.radar_node.get_logger().info('Setting target expansion...')
        command = RadarControlValue()
        command.key = 'target_expansion'
        command.value = self.target_expansion_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_interference_rejection(self):
        self.radar_node.get_logger().info('Setting interference rejection...')
        command = RadarControlValue()
        command.key = 'interference_rejection'
        command.value = self.interference_rejection_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_target_separation(self):
        self.radar_node.get_logger().info('Setting target separation...')
        command = RadarControlValue()
        command.key = 'target_separation'
        command.value = self.target_separation_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_scan_speed(self):
        self.radar_node.get_logger().info('Setting scan speed...')
        command = RadarControlValue()
        command.key = 'scan_speed'
        command.value = self.scan_speed_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_doppler_mode(self):
        self.radar_node.get_logger().info('Setting doppler mode...')
        command = RadarControlValue()
        command.key = 'doppler_mode'
        command.value = self.doppler_mode_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_doppler_speed(self):
        self.radar_node.get_logger().info('Setting doppler speed...')
        command = RadarControlValue()
        command.key = 'doppler_speed'
        command.value = '2.000000'  # Example value
        self.radar_node.command_publisher.publish(command)

    def set_antenna_height(self):
        self.radar_node.get_logger().info('Setting antenna height...')
        command = RadarControlValue()
        command.key = 'antenna_height'
        command.value = '4.000000'  # Example value
        self.radar_node.command_publisher.publish(command)

    def set_bearing_alignment(self):
        self.radar_node.get_logger().info('Setting bearing alignment...')
        command = RadarControlValue()
        command.key = 'bearing_alignment'
        command.value = '0.000000'  # Example value
        self.radar_node.command_publisher.publish(command)

    def set_sidelobe_suppression(self):
        self.radar_node.get_logger().info('Setting sidelobe suppression...')
        command = RadarControlValue()
        command.key = 'sidelobe_suppression'
        command.value = 'auto'  # Example value
        self.radar_node.command_publisher.publish(command)

    def set_lights(self):
        self.radar_node.get_logger().info('Setting lights...')
        command = RadarControlValue()
        command.key = 'lights'
        command.value = self.lights_combobox.currentText()
        self.radar_node.command_publisher.publish(command)
    
    def closeEvent(self, event):
        self.radar_node.get_logger().info('Closed by user')
        self.radar_node.destroy_node()
        event.accept()
        raise SystemExit
        

def main(args=None):
    rclpy.init(args=args)
    radar_node = Node('radar_control_panel')
    radar_node.command_publisher = radar_node.create_publisher(RadarControlValue, '/HaloA/change_state', 10)
    app = QtWidgets.QApplication([])
    gui = RadarConfigGUI(radar_node)
    gui.show()

    while 1:
        try:
            app.processEvents()
            rclpy.spin_once(radar_node, timeout_sec=0.1)
        except SystemExit:
            rclpy.get_logger().info('Closed by user')

    app.closeAllWindows()
    radar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
