#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from marine_radar_control_msgs.msg import RadarControlValue,RadarControlSet
from PyQt5 import QtWidgets, QtCore
import signal

class RadarConfigGUI(QtWidgets.QWidget):
    def __init__(self, radar_node):
        super().__init__()
        self.radar_node = radar_node
        self.initUI()
        self.sea_clutter_auto_mode=False

    def initUI(self):
        self.setWindowTitle('Radar Configuration')
        layout = QtWidgets.QVBoxLayout()

        self.start_button = QtWidgets.QPushButton('Start Radar')
        self.start_button.clicked.connect(self.start_radar)
        layout.addWidget(self.start_button)

        self.stop_button = QtWidgets.QPushButton('Stop Radar')
        self.stop_button.clicked.connect(self.stop_radar)
        layout.addWidget(self.stop_button)

        self.range_label = QtWidgets.QLabel('Set Range')
        layout.addWidget(self.range_label)
        self.range_combobox = QtWidgets.QComboBox()
        self.range_values = {
            '25': 25,
            '50': 50,
            '75': 75,
            '100': 100,
            '1/8 NM': 231.48,
            '1/4 NM': 463,
            '1/2 NM': 926,
            '3/4 NM': 1389,
            '1 NM': 1852,
            '1.5 NM': 2778,
            '2 NM': 3704,
            '3 NM': 5556,
            '4 NM': 7408,
            '6 NM': 11112,
            '8 NM': 14816,
            '12 NM': 22224,
            '16 NM': 29632,
            '24 NM': 44448
        }
        self.range_combobox.addItems(self.range_values.keys())
        self.range_combobox.currentIndexChanged.connect(self.set_range)
        layout.addWidget(self.range_combobox)

        self.gain_label = QtWidgets.QLabel('Gain')
        layout.addWidget(self.gain_label)
        self.gain_spinbox = QtWidgets.QSpinBox()
        self.gain_spinbox.setRange(0,100)
        self.gain_spinbox.setSingleStep(1)
        self.gain_spinbox.setValue(20)
        self.gain_spinbox.valueChanged.connect(self.set_gain)
        layout.addWidget(self.gain_spinbox)

        self.mode_label = QtWidgets.QLabel('Mode')
        layout.addWidget(self.mode_label)
        self.mode_combobox = QtWidgets.QComboBox()
        self.mode_combobox.addItems(['custom', 'harbor', 'offshore', 'weather', 'bird'])
        self.mode_combobox.currentIndexChanged.connect(self.set_mode)
        layout.addWidget(self.mode_combobox)

        self.sea_clutter_label = QtWidgets.QLabel('Sea Clutter')
        layout.addWidget(self.sea_clutter_label)
        self.sea_clutter_spinbox = QtWidgets.QSpinBox()
        self.sea_clutter_spinbox.setRange(0,100)
        self.sea_clutter_spinbox.setSingleStep(1)
        self.sea_clutter_spinbox.setValue(0)
        self.sea_clutter_spinbox.valueChanged.connect(self.set_sea_clutter)
        layout.addWidget(self.sea_clutter_spinbox)

        self.auto_sea_clutter_nudge_button = QtWidgets.QPushButton('Enable Auto Sea Clutter Nudge')
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
        value=self.range_values[self.range_combobox.currentText()]
        self.radar_node.get_logger().info('Setting range: %d'% value)
        command = RadarControlValue()
        command.key = 'range'
        command.value = str(value)
        self.radar_node.command_publisher.publish(command)

    def set_gain(self, value):
        self.radar_node.get_logger().info('Setting gain: %d' % value)
        command = RadarControlValue()
        command.key = 'gain'
        command.value = str(value+1)  
        self.radar_node.command_publisher.publish(command)

    def set_mode(self):
        self.radar_node.get_logger().info('Setting mode...')
        command = RadarControlValue()
        command.key = 'mode'
        command.value = self.mode_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_sea_clutter(self,value):
        command = RadarControlValue()
        if self.sea_clutter_auto_mode:
            self.radar_node.get_logger().info('Setting auto sea clutter offset: %d' % value)
            command.key = 'auto_sea_clutter_nudge'
        else:
            self.radar_node.get_logger().info('Setting sea clutter: %d' % value)
            command.key = 'sea_clutter'
        command.value = str(value)  
        self.radar_node.command_publisher.publish(command)

    def set_auto_sea_clutter_nudge(self):
        self.sea_clutter_auto_mode = not self.sea_clutter_auto_mode
        command = RadarControlValue()
        command.key = 'sea_clutter' 
        self.sea_clutter_spinbox.setValue(0)
        if self.sea_clutter_auto_mode:
            command.value = 'auto'      
            self.sea_clutter_label.setText('Auto Sea Clutter Offset')
            self.sea_clutter_spinbox.setRange(-50,50)
            self.radar_node.command_publisher.publish(command)
            command.key = 'auto_sea_clutter_nudge'
            self.auto_sea_clutter_nudge_button.setText('Disable Auto Sea Clutter Nudge')
        else:
            self.sea_clutter_label.setText('Sea Clutter')
            self.sea_clutter_spinbox.setRange(0,100)
            self.auto_sea_clutter_nudge_button.setText('Enable Auto Sea Clutter Nudge')
        command.value = '0'  #reset the value
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
        command.value = '0.784314'  
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
        command.value = '2.000000'  
        self.radar_node.command_publisher.publish(command)

    def set_antenna_height(self):
        self.radar_node.get_logger().info('Setting antenna height...')
        command = RadarControlValue()
        command.key = 'antenna_height'
        command.value = '4.000000'  
        self.radar_node.command_publisher.publish(command)

    def set_bearing_alignment(self):
        self.radar_node.get_logger().info('Setting bearing alignment...')
        command = RadarControlValue()
        command.key = 'bearing_alignment'
        command.value = '0.000000'  
        self.radar_node.command_publisher.publish(command)

    def set_sidelobe_suppression(self):
        self.radar_node.get_logger().info('Setting sidelobe suppression...')
        command = RadarControlValue()
        command.key = 'sidelobe_suppression'
        command.value = 'auto'  
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
    
def sync_radar_status(msg):
    print(msg)        

def main(args=None):
    rclpy.init(args=args)
    radar_node = Node('radar_control_panel')
    radar_node.command_publisher = radar_node.create_publisher(RadarControlValue, '/HaloA/change_state', 10)
    radar_node.status_subscription = radar_node.create_subscription(RadarControlSet, '/HaloA/status',sync_radar_status,10)
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
