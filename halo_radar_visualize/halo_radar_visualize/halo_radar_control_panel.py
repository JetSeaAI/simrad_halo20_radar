#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from marine_radar_control_msgs.msg import RadarControlValue,RadarControlSet
from PyQt5 import QtWidgets, QtCore
import math
class RadarConfigGUI(QtWidgets.QWidget):
    def __init__(self, radar_node):
        super().__init__()
        self.radar_node = radar_node
        self.initUI()
        self.sea_clutter_auto_mode=False

    def initUI(self):
        self.setWindowTitle('Radar Configuration')
        layout = QtWidgets.QVBoxLayout()

        self.operation_label = QtWidgets.QLabel('Radar Sratus: Unknown')
        layout.addWidget(self.operation_label)
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
            '50': 50,
            '1/32':57,
            '75': 75,
            '100': 100,
            '1/16':115,
            '1/8 NM': 231,
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

        self.rain_clutter_label = QtWidgets.QLabel('Rain Clutter')
        layout.addWidget(self.rain_clutter_label)
        self.rain_clutter_spinbox = QtWidgets.QSpinBox()
        self.rain_clutter_spinbox.setRange(0, 100)
        self.rain_clutter_spinbox.setSingleStep(1)
        self.rain_clutter_spinbox.setValue(0)
        self.rain_clutter_spinbox.valueChanged.connect(self.set_rain_clutter)
        layout.addWidget(self.rain_clutter_spinbox)

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


        self.bearing_alignment_label = QtWidgets.QLabel('Bearing alignment')
        layout.addWidget(self.bearing_alignment_label)
        self.bearing_alignment_spinbox = QtWidgets.QDoubleSpinBox()
        self.bearing_alignment_spinbox.setRange(0.0, 360.0)
        self.bearing_alignment_spinbox.setSingleStep(1.0)
        self.bearing_alignment_spinbox.setValue(0.0)#default value  
        self.bearing_alignment_spinbox.valueChanged.connect(self.set_bearing_alignment)
        layout.addWidget(self.bearing_alignment_spinbox)

        self.lights_label = QtWidgets.QLabel('Halo light')
        layout.addWidget(self.lights_label)
        self.lights_combobox = QtWidgets.QComboBox()
        self.lights_combobox.addItems(['off', 'low', 'medium', 'high'])
        self.lights_combobox.currentIndexChanged.connect(self.set_lights)
        layout.addWidget(self.lights_combobox)

        

        layout.addWidget(QtWidgets.QLabel('Advanced settings'))
        self.advanced_settings_checkbox = QtWidgets.QCheckBox('Show advanced settings')
        layout.addWidget(self.advanced_settings_checkbox)
        self.advanced_settings_checkbox.stateChanged.connect(self.set_advanced_settings)
        
        self.antenna_height_label = QtWidgets.QLabel('Antenna height')
        layout.addWidget(self.antenna_height_label)
        self.antenna_height_spinbox = QtWidgets.QDoubleSpinBox()
        self.antenna_height_spinbox.setRange(0, 30)
        self.antenna_height_spinbox.setSingleStep(1)
        self.antenna_height_spinbox.setValue(4) #default value 
        self.antenna_height_spinbox.setEnabled(False) #disabled for safety
        self.antenna_height_spinbox.valueChanged.connect(self.set_antenna_height)
        layout.addWidget(self.antenna_height_spinbox)

        self.doppler_speed_label = QtWidgets.QLabel('Doppler Speed threshold')
        layout.addWidget(self.doppler_speed_label)
        self.doppler_speed_spinbox = QtWidgets.QDoubleSpinBox()
        self.doppler_speed_spinbox.setRange(0.05, 15.95)
        self.doppler_speed_spinbox.setSingleStep(0.05)
        self.doppler_speed_spinbox.setValue(2.0)
        self.doppler_speed_spinbox.setEnabled(False)  #unknown effect disable for safety
        self.doppler_speed_spinbox.valueChanged.connect(self.set_doppler_speed)
        layout.addWidget(self.doppler_speed_spinbox)

        self.sidelobe_suppression_label = QtWidgets.QLabel('Sidelobe sup.')
        layout.addWidget(self.sidelobe_suppression_label)
        self.sidelobe_suppression_spinbox = QtWidgets.QSpinBox()
        self.sidelobe_suppression_spinbox.setRange(0, 100)
        self.sidelobe_suppression_spinbox.setSingleStep(1)
        self.sidelobe_suppression_spinbox.setValue(50)
        self.sidelobe_suppression_spinbox.setEnabled(False)  #unknown effect disable for safety
        self.sidelobe_suppression_spinbox.valueChanged.connect(self.set_sidelobe_suppression)
        layout.addWidget(self.sidelobe_suppression_spinbox)



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
        self.radar_node.get_logger().info('Setting mode: %s' % self.mode_combobox.currentText())
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
        self.radar_node.get_logger().info('Setting sea state: %s' % self.sea_state_combobox.currentText())
        command = RadarControlValue()
        command.key = 'sea_state'
        command.value = self.sea_state_combobox.currentText()
        self.radar_node.command_publisher.publish(command)

    def set_rain_clutter(self, value):
        self.radar_node.get_logger().info('Setting rain clutter: %d' % value)
        command = RadarControlValue()
        command.key = 'rain_clutter'
        command.value = str(value)  # Convert to float value
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

    def set_doppler_speed(self, value):
        self.radar_node.get_logger().info('Setting doppler speed: %.2f' % value)
        command = RadarControlValue()
        command.key = 'doppler_speed'
        command.value = str(value)
        self.radar_node.command_publisher.publish(command)

    def set_antenna_height(self, value):
        self.radar_node.get_logger().info('Setting antenna height: %.2f' % value)
        command = RadarControlValue()
        command.key = 'antenna_height'
        command.value = str(value)
        self.radar_node.command_publisher.publish(command)

    def set_bearing_alignment(self, value):
        self.radar_node.get_logger().info('Setting bearing alignment: %.2f' % value)
        command = RadarControlValue()
        command.key = 'bearing_alignment'
        command.value = str(value)
        self.radar_node.command_publisher.publish(command)

    def set_sidelobe_suppression(self, value):
        self.radar_node.get_logger().info('Setting sidelobe suppression: %d' % value)
        command = RadarControlValue()
        command.key = 'sidelobe_suppression'
        command.value = str(value)
        self.radar_node.command_publisher.publish(command)

    def set_lights(self):
        self.radar_node.get_logger().info('Setting lights...')
        command = RadarControlValue()
        command.key = 'lights'
        command.value = self.lights_combobox.currentText()
        self.radar_node.command_publisher.publish(command)
    
    def set_advanced_settings(self, enabled):
        self.antenna_height_spinbox.setEnabled(enabled)
        self.doppler_speed_spinbox.setEnabled(enabled)
        self.sidelobe_suppression_spinbox.setEnabled(enabled)

    def closeEvent(self, event):
        self.radar_node.get_logger().info('Closed by user')
        self.radar_node.destroy_node()
        event.accept()
        raise SystemExit
    
def sync_radar_status(msg):
    gui = RadarConfigGUI.instance
    update_map = {
        'status': lambda value: gui.operation_label.setText('Radar Status: ' + value),
        'range': lambda value: 
        gui.range_combobox.setCurrentText(list(gui.range_values.keys())
                                          [list(gui.range_values.values()).index(int(value))])
                                            if gui.range_values[gui.range_combobox.currentText()] != int(value) 
                                            else None,
        'gain': lambda value: gui.gain_spinbox.setValue(math.floor(float(value))) if gui.gain_spinbox.value() != math.floor(float(value)) else None,
        'mode': lambda value: gui.mode_combobox.setCurrentText(value) if gui.mode_combobox.currentText() != value else None,
        # 'sea_clutter': lambda value: gui.sea_clutter_spinbox.setValue(int(float(value))) if gui.sea_clutter_spinbox.value() != int(float(value)) else None,
        # 'auto_sea_clutter_nudge': lambda value: gui.sea_clutter_spinbox.setValue(int(value)) if gui.sea_clutter_spinbox.value() != int(value) else None,
        'sea_state': lambda value: gui.sea_state_combobox.setCurrentText(value) if gui.sea_state_combobox.currentText() != value else None,
        'rain_clutter': lambda value: gui.rain_clutter_spinbox.setValue(math.ceil(float(value))) if gui.rain_clutter_spinbox.value() != math.floor(float(value)) else None,
        'noise_rejection': lambda value: gui.noise_rejection_combobox.setCurrentText(value) if gui.noise_rejection_combobox.currentText() != value else None,
        'target_expansion': lambda value: gui.target_expansion_combobox.setCurrentText(value) if gui.target_expansion_combobox.currentText() != value else None,
        'interference_rejection': lambda value: gui.interference_rejection_combobox.setCurrentText(value) if gui.interference_rejection_combobox.currentText() != value else None,
        'target_separation': lambda value: gui.target_separation_combobox.setCurrentText(value) if gui.target_separation_combobox.currentText() != value else None,
        'scan_speed': lambda value: gui.scan_speed_combobox.setCurrentText(value) if gui.scan_speed_combobox.currentText() != value else None,
        'doppler_mode': lambda value: gui.doppler_mode_combobox.setCurrentText(value) if gui.doppler_mode_combobox.currentText() != value else None,
        'doppler_speed': lambda value: gui.doppler_speed_spinbox.setValue(float(value)) if gui.doppler_speed_spinbox.value() != float(value) else None,
        'antenna_height': lambda value: gui.antenna_height_spinbox.setValue(float(value)) if gui.antenna_height_spinbox.value() != float(value) else None,
        'bearing_alignment': lambda value: gui.bearing_alignment_spinbox.setValue(float(value)) if gui.bearing_alignment_spinbox.value() != float(value) else None,
        # 'sidelobe_suppression': lambda value: gui.sidelobe_suppression_spinbox.setValue(int(value)) if gui.sidelobe_suppression_spinbox.value() != int(value) else None,
        'lights': lambda value: gui.lights_combobox.setCurrentText(value) if gui.lights_combobox.currentText() != value else None,
    }


    for item in msg.items:
        if item.name in update_map and item.value is not None:
            update_map[item.name](item.value)

        if (item.name == 'sea_clutter') :
            if item.value == 'auto' and not gui.sea_clutter_auto_mode:
                gui.set_auto_sea_clutter_nudge()
            elif item.value != 'auto' and math.ceil(float(item.value)) != gui.sea_clutter_spinbox.value():
                if gui.sea_clutter_auto_mode:
                    gui.set_auto_sea_clutter_nudge()
                gui.sea_clutter_spinbox.setValue(math.ceil(float(item.value)))
                #some value jumpping issue here     
            
        
        if item.name == 'auto_sea_clutter_nudge' and gui.sea_clutter_auto_mode:
            gui.sea_clutter_spinbox.setValue(int(item.value))
            gui.sea_clutter_auto_mode = True

def main(args=None):
    rclpy.init(args=args)
    radar_node = Node('radar_control_panel')
    radar_node.command_publisher = radar_node.create_publisher(RadarControlValue, '/HaloA/change_state', 10)
    radar_node.status_subscription = radar_node.create_subscription(RadarControlSet, '/HaloA/state', sync_radar_status, 10)

    app = QtWidgets.QApplication([])
    gui = RadarConfigGUI(radar_node)
    RadarConfigGUI.instance = gui
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
