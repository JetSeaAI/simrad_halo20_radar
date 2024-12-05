import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class RadarMergeScan(Node):
    def __init__(self):
        super().__init__('radar_merge_scan')
        qos_profile = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan,
            '/radar/cropped_scan',
            self.listener_callback,
            qos_profile)
        self.publisher = self.create_publisher(LaserScan, '/radar/merged_scan', 10)
        self.scan_data = []
        self.total_angle = 0.0

    def listener_callback(self, msg):
        self.scan_data.append(msg)
        self.total_angle += msg.angle_increment * len(msg.ranges)
        # self.get_logger().info(f"Total angle: {self.total_angle}")

        if self.total_angle >= 240.0 * (3.14159265359 / 180.0):  # 240 degrees in radians
            merged_scan = self.merge_scans(self.scan_data)
            self.publisher.publish(merged_scan)
            self.scan_data = []
            self.total_angle = 0.0

    def merge_scans(self, scans):
        if not scans:
            return None

        merged_scan = LaserScan()
        merged_scan.header = scans[0].header
        merged_scan.angle_min = scans[0].angle_min
        merged_scan.angle_max = scans[-1].angle_max
        merged_scan.angle_increment = scans[0].angle_increment
        merged_scan.time_increment = scans[0].time_increment
        merged_scan.scan_time = scans[0].scan_time
        merged_scan.range_min = scans[0].range_min
        merged_scan.range_max = scans[0].range_max
        merged_scan.ranges = []
        merged_scan.intensities = []

        for scan in scans:
            merged_scan.ranges.extend(scan.ranges)
            merged_scan.intensities.extend(scan.intensities)

        return merged_scan

def main(args=None):
    rclpy.init(args=args)
    radar_merge_scan = RadarMergeScan()
    rclpy.spin(radar_merge_scan)
    radar_merge_scan.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()