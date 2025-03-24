#include "rclcpp/rclcpp.hpp"
#include "marine_sensor_msgs/msg/radar_sector.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omp.h>

class HaloRadarMergeScan : public rclcpp::Node {
public:
    HaloRadarMergeScan() : Node("halo_radar_merge_scan"), previous_angle_(0.0f) {
        this->declare_parameter<std::string>("single_shot_pointcloud_topic", "single_shot_radar_pointcloud");
        this->declare_parameter<std::string>("radar_input_topic", "/HaloA/data");
        this->declare_parameter<std::string>("merged_pointcloud_topic", "merged_pointcloud");

        single_shot_pointcloud_topic_ = this->get_parameter("single_shot_pointcloud_topic").as_string();
        radar_input_topic_ = this->get_parameter("radar_input_topic").as_string();
        merged_pointcloud_topic_ = this->get_parameter("merged_pointcloud_topic").as_string();

        pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            single_shot_pointcloud_topic_, 10, std::bind(&HaloRadarMergeScan::listener_callback, this, std::placeholders::_1));
        
        halo_subscription_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
            radar_input_topic_, 10, std::bind(&HaloRadarMergeScan::radar_data_callback, this, std::placeholders::_1));
        
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_pointcloud_topic_, 10);
    }

private:
    void radar_data_callback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg) {
        float angle = msg->angle_start;
        if (angle > previous_angle_) {
            if (!full_pointcloud_.empty()) {
                static sensor_msgs::msg::PointCloud2 merged_msg;
                merged_msg.data.clear();

                pcl::toROSMsg(full_pointcloud_, merged_msg);
                merged_msg.header.frame_id = "radar";
                merged_msg.header.stamp = now(); // 或可取用最後一個 msg 的 timestamp

                pointcloud_publisher_->publish(merged_msg);
                full_pointcloud_.clear();
            }
        }
        previous_angle_ = angle;
    }

    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);
        if (cloud.empty()) {
            return;
        }
        // 直接累積到全域變數
        full_pointcloud_ += cloud;
    }

    std::string single_shot_pointcloud_topic_;
    std::string radar_input_topic_;
    std::string merged_pointcloud_topic_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr halo_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    pcl::PointCloud<pcl::PointXYZI> full_pointcloud_;
    float previous_angle_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HaloRadarMergeScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}