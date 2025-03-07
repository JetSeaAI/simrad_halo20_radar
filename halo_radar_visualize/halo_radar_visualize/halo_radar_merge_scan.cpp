#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "marine_sensor_msgs/msg/radar_sector.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class HaloRadarMergeScan : public rclcpp::Node {
public:
    HaloRadarMergeScan() : Node("halo_radar_merge_scan"), previous_angle_(0), publish_merged_pointcloud_(false) {
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
            RCLCPP_INFO(this->get_logger(), "Full Scan. Publish the merged pointcloud");
            publish_merged_pointcloud_ = true;
        }
        previous_angle_ = angle;
    }

    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);
        if (cloud.empty()) {
            // RCLCPP_INFO(this->get_logger(), "No points found in the PointCloud2 message.");
            return;
        }
        full_stack_pointcloud_.push_back(cloud);
        if (publish_merged_pointcloud_) {
            merge_pointcloud(msg);
            publish_merged_pointcloud_ = false;
        }
    }

    void merge_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> full_pointcloud;
        for (const auto& cloud : full_stack_pointcloud_) {
            full_pointcloud += cloud;
        }
        sensor_msgs::msg::PointCloud2 full_pointcloud_msg;
        pcl::toROSMsg(full_pointcloud, full_pointcloud_msg);
        full_pointcloud_msg.header = msg->header;
        pointcloud_publisher_->publish(full_pointcloud_msg);
        full_stack_pointcloud_.clear();
    }

    std::string single_shot_pointcloud_topic_;
    std::string radar_input_topic_;
    std::string merged_pointcloud_topic_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr halo_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> full_stack_pointcloud_;
    float previous_angle_;
    bool publish_merged_pointcloud_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HaloRadarMergeScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
