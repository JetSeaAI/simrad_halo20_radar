#include "rclcpp/rclcpp.hpp"
#include "marine_sensor_msgs/msg/radar_sector.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <omp.h>
#include <rclcpp_components/register_node_macro.hpp>

class HaloRadarMergeScan : public rclcpp::Node {
public:
     explicit HaloRadarMergeScan(const rclcpp::NodeOptions & options)
        : Node("halo_radar_merge_scan", options), previous_angle_(0.0f) {
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
            if (!merged_msg_.data.empty()) {
                merged_msg_.header.stamp = now(); // get_clock()->now();
                pointcloud_publisher_->publish(merged_msg_);
                merged_msg_.data.clear();
            }
        }
        previous_angle_ = angle;
    }

    void listener_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        if (msg->data.empty()) {
            return;
        }
        // Incrementally update the merged_msg_ with new points
        if (merged_msg_.data.empty()) {
            merged_msg_ = *msg;
        } else {
            merged_msg_.data.insert(merged_msg_.data.end(), msg->data.begin(), msg->data.end());
            merged_msg_.width += msg->width; // Update the width to reflect the total number of points
        }
    }

    std::string single_shot_pointcloud_topic_;
    std::string radar_input_topic_;
    std::string merged_pointcloud_topic_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr halo_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    sensor_msgs::msg::PointCloud2 merged_msg_; // Incrementally updated pointcloud message
    float previous_angle_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(HaloRadarMergeScan)