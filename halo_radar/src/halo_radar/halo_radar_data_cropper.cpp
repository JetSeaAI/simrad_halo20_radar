#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class HaloRadarDataCropper : public rclcpp::Node
{
public:
    HaloRadarDataCropper() : Node("halo_radar_data_cropper")
    {
        this->declare_parameter<std::string>("input_pointcloud_topic", "merged_pointcloud");
        this->declare_parameter<std::string>("cropped_pointcloud_topic", "cropped_pointcloud");
        this->declare_parameter<double>("cropped_angle_start", -120.0);
        this->declare_parameter<double>("cropped_angle_end", 120.0);
        this->declare_parameter<double>("cropped_distance_start", 20.0);
        this->declare_parameter<double>("cropped_distance_end", 120.0);

        input_topic_ = this->get_parameter("input_pointcloud_topic").as_string();
        output_topic_ = this->get_parameter("cropped_pointcloud_topic").as_string();
        angle_start_ = this->get_parameter("cropped_angle_start").as_double();
        angle_end_ = this->get_parameter("cropped_angle_end").as_double();
        dist_start_ = this->get_parameter("cropped_distance_start").as_double();
        dist_end_ = this->get_parameter("cropped_distance_end").as_double();

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&HaloRadarDataCropper::cloudCallback, this, std::placeholders::_1));
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI> input_cloud;
        pcl::fromROSMsg(*msg, input_cloud);

        pcl::PointCloud<pcl::PointXYZI> cropped_cloud;
        cropped_cloud.reserve(input_cloud.size());

#pragma omp parallel for default(shared)
        for (int i = 0; i < static_cast<int>(input_cloud.size()); ++i)
        {
            const auto &pt = input_cloud[i];
            float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            float angle_deg = std::atan2(pt.y, pt.x) * 180.0f / static_cast<float>(M_PI);

            if (angle_deg >= angle_start_ && angle_deg <= angle_end_ &&
                dist >= dist_start_ && dist <= dist_end_)
            {
#pragma omp critical
                cropped_cloud.push_back(pt);
            }
        }

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(cropped_cloud, output_msg);
        output_msg.header = msg->header;
        pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    std::string input_topic_;
    std::string output_topic_;
    double angle_start_;
    double angle_end_;
    double dist_start_;
    double dist_end_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HaloRadarDataCropper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}