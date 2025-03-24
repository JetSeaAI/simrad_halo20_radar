#include <rclcpp/rclcpp.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <vector>
#include <cmath>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <omp.h>

using namespace std::chrono_literals;

class RadarVisualizeNode : public rclcpp::Node
{
public:
    RadarVisualizeNode()
        : Node("radar_visualize_node"), publish_count_(0), previous_angle_(0.0)
    {
        this->declare_parameter<std::string>("single_shot_pointcloud_topic", "single_shot_radar_pointcloud");
        this->declare_parameter<std::string>("radar_input_topic", "/HaloA/data");
        this->declare_parameter<std::string>("frame_id", "radar");

        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("single_shot_pointcloud_topic").as_string(), 10);

        subscription_ = this->create_subscription<marine_sensor_msgs::msg::RadarSector>(
            this->get_parameter("radar_input_topic").as_string(),
            10,
            std::bind(&RadarVisualizeNode::radar_echo_data_callback, this, std::placeholders::_1));

        pointcloud_.header.frame_id = this->get_parameter("frame_id").as_string();
        pointcloud_.height = 1;
        pointcloud_.fields = {
            create_point_field("x", 0),
            create_point_field("y", 4),
            create_point_field("z", 8),
            create_point_field("intensity", 12),
        };
        pointcloud_.is_bigendian = false;
        pointcloud_.point_step = 16;
        pointcloud_.is_dense = true;

        timer_ = this->create_wall_timer(5s, std::bind(&RadarVisualizeNode::report_publish_rate, this));
    }

private:
    sensor_msgs::msg::PointField create_point_field(const std::string &name, int offset)
    {
        sensor_msgs::msg::PointField field;
        field.name = name;
        field.offset = offset;
        field.datatype = sensor_msgs::msg::PointField::FLOAT32;
        field.count = 1;
        return field;
    }

    void radar_echo_data_callback(const marine_sensor_msgs::msg::RadarSector::SharedPtr msg)
    {
        double angle_start = msg->angle_start;
        double angle_increment = msg->angle_increment;
        double range_min = msg->range_min;
        double range_max = msg->range_max;
        const auto &intensities = msg->intensities;
        pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
        pcl_cloud.reserve(intensities.size() * intensities[0].echoes.size());

        angle_start -= offset_;
        if (angle_start - previous_angle_ > 0.001 && angle_start > 0.05)
        {
            RCLCPP_WARN(this->get_logger(), "Angle Jump Detected. angle_start: %f, previous_angle: %f", angle_start, previous_angle_);
        }

#pragma omp parallel for default(shared)
        for (int i = 0; i < static_cast<int>(intensities.size()); ++i)
        {
            double angle = angle_start + i * angle_increment;
            const auto &echoes = intensities[i].echoes;

            for (size_t j = 0; j < echoes.size(); ++j)
            {
                if (echoes[j] > 0)
                {
                    double r = range_min + j * (range_max - range_min) / echoes.size();
                    pcl::PointXYZI pt;
                    pt.x = r * std::cos(angle);
                    pt.y = r * std::sin(angle);
                    pt.z = 0.0;
                    pt.intensity = echoes[j];

#pragma omp critical
                    pcl_cloud.push_back(pt);
                }
            }
        }

        previous_angle_ = angle_start + (intensities.size() - 1) * angle_increment;

        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(pcl_cloud, ros_cloud);
        ros_cloud.header.frame_id = pointcloud_.header.frame_id;
        ros_cloud.header.stamp = this->get_clock()->now();
        publish_pointcloud(ros_cloud);

        publish_count_++;
    }

    void publish_pointcloud(const sensor_msgs::msg::PointCloud2 &ros_cloud)
    {
        pointcloud_publisher_->publish(ros_cloud);
    }

    void report_publish_rate()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing rate: %f pointclouds per second", publish_count_ / 5.0);
        publish_count_ = 0;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Subscription<marine_sensor_msgs::msg::RadarSector>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2 pointcloud_;
    double offset_ = 2 * M_PI;
    double previous_angle_;
    int publish_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarVisualizeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
