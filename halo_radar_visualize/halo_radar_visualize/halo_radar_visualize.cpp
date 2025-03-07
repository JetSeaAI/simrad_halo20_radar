#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include <vector>
#include <cmath>
#include <chrono>

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
        std::vector<std::vector<float>> points;

        angle_start -= offset_;
        if (angle_start - previous_angle_ > 0.001 && angle_start > 0.05)
        {
            RCLCPP_WARN(this->get_logger(), "Angle Jump Detected. angle_start: %f, previous_angle: %f", angle_start, previous_angle_);
        }

        for (size_t i = 0; i < intensities.size(); ++i)
        {
            double angle = angle_start + i * angle_increment;
            const auto &echoes = intensities[i].echoes;
            auto generated_points = generate_points(angle, echoes, range_min, range_max);
            points.insert(points.end(), generated_points.begin(), generated_points.end());
        }

        previous_angle_ = angle_start + (intensities.size() - 1) * angle_increment;
        publish_pointcloud(points);
        publish_count_++;
    }

    std::vector<std::vector<float>> generate_points(double angle, const std::vector<float> &intensities_echoes, double range_min, double range_max)
    {
        std::vector<std::vector<float>> points;
        for (size_t i = 0; i < intensities_echoes.size(); ++i)
        {
            if (intensities_echoes[i] > 0)
            {
                double r = range_min + i * (range_max - range_min) / intensities_echoes.size();
                double x = r * std::cos(angle);
                double y = r * std::sin(angle);
                double z = 0.0;
                points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), intensities_echoes[i]});
            }
        }
        return points;
    }

    void publish_pointcloud(const std::vector<std::vector<float>> &points)
    {
        pointcloud_.header.stamp = this->get_clock()->now();
        pointcloud_.width = points.size();

        std::vector<uint8_t> data(points.size() * pointcloud_.point_step);
        for (size_t i = 0; i < points.size(); ++i)
        {
            std::memcpy(&data[i * pointcloud_.point_step], points[i].data(), pointcloud_.point_step);
        }
        pointcloud_.data = std::move(data);

        pointcloud_publisher_->publish(pointcloud_);
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
