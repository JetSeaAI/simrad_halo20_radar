
#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
        sensor_msgs::msg::PointCloud2 cropped;
        cropped.header = msg->header;
        cropped.height = 1; 
        cropped.is_bigendian = msg->is_bigendian;
        cropped.is_dense = false;

        // 建立輸出雲的欄位結構 (x, y, z, intensity)
        cropped.fields = msg->fields; 
        cropped.fields.resize(4); 
        cropped.fields[0].offset = 0;
        cropped.fields[1].offset = 4;
        cropped.fields[2].offset = 8;
        cropped.fields[3].offset = 12;

        cropped.point_step = 16;

        std::vector<uint8_t> data_buffer;
        data_buffer.reserve(msg->width * cropped.point_step);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");

        for (size_t idx = 0; idx < msg->width * msg->height;
             ++idx, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            float intensity = *iter_i;

            float dist = std::sqrt(x * x + y * y + z * z);
            float angle_deg = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);

            if (angle_deg >= angle_start_ && angle_deg <= angle_end_ &&
                dist >= dist_start_ && dist <= dist_end_)
            {
                // 塞進 data_buffer
                const uint8_t* raw_ptr = reinterpret_cast<const uint8_t*>(&x);
                data_buffer.insert(data_buffer.end(), raw_ptr, raw_ptr + 4);
                raw_ptr = reinterpret_cast<const uint8_t*>(&y);
                data_buffer.insert(data_buffer.end(), raw_ptr, raw_ptr + 4);
                raw_ptr = reinterpret_cast<const uint8_t*>(&z);
                data_buffer.insert(data_buffer.end(), raw_ptr, raw_ptr + 4);
                raw_ptr = reinterpret_cast<const uint8_t*>(&intensity);
                data_buffer.insert(data_buffer.end(), raw_ptr, raw_ptr + 4);
            }
        }

        cropped.row_step = static_cast<uint32_t>(data_buffer.size());
        cropped.data = data_buffer;
        cropped.width = cropped.data.size() / cropped.point_step;
        pub_->publish(cropped);
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