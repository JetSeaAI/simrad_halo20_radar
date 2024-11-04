#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>
#include "angular_speed_estimator.h"
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char* argv[])
{
  if(argc != 3)
  {
    std::cout << "Usage: fix_scan_time in.bag out.bag\n";
    exit(-1);
  }

  rclcpp::init(argc, argv);

  rosbag2_cpp::Reader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = argv[1];
  storage_options.storage_id = "sqlite3";
  reader.open(storage_options);

  rosbag2_cpp::Writer writer;
  writer.open(argv[2]);

  std::map<std::string, AngularSpeedEstimator> estimators;

  while (reader.has_next())
  {
    auto bag_message = reader.read_next();
    auto topic_name = bag_message->topic_name;
    auto serialized_data = bag_message->serialized_data;

    auto sector = std::make_shared<marine_sensor_msgs::msg::RadarSector>();
    rclcpp::Serialization<marine_sensor_msgs::msg::RadarSector> serializer;
    rclcpp::SerializedMessage serialized_msg(*serialized_data);
    serializer.deserialize_message(&serialized_msg, sector.get());

    if(sector != nullptr)
    {
      auto angular_speed = estimators[topic_name].update(sector->header.stamp, sector->angle_start);
      double scan_time = 0.0;
      if(angular_speed != 0.0)
        scan_time = 2*M_PI/fabs(angular_speed);

      sector->scan_time = rclcpp::Duration::from_seconds(scan_time);

      double time_increment = 0.0;
      if (scan_time > 0)
        time_increment = std::abs(sector->angle_increment)/scan_time;
      sector->time_increment = rclcpp::Duration::from_seconds(time_increment);

      auto serialized_out_msg = std::make_shared<rclcpp::SerializedMessage>();
      serializer.serialize_message(sector.get(), serialized_out_msg.get());
      writer.write(*serialized_out_msg, topic_name, "std_msgs/msg/String", rclcpp::Clock().now());
      continue;
    }
    writer.write(bag_message);
  }

  rclcpp::shutdown();
  return 0;
}
