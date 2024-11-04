#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <marine_sensor_msgs/msg/radar_sector.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  if(argc != 3)
  {
    std::cout << "Usage: fix_angle_increments in.bag out.bag\n";
    exit(-1);
  }

  auto node = rclcpp::Node::make_shared("fix_angle_increments");

  rosbag2_cpp::Reader reader;
  reader.open(argv[1]);

  rosbag2_cpp::Writer writer;
  writer.open(argv[2]);

  while (reader.has_next())
  {
    auto bag_message = reader.read_next();
    auto sector = std::make_shared<marine_sensor_msgs::msg::RadarSector>();
    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
    rclcpp::Serialization<marine_sensor_msgs::msg::RadarSector> serialization;
    serialization.deserialize_message(&serialized_msg, sector.get());

    if(sector->intensities.size() > 1)
    {
      if(sector->angle_increment > 0.0)
      {
        auto angle_finish = sector->angle_start + sector->angle_increment * (sector->intensities.size() - 1);
        angle_finish -= 2 * M_PI;
        sector->angle_increment = (angle_finish - sector->angle_start) / float(sector->intensities.size() - 1);
        rclcpp::SerializedMessage serialized_out_msg;
        serialization.serialize_message(sector.get(), &serialized_out_msg);
          writer.write(serialized_out_msg, bag_message->topic_name, "marine_sensor_msgs/msg/RadarSector", rclcpp::Time(bag_message->time_stamp));
        continue;
      }
    }
      writer.write(bag_message);
  }

  rclcpp::shutdown();
  return 0;
}
