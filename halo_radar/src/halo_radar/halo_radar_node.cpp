#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <iostream>
#include "halo_radar.h"
#include "marine_sensor_msgs/msg/radar_sector.hpp"
#include "marine_radar_control_msgs/msg/radar_control_set.hpp"
#include "marine_radar_control_msgs/msg/radar_control_value.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/callback_group.hpp"
#include "rmw/qos_profiles.h"
#include "rclcpp/qos.hpp"
#include <future>
#include "angular_speed_estimator.h"

/**
 * @class RosRadar
 * @brief A ROS2 node for interfacing with the Halo radar system.
 * 
 * This class inherits from both halo_radar::Radar and rclcpp::Node, providing
 * functionality to process radar data and publish it to ROS2 topics, as well as
 * handle state changes and control commands.
 * 
 * @param addresses A set of addresses used to initialize the radar.
 * 
 * @section Publishers
 * - m_data_pub: Publishes radar sector data to the topic "<label>/data".
 * - m_state_pub: Publishes radar control set data to the topic "<label>/state".
 * 
 * @section Subscriptions
 * - m_state_change_sub: Subscribes to radar control value changes from the topic "<label>/change_state".
 * 
 * @section Timers
 * - m_heartbeat_timer: A timer that triggers the hbTimerCallback function every second.
 * 
 * @section Parameters
 * - range_correction_factor: A double parameter for range correction factor.
 * - frame_id: A string parameter for the frame ID.
 * 
 * @section Methods
 * - processData: Processes radar scanlines and publishes radar sector data.
 * - stateUpdated: Updates and publishes the radar control set state.
 * - stateChangeCallback: Callback function for handling state change commands.
 * - hbTimerCallback: Callback function for handling heartbeat timer events.
 * - createEnumControl: Helper function to create an enum control item.
 * - createFloatControl: Helper function to create a float control item.
 * - createFloatWithAutoControl: Helper function to create a float control item with auto mode.
 * 
 * @section Members
 * - m_data_pub: Publisher for radar sector data.
 * - m_state_pub: Publisher for radar control set data.
 * - m_state_change_sub: Subscription for radar control value changes.
 * - m_heartbeat_timer: Timer for heartbeat events.
 * - m_rangeCorrectionFactor: Range correction factor.
 * - m_frame_id: Frame ID.
 * - m_estimator: Angular speed estimator.
 */
using std::placeholders::_1;

class RosRadar : public halo_radar::Radar, public rclcpp::Node
{
public:
  RosRadar(halo_radar::AddressSet const &addresses) 
    : halo_radar::Radar(addresses)
    , Node("ros_radar_" + addresses.label)
  {
    // Create a separate callback group for subscriptions and timers
    const auto callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_data_pub = this->create_publisher<marine_sensor_msgs::msg::RadarSector>(addresses.label + "/data", 10);
    m_state_pub = this->create_publisher<marine_radar_control_msgs::msg::RadarControlSet>(addresses.label + "/state", 10);
    
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group;

    m_state_change_sub = this->create_subscription<marine_radar_control_msgs::msg::RadarControlValue>(
        addresses.label + "/change_state", 10, std::bind(&RosRadar::stateChangeCallback, this,_1), options);
    m_heartbeat_timer = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&RosRadar::hbTimerCallback, this),
      callback_group);

    this->declare_parameter<double>("range_correction_factor", m_rangeCorrectionFactor);
    this->declare_parameter<std::string>("frame_id", m_frame_id);
    this->get_parameter("range_correction_factor", m_rangeCorrectionFactor);
    this->get_parameter("frame_id", m_frame_id);

    // Create a separate executor for the callback group
    m_executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    m_executor->add_callback_group(callback_group, this->get_node_base_interface());
    m_executor_thread = std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, m_executor));

    startThreads();
  }

  ~RosRadar()
  {
    m_executor->cancel();
    if (m_executor_thread.joinable())
    {
      m_executor_thread.join();
    }
  }


protected:
  void processData(std::vector<halo_radar::Scanline> const &scanlines) override
  {
    if(scanlines.empty())
      return;
    marine_sensor_msgs::msg::RadarSector rs;
    rs.header.stamp = this->now();
    rs.header.frame_id = m_frame_id;
    rs.angle_start = 2.0*M_PI*(360-scanlines.front().angle)/360.0;
    double  angle_max = 2.0*M_PI*(360-scanlines.back().angle)/360.0;
    if(scanlines.size() > 1)
    {
      if (angle_max > rs.angle_start && angle_max-rs.angle_start > M_PI) // have we looped around (also make sure angle are decreasing)
        angle_max -= 2.0*M_PI;
      rs.angle_increment = (angle_max-rs.angle_start)/double(scanlines.size()-1);

    }
    rs.range_min = 0.0;
    rs.range_max = scanlines.front().range;
    for (auto sl : scanlines)
    {
      marine_sensor_msgs::msg::RadarEcho echo;
      for (auto i : sl.intensities)
        echo.echoes.push_back(i/15.0); // 4 bit int to float
      rs.intensities.push_back(echo);
    }

    auto angular_speed = m_estimator.update(rs.header.stamp,  rs.angle_start);
    double scan_time = 0.0;
    if(angular_speed != 0.0)
      scan_time = 2*M_PI/fabs(angular_speed);

    rs.scan_time = rclcpp::Duration::from_seconds(scan_time);

    double time_increment = 0.0;
    if (scan_time > 0)
      time_increment = std::abs(rs.angle_increment)/scan_time;
    rs.time_increment =  rclcpp::Duration::from_seconds(time_increment);
    m_data_pub->publish(rs);
  }

  void stateUpdated() override
  {
    marine_radar_control_msgs::msg::RadarControlSet rcs;

    std::string statusEnums[] = {"standby", "transmit", ""};

    createEnumControl("status", "Status", statusEnums, rcs);
    createFloatControl("range", "Range", 25, 75000, rcs);

    std::string modeEnums[] = {"custom", "harbor", "offshore", "weather", "bird", ""};

    createEnumControl("mode", "Mode", modeEnums, rcs);
    createFloatWithAutoControl("gain", "gain_mode", "Gain", 0, 100, rcs);
    createFloatWithAutoControl("sea_clutter", "sea_clutter_mode", "Sea clutter", 0, 100, rcs);
    createFloatControl("auto_sea_clutter_nudge", "Auto sea clut adj", -50, 50, rcs);

    std::string seaStateEnums[] = {"calm", "moderate", "rough", ""};

    createEnumControl("sea_state", "Sea state", seaStateEnums, rcs);
    createFloatControl("rain_clutter", "Rain clutter", 0, 100, rcs);

    std::string lowMedHighEnums[] = {"off", "low", "medium", "high", ""};

    createEnumControl("noise_rejection", "Noise rejection", lowMedHighEnums, rcs);
    createEnumControl("target_expansion", "Target expansion", lowMedHighEnums, rcs);
    createEnumControl("interference_rejection", "Interf. rej", lowMedHighEnums, rcs);
    createEnumControl("target_separation", "Target separation", lowMedHighEnums, rcs);

    std::string scanSpeedEnums[] = {"off", "medium", "high", ""};

    createEnumControl("scan_speed", "Fast scan", scanSpeedEnums, rcs);

    std::string dopplerModeEnums[] = {"off", "normal", "approaching_only", ""};

    createEnumControl("doppler_mode", "VelocityTrack", dopplerModeEnums, rcs);
    createFloatControl("doppler_speed", "Speed threshold", 0.05, 15.95, rcs);
    createFloatControl("antenna_height", "Antenna height", 0.0, 30.175, rcs);
    createFloatControl("bearing_alignment", "Bearing alignment", 0, 360, rcs);
    createFloatWithAutoControl("sidelobe_suppression", "sidelobe_suppression_mode", "Sidelobe sup.", 0, 100, rcs);
    createEnumControl("lights", "Halo light", lowMedHighEnums, rcs);

    m_state_pub->publish(rcs);
  }

private:
  void stateChangeCallback(const marine_radar_control_msgs::msg::RadarControlValue::SharedPtr cv)
  {
    RCLCPP_INFO(this->get_logger(), "State change requested: key=%s, value=%s", cv->key.c_str(), cv->value.c_str());
    sendCommand(cv->key, cv->value);
  }
  void testCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << "I heard: '" << msg->data << "'" << std::endl;
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  void hbTimerCallback()
  {
    if (checkHeartbeat())
      stateUpdated();
    RCLCPP_INFO(this->get_logger(), "Heartbeat");
  }

  void createEnumControl(std::string const &name, std::string const &label, std::string const enums[],
                         marine_radar_control_msgs::msg::RadarControlSet &rcs)
  {
    if (m_state.find(name) != m_state.end())
    {
      marine_radar_control_msgs::msg::RadarControlItem rci;
      rci.name = name;
      rci.value = m_state[name];
      rci.label = label;
      rci.type = marine_radar_control_msgs::msg::RadarControlItem::CONTROL_TYPE_ENUM;
      for (int i = 0; !enums[i].empty(); i++)
        rci.enums.push_back(enums[i]);
      rcs.items.push_back(rci);
    }
  }

  void createFloatControl(std::string const &name, std::string const &label, float min_value, float max_value,
                          marine_radar_control_msgs::msg::RadarControlSet &rcs)
  {
    if (m_state.find(name) != m_state.end())
    {
      marine_radar_control_msgs::msg::RadarControlItem rci;
      rci.name = name;
      rci.value = m_state[name];
      rci.label = label;
      rci.type = marine_radar_control_msgs::msg::RadarControlItem::CONTROL_TYPE_FLOAT;
      rci.min_value = min_value;
      rci.max_value = max_value;
      rcs.items.push_back(rci);
    }
  }

  void createFloatWithAutoControl(std::string const &name, std::string const &auto_name, std::string const &label,
                                  float min_value, float max_value, marine_radar_control_msgs::msg::RadarControlSet &rcs)
  {
    if (m_state.find(name) != m_state.end() && m_state.find(auto_name) != m_state.end())
    {
      marine_radar_control_msgs::msg::RadarControlItem rci;
      rci.name = name;
      std::string value = m_state[name];
      if (m_state[auto_name] == "auto")
        value = "auto";
      rci.value = value;
      rci.label = label;
      rci.type = marine_radar_control_msgs::msg::RadarControlItem::CONTROL_TYPE_FLOAT_WITH_AUTO;
      rci.min_value = min_value;
      rci.max_value = max_value;
      rcs.items.push_back(rci);
    }
  }

  rclcpp::Publisher<marine_sensor_msgs::msg::RadarSector>::SharedPtr m_data_pub;
  rclcpp::Publisher<marine_radar_control_msgs::msg::RadarControlSet>::SharedPtr m_state_pub;
  rclcpp::Subscription<marine_radar_control_msgs::msg::RadarControlValue>::SharedPtr m_state_change_sub;
  rclcpp::TimerBase::SharedPtr m_heartbeat_timer;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_test_sub;

  double m_rangeCorrectionFactor = 1.024;
  std::string m_frame_id = "radar";

  AngularSpeedEstimator m_estimator;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> m_executor;
  std::thread m_executor_thread;
};

std::shared_ptr<halo_radar::HeadingSender> headingSender;

void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if(headingSender)
  {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    double heading = 90.0-180.0*yaw/M_PI;
    headingSender->setHeading(heading);
  }
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("halo_radar");
  std::vector<std::shared_ptr<RosRadar>> radars;
  std::vector<uint32_t> hostIPs;
  if (node->has_parameter("hostIPs"))
  {
    std::vector<std::string> hostIPstrings;
    node->get_parameter("hostIPs", hostIPstrings);
    for (auto s: hostIPstrings)
      hostIPs.push_back(halo_radar::ipAddressFromString(s));
  }

  std::future<void> scanResult = std::async(std::launch::async, [&] {
    while(radars.empty())
    {
      std::vector<halo_radar::AddressSet> as;
      if(hostIPs.empty())
        as = halo_radar::scan();
      else
        as = halo_radar::scan(hostIPs);
      if(as.empty())
        RCLCPP_WARN(node->get_logger(), "No radars found!");
      for (auto a : as)
      {
        radars.push_back(std::make_shared<RosRadar>(a));
        if(!headingSender)
          headingSender = std::make_shared<halo_radar::HeadingSender>(a.interface);
      }
    }
  });
  
  while (rclcpp::ok())
  {
    rclcpp::spin(node);
  }


  rclcpp::shutdown();
  return 0;
}
