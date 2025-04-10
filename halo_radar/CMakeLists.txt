cmake_minimum_required(VERSION 3.5)
project(halo_radar)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(marine_radar_control_msgs REQUIRED)
find_package(marine_sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${marine_radar_control_msgs_INCLUDE_DIRS}
  ${marine_sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

set (SOURCES
    src/halo_radar/halo_radar.cpp
    src/halo_radar/halo_radar_node.cpp
)

set(DEPENDENCIES
  rclcpp
  std_msgs
  marine_radar_control_msgs
  marine_sensor_msgs
  rosbag2_cpp
  nav_msgs
  tf2
  tf2_geometry_msgs
  sensor_msgs
  pcl_conversions
)

add_executable(${PROJECT_NAME} ${SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}
  rosbag2_cpp::rosbag2_cpp
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

add_library(halo_radar_components SHARED
  src/halo_radar/halo_radar_visualize.cpp
  src/halo_radar/halo_radar_merge_scan.cpp
  src/halo_radar/halo_radar_data_cropper.cpp
)

ament_target_dependencies(halo_radar_components
  ${DEPENDENCIES}
  rclcpp_components
)

rclcpp_components_register_nodes(halo_radar_components
  "RadarVisualizeNode"
  "HaloRadarMergeScan"
  "HaloRadarDataCropper"
)

install(TARGETS halo_radar_components
  DESTINATION lib
)

add_executable(fix_angle_increments src/halo_radar/fix_angle_increments.cpp)
ament_target_dependencies(fix_angle_increments ${DEPENDENCIES})

install(TARGETS fix_angle_increments
  DESTINATION lib/${PROJECT_NAME})

add_executable(fix_scan_times src/halo_radar/fix_scan_times.cpp)
ament_target_dependencies(fix_scan_times ${DEPENDENCIES})

install(TARGETS fix_scan_times
  DESTINATION lib/${PROJECT_NAME})


ament_export_dependencies(${dependencies})

ament_package()
