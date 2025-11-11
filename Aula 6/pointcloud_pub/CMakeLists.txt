cmake_minimum_required(VERSION 3.5)
project(pointcloud_pub)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pointcloud_publisher src/pointcloud_publisher.cpp)
ament_target_dependencies(pointcloud_publisher rclcpp sensor_msgs)

install(TARGETS
  pointcloud_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
