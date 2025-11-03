cmake_minimum_required(VERSION 3.8)
project(simple_car_description)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(simple_car_description_simple_car_hardware SHARED
  config/simple_car_hardware.xml
)
add_executable(wheel_ref_publisher src/wheel_ref_publisher.cpp)
ament_target_dependencies(wheel_ref_publisher rclcpp std_msgs)


install(TARGETS
  simple_car_description_simple_car_hardware
  DESTINATION lib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  config/simple_car_hardware.xml 
)


install(DIRECTORY urdf launch rviz
DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
        wheel_ref_publisher 
        DESTINATION lib/${PROJECT_NAME})

ament_package()