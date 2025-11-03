cmake_minimum_required(VERSION 3.8)
project(simple_car_description)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(wheel_ref_publisher src/wheel_ref_publisher.cpp)
ament_target_dependencies(wheel_ref_publisher rclcpp std_msgs)


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