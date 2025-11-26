cmake_minimum_required(VERSION 3.8)
project(pointcloud_parser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_executable(pointcloud_parser src/pointcloud_parser.cpp)
target_include_directories(pointcloud_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(pointcloud_parser PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pointcloud_parser
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
  "tf2_ros"
  "tf2"
)

install(TARGETS pointcloud_parser
  DESTINATION lib/${PROJECT_NAME})

ament_package()
