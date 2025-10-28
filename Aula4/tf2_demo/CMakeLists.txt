cmake_minimum_required(VERSION 3.8)
project(tf2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# nó estático
add_executable(static_cargo_pub src/static_drone_pub.cpp)
target_include_directories(static_cargo_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(static_cargo_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  static_cargo_pub
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

# nó de publicação da transformada do drone realizando a curva de Viviani
add_executable(drone_tf_pub src/drone_tf_pub.cpp)
target_include_directories(drone_tf_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(drone_tf_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  drone_tf_pub
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

# nó do TF do cargo, melhorado
add_executable(cargo_pubsub src/cargo_pubsub.cpp)
target_include_directories(cargo_pubsub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(cargo_pubsub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  cargo_pubsub
  "rclcpp"
  "tf2"
  "tf2_ros"
  "tf2_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

# Eigen é linkado pelo comando target_link_libraries - forma padrão do CMake de linkar bibliotecas externas
target_link_libraries(drone_tf_pub Eigen3::Eigen) 

install(TARGETS cargo_pubsub drone_tf_pub static_cargo_pub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
