cmake_minimum_required(VERSION 3.5.0)
project(state_feedback_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
list( APPEND CMAKE_MODULE_PATH
  ${PROJECT_SOURCE_DIR}/CMake)

# Find other packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realtime_tools REQUIRED)

# Dependências do controlador
set(CONTROLLER_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  pluginlib
  hardware_interface
  controller_interface
  realtime_tools
  trajectory_msgs
)

## Instruções para compilar e exportar controlador

add_library(
  state_feedback_controller
  SHARED
  src/state_feedback_controller.cpp
)

target_compile_features(state_feedback_controller PUBLIC cxx_std_17)
target_include_directories(state_feedback_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(state_feedback_controller PUBLIC
  ${trajectory_msgs_TARGETS}
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
)

# Causes the visibility macros to use dllexport rather than dllimport,
#  which is appropriate when building the dll but not consuming it.
target_compile_definitions(state_feedback_controller PRIVATE "STATE_FEEDBACK_CONTROLLER_BUILDING_LIBRARY")
# Exporta o plugin do controlador
pluginlib_export_plugin_description_file(controller_interface state_feedback_controller.xml)

## Instalação
install(DIRECTORY
  launch config scripts
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES state_feedback_controller.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS state_feedback_controller
  EXPORT export_state_feedback_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# install(PROGRAMS
# 	scripts/tune_state_feedback_controller.py
# 	DESTINATION lib/${PROJECT_NAME}
# )


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

## Exporta dependencias
# ament_export_targets(export_state_feedback_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_include_directories(
  include
)
ament_export_libraries(
  state_feedback_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)


ament_package()
