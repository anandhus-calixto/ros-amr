cmake_minimum_required(VERSION 3.10)
project(calixto-ros-bot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. Find standard ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# --- NEW: Use PkgConfig to find libserial ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# 2. Build the hardware library
add_library(calixto_hardware SHARED
  hardware/diffbot_system.cpp
)

# --- ADD the PkgConfig include directories ---
target_include_directories(calixto_hardware PRIVATE
  hardware/include
  ${LIBSERIAL_INCLUDE_DIRS}
)

# Use ament_target_dependencies for ROS packages
ament_target_dependencies(calixto_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  diagnostic_msgs
)

# --- NEW: Link the libserial libraries specifically ---
target_link_libraries(calixto_hardware
  ${LIBSERIAL_LIBRARIES}
)

# 3. Export the plugin
pluginlib_export_plugin_description_file(hardware_interface calixto-ros-bot.xml)

# 4. Install
install(TARGETS calixto_hardware
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config description launch worlds maps rviz gazebo
  DESTINATION share/${PROJECT_NAME}
)

install(FILES calixto-ros-bot.xml
  DESTINATION share/${PROJECT_NAME}
)

install(FILES calixto-ros-bot.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
