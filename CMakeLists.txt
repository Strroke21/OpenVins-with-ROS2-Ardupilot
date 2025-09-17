cmake_minimum_required(VERSION 3.8)
project(vins_mono)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Find MAVSDK
find_package(PkgConfig REQUIRED)
pkg_check_modules(MAVSDK REQUIRED mavsdk)

include_directories(
  include
  ${MAVSDK_INCLUDE_DIRS}
)

link_directories(
  ${MAVSDK_LIBRARY_DIRS}
)

# Create executable (target name = vins_mono_node)
add_executable(vins_mono_node src/ov_ros_main.cpp)

ament_target_dependencies(vins_mono_node
  rclcpp
  nav_msgs
)

# Link against MAVSDK libraries
target_link_libraries(vins_mono_node
  ${MAVSDK_LIBRARIES}
)

install(TARGETS
  vins_mono_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
