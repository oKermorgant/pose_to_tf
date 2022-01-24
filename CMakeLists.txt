cmake_minimum_required(VERSION 3.5)
project(pose_to_tf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(pose_to_tf pose_to_tf.cpp)
ament_target_dependencies(pose_to_tf rclcpp nav_msgs tf2_ros geometry_msgs)

install(TARGETS pose_to_tf DESTINATION lib/${PROJECT_NAME})

ament_package()
