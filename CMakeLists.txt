cmake_minimum_required(VERSION 3.5)
project(pose_to_tf)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(pose_to_tf pose_to_tf.cpp)

ament_auto_package()
