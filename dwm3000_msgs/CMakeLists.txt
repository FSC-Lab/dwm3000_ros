cmake_minimum_required(VERSION 3.20.0)
project(dwm3000_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/UWBRange.msg" DEPENDENCIES
                           std_msgs)

ament_package()
