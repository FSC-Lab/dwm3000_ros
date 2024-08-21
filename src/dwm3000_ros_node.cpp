#include <chrono>
#include <iostream>
#include <thread>

#include "dwm3000_ros/dwm3000_ros.hpp"
#include "fsc_serial/fsc_serial.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "ros/spinner.h"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  ros::init(argc, argv, "dwm3000_ros_node");
  nodelib::Dwm3000Ros node;

  ros::spin();
}
