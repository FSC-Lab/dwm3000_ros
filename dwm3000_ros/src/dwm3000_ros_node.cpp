#include "dwm3000_ros/dwm3000_ros.hpp"
#include "ros/init.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dwm3000_ros_node");
  nodelib::Dwm3000Ros node;

  ros::spin();
}
