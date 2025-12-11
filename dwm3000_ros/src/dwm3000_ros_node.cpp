#include "dwm3000_ros/dwm3000_ros.hpp"
#include "rclcpp/executors.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nodelib::Dwm3000Ros>();

  rclcpp::spin(node);
}
