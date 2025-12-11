#ifndef DWM3000_ROS_DWM3000_ROS_HPP_
#define DWM3000_ROS_DWM3000_ROS_HPP_

#include <cstdint>
#include <string>

#include "dwm3000_msgs/msg/uwb_range.hpp"
#include "fsc_serial/fsc_serial.hpp"
#include "pb_decode.h"
#include "rclcpp/node.hpp"

namespace nodelib {

namespace dm = dwm3000_msgs::msg;

class Dwm3000Ros : public rclcpp::Node {
 public:
  Dwm3000Ros();
  ~Dwm3000Ros() override;

  bool openPort(const std::string& device, std::int64_t baud);

  [[nodiscard]] bool isOpen() const;

 private:
  void parsingCallback(std::span<std::uint8_t const> data);
  fsc::CallbackSerial serial_;
  rclcpp::Publisher<dm::UWBRange>::SharedPtr range_pub_;
  pb_istream_t stream_;

  std::vector<std::uint8_t> rx_buffer_;
};
}  // namespace nodelib

#endif  // DWM3000_ROS_DWM3000_ROS_HPP_
