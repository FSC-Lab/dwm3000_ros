#include "dwm3000_ros/dwm3000_ros.hpp"

#include "UWBRange.pb.h"

#define FWD(...) static_cast<decltype(__VA_ARGS__)&&>(__VA_ARGS__)

namespace nodelib {

using std::string_literals::operator""s;

static constexpr std::int64_t kDefaultBaud = 115200;

Dwm3000Ros::Dwm3000Ros() : rclcpp::Node("dwm3000_ros") {
  declare_parameter("device", "/dev/ttyUSB0"s);
  declare_parameter("baud", kDefaultBaud);

  std::int64_t baud;
  std::string device;
  get_parameter("device", device);
  get_parameter("baud", baud);

  if (openPort(device, baud)) {
    RCLCPP_INFO(get_logger(), "Opened serial port: %s:%ld", device.c_str(),
                baud);
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port!");
  }
}

Dwm3000Ros::~Dwm3000Ros() = default;

bool Dwm3000Ros::openPort(const std::string& device, std::int64_t baud) {
  if (!serial_.open(device, baud) || !serial_.isOpen()) {
    serial_.clearCallback();
    return false;
  }

  range_pub_ = create_publisher<dm::UWBRange>("/uwb/range", 10);

  serial_.setCallback(std::bind_front(&Dwm3000Ros::parsingCallback, this));
  return true;
}

bool Dwm3000Ros::isOpen() const { return serial_.isOpen(); }

void Dwm3000Ros::parsingCallback(std::span<std::uint8_t const> data) {
  rx_buffer_.insert(rx_buffer_.end(), data.begin(), data.end());

  while (rx_buffer_.size() >= 2) {
    // Peek length (2 bytes, Little Endian)
    uint16_t msg_len = rx_buffer_[0] | (rx_buffer_[1] << 8);

    // Check if we have the full message (Header + Body)
    if (rx_buffer_.size() < 2 + msg_len) {
      break;  // Wait for more data
    }
    dwm3000_UWBRange in_msg = dwm3000_UWBRange_init_zero;

    stream_ = pb_istream_from_buffer(rx_buffer_.data() + 2, msg_len);

    if (!pb_decode(&stream_, dwm3000_UWBRange_fields, &in_msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1, "Decoding failed: %s",
                           PB_GET_ERROR(&stream_));
    } else {
      dm::UWBRange msg;
      msg.header.stamp = now();

      msg.source_id = in_msg.source_id;
      std::uint64_t stamp = in_msg.stamp;
      msg.range = in_msg.range;
      msg.destination_id = in_msg.destination_id;
      msg.num_units = in_msg.num_units;
      msg.tof = in_msg.tof;
      range_pub_->publish(msg);
    }

    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 2 + msg_len);
  }
}
}  // namespace nodelib
