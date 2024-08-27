#include <utility>

#include "dwm3000_ros/dwm3000_ros.hpp"
#include "dwm3000_ros/utils.hpp"
#include "ros/publisher.h"
#include "ros/ros.h"

#include "UWBRange.pb.h"
#include "dwm3000_ros/UWBRange.h"
#include "dwm3000_ros/utils.hpp"
#include "fsc_serial/fsc_serial.hpp"
#include "pb_decode.h"

#define FWD(...) static_cast<decltype(__VA_ARGS__) &&>(__VA_ARGS__)

namespace nodelib {

using namespace std::string_literals;

struct Dwm3000Ros::Impl {

  Impl() {
    ros::NodeHandle pnh("~");

    const auto device = pnh.param("device", "/dev/ttyUSB0"s);
    const auto baud = pnh.param("baud", 115200);

    if (openPort(device, baud)) {
      ROS_INFO("Opened serial port: %s:%d", device.c_str(), baud);
    } else {
      ROS_ERROR("Failed to open serial port!");
    }
  }

  bool openPort(const std::string &device, std::int64_t baud) {
    if (!serial.open(device, baud) || !serial.isOpen()) {
      serial.clearCallback();
      return false;
    }

    range_pub = nh.advertise<dwm3000_ros::UWBRange>("/uwb/range", 1);

    serial.setCallback(
        [this](auto &&... args) { parsingCallback(FWD(args)...); });
    return true;
  }

  void parsingCallback(std::uint8_t const *data, std::size_t size) {
    dwm3000_UWBRange in_msg = dwm3000_UWBRange_init_zero;

    stream = pb_istream_from_buffer(data, size);

    if (!pb_decode(&stream, dwm3000_UWBRange_fields, &in_msg)) {
      ROS_WARN_THROTTLE(1, "Decoding failed: %s", PB_GET_ERROR(&stream));
    }

    dwm3000_ros::UWBRange msg;
    msg.header.stamp = ros::Time::now();

    msg.source_id = in_msg.source_id;
    std::uint64_t stamp = in_msg.stamp;
    msg.range = in_msg.range;
    msg.destination_id = in_msg.destination_id;
    msg.num_units = in_msg.num_units;
    msg.tof = in_msg.tof;
    range_pub.publish(msg);
  }

  ros::NodeHandle nh;
  fsc::CallbackSerial serial;
  ros::Publisher range_pub;
  pb_istream_t stream;
};

Dwm3000Ros::Dwm3000Ros() : pimpl_(std::make_unique<Impl>()) {}

Dwm3000Ros::~Dwm3000Ros() = default;

bool Dwm3000Ros::openPort(const std::string &device, std::int64_t baud) {
  return pimpl_->openPort(device, baud);
}

bool Dwm3000Ros::isOpen() { return pimpl_->serial.isOpen(); }

} // namespace nodelib
