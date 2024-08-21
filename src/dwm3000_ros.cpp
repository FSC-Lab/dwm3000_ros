#include <utility>

#include "dwm3000_ros/dwm3000_ros.hpp"
#include "dwm3000_ros/utils.hpp"
#include "ros/publisher.h"
#include "ros/ros.h"

#include "dwm3000_ros/UWBRange.h"
#include "dwm3000_ros/utils.hpp"
#include "fsc_serial/fsc_serial.hpp"

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
    using utils::Unpack;
    if (data[0] != 0xFE || size < 24) {
      return;
    }
    // TODO(Hs293Go): Finalize message definition and define size/offset enums
    dwm3000_ros::UWBRange msg;
    msg.header.stamp = ros::Time::now();

    const unsigned char *packet_start = &data[0];
    int length = Unpack<std::uint8_t>(packet_start + 1);
    msg.sequence_number = Unpack<std::uint8_t>(packet_start + 2);
    msg.source_id = Unpack<std::uint8_t>(packet_start + 3);
    std::uint64_t stamp = Unpack<std::uint64_t>(packet_start + 4);
    msg.range = Unpack<double>(packet_start + 12);
    std::uint32_t crc32 = Unpack<std::uint32_t>(packet_start + 20);
    range_pub.publish(msg);
  }

  ros::NodeHandle nh;
  fsc::CallbackSerial serial;
  ros::Publisher range_pub;
};

Dwm3000Ros::Dwm3000Ros() : pimpl_(std::make_unique<Impl>()) {}

Dwm3000Ros::~Dwm3000Ros() = default;

bool Dwm3000Ros::openPort(const std::string &device, std::int64_t baud) {
  return pimpl_->openPort(device, baud);
}

bool Dwm3000Ros::isOpen() { return pimpl_->serial.isOpen(); }

} // namespace nodelib
