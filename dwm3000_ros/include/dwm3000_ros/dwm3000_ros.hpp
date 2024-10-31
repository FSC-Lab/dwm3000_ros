#ifndef DWM3000_ROS_DWM3000_ROS_HPP_
#define DWM3000_ROS_DWM3000_ROS_HPP_

#include <cstdint>
#include <memory>
#include <string>

namespace nodelib {

class Dwm3000Ros {
 public:
  Dwm3000Ros();
  ~Dwm3000Ros();

  bool openPort(const std::string &device, std::int64_t baud);

  [[nodiscard]] bool isOpen() const;

 private:
  struct Impl;

  std::unique_ptr<Impl> pimpl_;
};
}  // namespace nodelib

#endif  // DWM3000_ROS_DWM3000_ROS_HPP_
