#ifndef DWM3000_ROS_UTILS_HPP_
#define DWM3000_ROS_UTILS_HPP_
#include <cstdint>
#include <cstring>

namespace utils {
template <typename T> T Unpack(std::uint8_t const *in) {
  union {
    std::uint8_t in[sizeof(T)];
    T out;
  } unpacker;

  std::memcpy(unpacker.in, in, sizeof(T));

  return unpacker.out;
}
} // namespace utils

#endif // DWM3000_ROS_UTILS_HPP_
