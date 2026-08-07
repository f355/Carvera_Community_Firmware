#pragma once

#include <cstdint>

namespace player_link {

constexpr uint16_t read_u16(const uint8_t* data) { return (static_cast<uint16_t>(data[0]) << 8) | data[1]; }

constexpr uint32_t read_u32(const uint8_t* data) {
  return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | data[3];
}

template <typename Byte>
constexpr void write_u16(Byte* data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value >> 8);
  data[1] = static_cast<uint8_t>(value);
}

template <typename Byte>
constexpr void write_u32(Byte* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value >> 24);
  data[1] = static_cast<uint8_t>(value >> 16);
  data[2] = static_cast<uint8_t>(value >> 8);
  data[3] = static_cast<uint8_t>(value);
}

}  // namespace player_link
