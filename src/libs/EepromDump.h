#ifndef EEPROM_DUMP_H
#define EEPROM_DUMP_H

#include <array>
#include <cstddef>
#include <cstdint>

namespace eeprom_dump {

constexpr std::size_t row_size = 16;
constexpr std::size_t row_text_size = 4 + 1 + row_size * 2;

constexpr char hex_digit(uint8_t value) {
  return value < 10 ? static_cast<char>('0' + value) : static_cast<char>('A' + value - 10);
}

constexpr std::array<char, row_text_size + 1> format_row(uint16_t address, const std::array<uint8_t, row_size>& bytes) {
  std::array<char, row_text_size + 1> result{};
  for (std::size_t i = 0; i < 4; ++i) {
    const auto shift = static_cast<unsigned>((3 - i) * 4);
    result[i] = hex_digit(static_cast<uint8_t>((address >> shift) & 0x0F));
  }
  result[4] = ':';
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    result[5 + i * 2] = hex_digit(static_cast<uint8_t>(bytes[i] >> 4));
    result[6 + i * 2] = hex_digit(static_cast<uint8_t>(bytes[i] & 0x0F));
  }
  return result;
}

}  // namespace eeprom_dump

#endif
