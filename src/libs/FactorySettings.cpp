#include "FactorySettings.h"

#include <limits>

namespace factory_settings {
namespace {

bool whitespace(char value) { return value == ' ' || value == '\t' || value == '\r' || value == '\n'; }

Key key_from_name(std::string_view name) {
  if (name == "Machine_Model") return Key::machine_model;
  if (name == "A_Axis_home_enable") return Key::a_axis_home;
  if (name == "C_Axis_home_enable") return Key::c_axis_home;
  if (name == "Atc_enable") return Key::atc;
  if (name == "CE1_Expand") return Key::ce1_expand;
  return Key::unknown;
}

void set_bit(char& value, uint8_t bit, bool enabled) {
  uint8_t bits = static_cast<uint8_t>(value);
  if (enabled) {
    bits |= static_cast<uint8_t>(1U << bit);
  } else {
    bits &= static_cast<uint8_t>(~(1U << bit));
  }
  value = static_cast<char>(bits);
}

}  // namespace

ParseResult parse(std::string_view line, Setting& setting) {
  std::size_t cursor = 0;
  while (cursor < line.size() && whitespace(line[cursor])) ++cursor;
  if (cursor == line.size() || line[cursor] == '#') return ParseResult::ignored;

  const std::size_t key_begin = cursor;
  while (cursor < line.size() && !whitespace(line[cursor])) ++cursor;
  if (cursor == line.size()) return ParseResult::invalid;
  const Key key = key_from_name(line.substr(key_begin, cursor - key_begin));

  while (cursor < line.size() && whitespace(line[cursor])) ++cursor;
  if (cursor == line.size() || line[cursor] == '#') return ParseResult::invalid;

  unsigned value = 0;
  bool have_digit = false;
  while (cursor < line.size() && line[cursor] >= '0' && line[cursor] <= '9') {
    have_digit = true;
    value = value * 10U + static_cast<unsigned>(line[cursor] - '0');
    if (value > std::numeric_limits<uint8_t>::max()) return ParseResult::invalid;
    ++cursor;
  }
  if (!have_digit) return ParseResult::invalid;

  while (cursor < line.size() && whitespace(line[cursor])) ++cursor;
  if (cursor != line.size() && line[cursor] != '#') return ParseResult::invalid;

  setting = Setting{key, static_cast<uint8_t>(value)};
  return ParseResult::setting;
}

bool apply(FACTORY_SET& factory, const Setting& setting) {
  switch (setting.key) {
    case Key::machine_model:
      factory.MachineModel = static_cast<char>(setting.value);
      return true;
    case Key::a_axis_home:
      set_bit(factory.FuncSetting, 0, setting.value == 1);
      return true;
    case Key::c_axis_home:
      set_bit(factory.FuncSetting, 1, setting.value == 1);
      return true;
    case Key::atc:
      set_bit(factory.FuncSetting, 2, setting.value == 1);
      return true;
    case Key::ce1_expand:
      set_bit(factory.FuncSetting, 3, setting.value == 1);
      return true;
    case Key::unknown:
      return false;
  }
  return false;
}

}  // namespace factory_settings
