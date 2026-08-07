#pragma once

#include <cstdint>
#include <string_view>

struct FACTORY_SET {
  char MachineModel;
  char FuncSetting;
  char reserve1;
  char reserve2;
};

namespace factory_settings {

enum class Key : uint8_t {
  unknown,
  machine_model,
  a_axis_home,
  c_axis_home,
  atc,
  ce1_expand,
};

struct Setting {
  Key key;
  uint8_t value;
};

enum class ParseResult : uint8_t { ignored, setting, invalid };

ParseResult parse(std::string_view line, Setting& setting);
bool apply(FACTORY_SET& factory, const Setting& setting);

}  // namespace factory_settings
