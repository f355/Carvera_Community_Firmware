#pragma once

#include <cstdint>
#include <string_view>

#include "MachineModel.h"

struct FACTORY_SET {
  Machine MachineModel;
  char FuncSetting;
  char reserve1;
  char reserve2;
};

static_assert(sizeof(FACTORY_SET) == 4, "Unexpected factory settings record size");

class FactorySettings {
 public:
  enum class LineResult : uint8_t { ignored, applied, missing_pair, missing_value };

  explicit FactorySettings(FACTORY_SET& values) : values_(values) {}

  LineResult apply_line(std::string_view line);

 private:
  enum class Key : uint8_t { unknown, machine_model, a_axis_home, c_axis_home, atc, ce1_expand };

  static Key key_from_name(std::string_view name);
  void set_bit(uint8_t bit, bool enabled);

  FACTORY_SET& values_;
};
