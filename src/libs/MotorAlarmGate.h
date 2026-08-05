#pragma once

#include <cstdint>

namespace motor_alarm {

constexpr bool ready(bool driver_powered, uint32_t power_on_us, uint32_t now_us, uint32_t settle_us) {
  if (settle_us == 0) return true;
  return driver_powered && now_us - power_on_us >= settle_us;
}

}  // namespace motor_alarm
