#pragma once

#include <cstdint>

namespace accessory_switch {

uint16_t configured_name(uint16_t setting, const char* fallback);
bool set_state(uint16_t name, bool state);
bool set_power(uint16_t name, float power);
bool bed_cleaning_supported();
bool bed_cleaning_owns(uint16_t name);
void set_bed_cleaning(bool enabled);

}
