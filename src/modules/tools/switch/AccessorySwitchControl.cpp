#include "AccessorySwitchControl.h"

#include "Config.h"
#include "ConfigValue.h"
#include "Kernel.h"
#include "PublicData.h"
#include "SwitchPublicAccess.h"
#include "checksumm.h"
#include "utils.h"

#include <string>

namespace {
constexpr uint16_t accessory_checksum = CHECKSUM("accessory");
}

uint16_t accessory_switch::configured_name(uint16_t setting, const char* fallback)
{
    const std::string name = THEKERNEL->config->value(accessory_checksum, setting)->as_string(fallback);
    return name.empty() || name == "nc" ? 0 : get_checksum(name);
}

bool accessory_switch::set_state(uint16_t name, bool state)
{
    return name != 0 && PublicData::set_value(switch_checksum, name, state_checksum, &state);
}

bool accessory_switch::set_power(uint16_t name, float power)
{
    if(name == 0) return false;
    pad_switch value{};
    value.state = true;
    value.value = power;
    return PublicData::set_value(switch_checksum, name, state_value_checksum, &value);
}
