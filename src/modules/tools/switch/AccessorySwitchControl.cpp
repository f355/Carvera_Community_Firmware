#include "AccessorySwitchControl.h"

#include "Config.h"
#include "ConfigValue.h"
#include "Kernel.h"
#include "PublicData.h"
#include "SwitchPublicAccess.h"
#include "SpindlePublicAccess.h"
#include "checksumm.h"
#include "utils.h"

#include <string>

namespace {
constexpr uint16_t accessory_checksum = CHECKSUM("accessory");
constexpr uint16_t bed_cleaning_checksum = CHECKSUM("bed_cleaning");
constexpr uint16_t enable_checksum = CHECKSUM("enable");
constexpr uint16_t bed_cleaning_switch_checksum = CHECKSUM("bed_cleaning_switch");
constexpr uint16_t bed_cleaning_fan_switch_checksum = CHECKSUM("bed_cleaning_fan_switch");
constexpr uint16_t bed_cleaning_fan_power_checksum = CHECKSUM("bed_cleaning_fan_power");

bool spindle_is_running()
{
    spindle_status status{};
    return PublicData::get_value(
        pwm_spindle_control_checksum, get_spindle_status_checksum, &status) && status.state;
}
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

bool accessory_switch::bed_cleaning_supported()
{
    return THEKERNEL->config->value(
        bed_cleaning_checksum, enable_checksum)->as_bool(false);
}

bool accessory_switch::bed_cleaning_owns(uint16_t name)
{
    if(name == 0 || !THEKERNEL->is_bed_cleaning()) return false;
    return name == configured_name(bed_cleaning_switch_checksum, "nc") ||
        name == configured_name(bed_cleaning_fan_switch_checksum, "nc");
}

void accessory_switch::set_bed_cleaning(bool enabled)
{
    THEKERNEL->set_bed_cleaning(enabled);
    const uint16_t cleaning_switch = configured_name(bed_cleaning_switch_checksum, "nc");
    const uint16_t fan_switch = configured_name(bed_cleaning_fan_switch_checksum, "nc");

    set_state(cleaning_switch, enabled);
    if(enabled) {
        const float power = THEKERNEL->config->value(
            accessory_checksum, bed_cleaning_fan_power_checksum)->as_number(100.0F);
        set_power(fan_switch, power);
    } else if(THEKERNEL->is_auto_blowing() && spindle_is_running()) {
        set_power(fan_switch, THEKERNEL->get_auto_blowing_power());
    } else {
        set_power(fan_switch, 0);
    }
}
