#include "AccessorySwitchControl.h"

#include <string>

#include "Config.h"
#include "ConfigValue.h"
#include "Kernel.h"
#include "PublicData.h"
#include "SpindlePublicAccess.h"
#include "SwitchPublicAccess.h"
#include "checksumm.h"
#include "utils.h"

namespace {
constexpr uint16_t accessory_checksum = CHECKSUM("accessory");
constexpr uint16_t chip_clear_switch_checksum = CHECKSUM("chip_clear_switch");
constexpr uint16_t chip_clear_power_checksum = CHECKSUM("chip_clear_power");
constexpr uint16_t auto_blowing_switch_checksum = CHECKSUM("auto_blowing_switch");
constexpr uint16_t auto_blowing_power_checksum = CHECKSUM("auto_blowing_power");
constexpr uint16_t bed_cleaning_checksum = CHECKSUM("bed_cleaning");
constexpr uint16_t enable_checksum = CHECKSUM("enable");
constexpr uint16_t bed_cleaning_switch_checksum = CHECKSUM("bed_cleaning_switch");
constexpr uint16_t bed_cleaning_fan_switch_checksum = CHECKSUM("bed_cleaning_fan_switch");
constexpr uint16_t bed_cleaning_fan_power_checksum = CHECKSUM("bed_cleaning_fan_power");

bool spindle_is_running()
{
    spindle_status status{};
    return PublicData::get_value(pwm_spindle_control_checksum, get_spindle_status_checksum, &status) && status.state;
}
} // namespace

AccessorySwitchControl::AccessorySwitchControl()
    : chip_clear_switch_(read_switch_name(chip_clear_switch_checksum, "vacuum")),
      auto_blowing_switch_(read_switch_name(auto_blowing_switch_checksum, "nc")),
      bed_cleaning_switch_(read_switch_name(bed_cleaning_switch_checksum, "nc")),
      bed_cleaning_fan_switch_(read_switch_name(bed_cleaning_fan_switch_checksum, "nc")),
      chip_clear_power_(THEKERNEL->config->value(accessory_checksum, chip_clear_power_checksum)->as_number(-1.0F)),
      auto_blowing_power_(THEKERNEL->config->value(accessory_checksum, auto_blowing_power_checksum)->as_number(30.0F)),
      bed_cleaning_fan_power_(
          THEKERNEL->config->value(accessory_checksum, bed_cleaning_fan_power_checksum)->as_number(100.0F)),
      bed_cleaning_enabled_(THEKERNEL->config->value(bed_cleaning_checksum, enable_checksum)->as_bool(false))
{
}

void AccessorySwitchControl::apply_chip_clear_power() const
{
    if (chip_clear_power_ >= 0)
        set_power(chip_clear_switch_, chip_clear_power_);
}

uint16_t AccessorySwitchControl::read_switch_name(uint16_t setting, const char *fallback)
{
    const std::string name = THEKERNEL->config->value(accessory_checksum, setting)->as_string(fallback);
    return name.empty() || name == "nc" ? 0 : get_checksum(name);
}

bool AccessorySwitchControl::set_state(uint16_t name, bool state) const
{
    return name != 0 && PublicData::set_value(switch_checksum, name, state_checksum, &state);
}

bool AccessorySwitchControl::set_power(uint16_t name, float power) const
{
    if (name == 0)
        return false;
    pad_switch value{};
    value.state = true;
    value.value = power;
    return PublicData::set_value(switch_checksum, name, state_value_checksum, &value);
}

bool AccessorySwitchControl::bed_cleaning_owns(uint16_t name) const
{
    return name != 0 && THEKERNEL->is_bed_cleaning() &&
           (name == bed_cleaning_switch_ || name == bed_cleaning_fan_switch_);
}

void AccessorySwitchControl::set_bed_cleaning(bool enabled) const
{
    THEKERNEL->set_bed_cleaning(enabled);
    set_state(bed_cleaning_switch_, enabled);
    if (enabled) {
        set_power(bed_cleaning_fan_switch_, bed_cleaning_fan_power_);
    }
    else if (THEKERNEL->is_auto_blowing() && spindle_is_running()) {
        set_power(bed_cleaning_fan_switch_, THEKERNEL->get_auto_blowing_power());
    }
    else {
        set_power(bed_cleaning_fan_switch_, 0);
    }
}
