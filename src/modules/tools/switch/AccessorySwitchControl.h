#pragma once

#include <cstdint>

class AccessorySwitchControl {
  public:
    AccessorySwitchControl();

    uint16_t chip_clear_switch() const
    {
        return chip_clear_switch_;
    }
    uint16_t auto_blowing_switch() const
    {
        return auto_blowing_switch_;
    }
    uint16_t bed_cleaning_fan_switch() const
    {
        return bed_cleaning_fan_switch_;
    }
    float default_auto_blowing_power() const
    {
        return auto_blowing_power_;
    }

    bool set_state(uint16_t name, bool state) const;
    bool set_power(uint16_t name, float power) const;
    bool bed_cleaning_supported() const
    {
        return bed_cleaning_enabled_;
    }
    bool bed_cleaning_owns(uint16_t name) const;
    void set_bed_cleaning(bool enabled) const;

  private:
    static uint16_t read_switch_name(uint16_t setting, const char *fallback);

    uint16_t chip_clear_switch_;
    uint16_t auto_blowing_switch_;
    uint16_t bed_cleaning_switch_;
    uint16_t bed_cleaning_fan_switch_;
    float auto_blowing_power_;
    float bed_cleaning_fan_power_;
    bool bed_cleaning_enabled_;
};
