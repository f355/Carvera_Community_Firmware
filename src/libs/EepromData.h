#ifndef EEPROM_DATA_H
#define EEPROM_DATA_H

#include <cstddef>

struct EEPROM_data {
    float TLO;
    float REFMZ;
    float TOOLMZ;
    float reserve;
    int TOOL;
    float perm_vars[20];
    bool tool_not_calibrated;
    int current_wcs;
    float WCScoord[6][4];
    float WCSrotation[6];
};

namespace eeprom_layout {

struct Z1Record {
    float TLO;
    float G54[3];
    float REFMZ;
    float TOOLMZ;
    float reserve;
    int TOOL;
    float G54AB[2];
};

static_assert(sizeof(EEPROM_data) == 228, "Unexpected Dev EEPROM layout");
static_assert(offsetof(EEPROM_data, REFMZ) == 4, "Unexpected Dev REFMZ offset");
static_assert(offsetof(EEPROM_data, TOOL) == 16, "Unexpected Dev TOOL offset");
static_assert(sizeof(Z1Record) == 40, "Unexpected Z1 EEPROM layout");
static_assert(offsetof(Z1Record, G54) == 4, "Unexpected Z1 G54 offset");
static_assert(offsetof(Z1Record, REFMZ) == 16, "Unexpected Z1 REFMZ offset");
static_assert(offsetof(Z1Record, TOOL) == 28, "Unexpected Z1 TOOL offset");

constexpr std::size_t runtime_record_size()
{
#if defined(MACHINE_Z1)
    return sizeof(Z1Record);
#else
    return sizeof(EEPROM_data);
#endif
}

constexpr bool valid_tool_number(int tool)
{
    return tool >= -1;
}

inline void decode_z1(const Z1Record &stored, EEPROM_data &runtime)
{
    runtime = {};
    runtime.TLO = stored.TLO;
    runtime.REFMZ = stored.REFMZ;
    runtime.TOOLMZ = stored.TOOLMZ;
    runtime.reserve = stored.reserve;
    runtime.TOOL = stored.TOOL;
    runtime.WCScoord[0][0] = stored.G54[0];
    runtime.WCScoord[0][1] = stored.G54[1];
    runtime.WCScoord[0][2] = stored.G54[2];
    runtime.WCScoord[0][3] = stored.G54AB[0];
}

inline void update_z1(const EEPROM_data &runtime, Z1Record &stored)
{
    stored.TLO = runtime.TLO;
    stored.G54[0] = runtime.WCScoord[0][0];
    stored.G54[1] = runtime.WCScoord[0][1];
    stored.G54[2] = runtime.WCScoord[0][2];
    stored.REFMZ = runtime.REFMZ;
    stored.TOOLMZ = runtime.TOOLMZ;
    stored.reserve = runtime.reserve;
    stored.TOOL = runtime.TOOL;
    stored.G54AB[0] = runtime.WCScoord[0][3];
}

} // namespace eeprom_layout

#endif
