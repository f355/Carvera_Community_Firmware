#pragma once

namespace atc_profile {

struct Defaults {
    float toolrack_z;
    float toolrack_offset_x;
    float toolrack_offset_y;
    int lowest_tool_number;
    float rotation_offset_x;
    float rotation_offset_y;
    float rotation_offset_z;
    float clearance_x;
    float clearance_y;
    float clearance_z;
};

constexpr Defaults legacy_defaults(bool carvera)
{
    return carvera
        ? Defaults{-105.0F, 356.0F, 0.0F, 0,
                   -8.0F, 37.5F, 22.5F, -75.0F, -3.0F, -3.0F}
        : Defaults{-108.0F, 126.0F, 196.0F, 1,
                   30.0F, 82.5F, 23.0F, -5.0F, -21.0F, -5.0F};
}

} // namespace atc_profile
