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

struct ManualCoordinates {
    float parking_x;
    float parking_y;
    float probe_x;
    float probe_y;
    float probe_z;
};

constexpr Defaults legacy_defaults(bool carvera)
{
    return carvera
        ? Defaults{-105.0F, 356.0F, 0.0F, 0,
                   -8.0F, 37.5F, 22.5F, -75.0F, -3.0F, -3.0F}
        : Defaults{-108.0F, 126.0F, 196.0F, 1,
                   30.0F, 82.5F, 23.0F, -5.0F, -21.0F, -5.0F};
}

constexpr ManualCoordinates z1_manual_coordinates(float anchor_x, float anchor_y,
                                                   float toolrack_offset_x,
                                                   float toolrack_offset_y,
                                                   float toolrack_z)
{
    return ManualCoordinates{
        anchor_x + toolrack_offset_x + 132.0F,
        anchor_y + toolrack_offset_y,
        anchor_x + 181.0F,
        anchor_y + 181.0F,
        toolrack_z,
    };
}

} // namespace atc_profile
