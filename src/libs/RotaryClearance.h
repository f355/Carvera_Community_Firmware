#pragma once

namespace rotary_clearance {

constexpr bool active(bool rotary_installed, float clearance_y_min)
{
    return rotary_installed && clearance_y_min == clearance_y_min;
}

constexpr float effective_y_min(bool rotary_installed, float configured_y_min,
                                float clearance_y_min)
{
    if (!active(rotary_installed, clearance_y_min)) return configured_y_min;
    if (configured_y_min != configured_y_min) return clearance_y_min;
    return clearance_y_min > configured_y_min ? clearance_y_min : configured_y_min;
}

constexpr unsigned y_before_x_homing_order(unsigned order, bool clearance_active)
{
    if (!clearance_active) return order;

    unsigned x_shift = 18;
    unsigned y_shift = 18;
    for (unsigned shift = 0; shift < 18; shift += 3) {
        const unsigned axis = (order >> shift) & 0x07U;
        if (axis == 1U) x_shift = shift;
        if (axis == 2U) y_shift = shift;
    }

    if (x_shift >= y_shift || y_shift == 18) return order;

    const unsigned axis_mask = (0x07U << x_shift) | (0x07U << y_shift);
    return (order & ~axis_mask) | (2U << x_shift) | (1U << y_shift);
}

} // namespace rotary_clearance
