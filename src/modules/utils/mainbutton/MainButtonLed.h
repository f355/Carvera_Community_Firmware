#ifndef MAINBUTTON_LED_H
#define MAINBUTTON_LED_H

#include <cstdint>

constexpr uint8_t mainbutton_led_scale(uint8_t channel, uint8_t brightness)
{
    const uint32_t scaled = (static_cast<uint32_t>(channel) * brightness + 52U) / 104U;
    return static_cast<uint8_t>(scaled > 255U ? 255U : scaled);
}

#if defined(MACHINE_Z1)

#include <array>

struct MainButtonLedColor {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

using MainButtonLedGroups = std::array<MainButtonLedColor, 5>;

void mainbutton_led_write_strip(const MainButtonLedGroups& groups);

#else

void mainbutton_led_write_strip(unsigned char R1, unsigned char G1, unsigned char B1, unsigned char R2,
                                unsigned char G2, unsigned char B2, unsigned char R3, unsigned char G3,
                                unsigned char B3, unsigned char R4, unsigned char G4, unsigned char B4,
                                unsigned char R5, unsigned char G5, unsigned char B5);

#endif

#endif
