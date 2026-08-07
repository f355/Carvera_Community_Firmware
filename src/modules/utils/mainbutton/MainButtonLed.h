#ifndef MAINBUTTON_LED_H
#define MAINBUTTON_LED_H

#include <array>
#include <cstdint>

class MainButtonLed {
  public:
    struct Color {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };

    constexpr explicit MainButtonLed(uint8_t brightness = 104) : brightness_(brightness) {}

    void set_brightness(uint8_t brightness)
    {
        brightness_ = brightness;
    }
    constexpr uint8_t brightness() const
    {
        return brightness_;
    }
    constexpr uint8_t scale(uint8_t channel) const
    {
        const uint32_t scaled = (static_cast<uint32_t>(channel) * brightness_ + 52U) / 104U;
        return static_cast<uint8_t>(scaled > 255U ? 255U : scaled);
    }

    void set_all(Color color) const;
    void set_number(Color front, Color back, uint8_t number, bool row) const;

  private:
    using Colors = std::array<Color, 5>;
    void write(const Colors &colors, bool exclusive) const;

    uint8_t brightness_;
};

#endif
