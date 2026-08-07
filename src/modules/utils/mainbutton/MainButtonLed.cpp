#include "MainButtonLed.h"

#include "LPC17xx.h"

namespace {

#define LED_NOP1 __NOP();
#define LED_NOP2 LED_NOP1 LED_NOP1
#define LED_NOP4 LED_NOP2 LED_NOP2
#define LED_NOP16 LED_NOP4 LED_NOP4 LED_NOP4 LED_NOP4
#define LED_NOP32 LED_NOP16 LED_NOP16
#define LED_NOP64 LED_NOP32 LED_NOP32
#define LED_HIGH_ONE LED_NOP64 LED_NOP16
#define LED_HIGH_ZERO LED_NOP32
#define LED_LOW LED_NOP64 LED_NOP4 LED_NOP2

#if defined(MACHINE_FAMILY_Z1)
constexpr uint32_t led_pin_mask = 1UL << 22;

__attribute__((always_inline)) inline void write_bit(bool one)
{
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    LPC_GPIO0->FIOSET = led_pin_mask;
    if (one) {
        LED_HIGH_ONE
    }
    else {
        LED_HIGH_ZERO
    }
    LPC_GPIO0->FIOCLR = led_pin_mask;
    if (interrupt_mask == 0U)
        __enable_irq();
    LED_LOW
}
#else
constexpr uint32_t led_pin_mask = 1UL << 15;

__attribute__((always_inline)) inline void write_bit(bool one)
{
    LPC_GPIO1->FIOSET = led_pin_mask;
    if (one) {
        LED_HIGH_ONE
    }
    else {
        LED_HIGH_ZERO
    }
    LPC_GPIO1->FIOCLR = led_pin_mask;
    LED_LOW
}
#endif

__attribute__((always_inline)) inline void write_byte(uint8_t value)
{
    for (uint8_t bit = 0; bit < 8; ++bit)
        write_bit((value & (0x80U >> bit)) != 0U);
}

} // namespace

void MainButtonLed::set_all(Color color) const
{
    Colors colors;
    colors.fill(color);
    write(colors, false);
}

void MainButtonLed::set_number(Color front, Color back, uint8_t number, bool row) const
{
    const uint8_t maximum = row ? 5 : 3;
    if (number == 0 || number > maximum)
        return;

    Colors colors;
    colors.fill(back);
    for (uint8_t index = 0; index < colors.size(); ++index) {
        const bool selected = row ? index < number : index % 2 == 0 && index / 2 < number;
        if (selected)
            colors[index] = front;
    }
    write(colors, true);
}

void MainButtonLed::write(const Colors &colors, bool exclusive) const
{
#if defined(MACHINE_FAMILY_Z1)
    (void)exclusive;
    constexpr uint32_t frame_priority_mask = 3U << (8U - __NVIC_PRIO_BITS);
    const uint32_t previous_priority_mask = __get_BASEPRI();
    if (previous_priority_mask == 0U || previous_priority_mask > frame_priority_mask)
        __set_BASEPRI(frame_priority_mask);

    constexpr uint8_t group_order[] = {0, 1, 2, 3, 4, 1};
    for (uint8_t group : group_order) {
        for (uint8_t led = 0; led < 3; ++led) {
            write_byte(colors[group].green);
            write_byte(colors[group].red);
            write_byte(colors[group].blue);
        }
    }

    __set_BASEPRI(previous_priority_mask);
#else
    if (exclusive) {
        __disable_irq();
    }
    else {
        NVIC_DisableIRQ(TIMER0_IRQn);
        NVIC_DisableIRQ(TIMER1_IRQn);
    }

    for (const Color &color : colors) {
        write_byte(color.red);
        write_byte(color.green);
        write_byte(color.blue);
    }

    if (exclusive) {
        __enable_irq();
    }
    else {
        NVIC_EnableIRQ(TIMER0_IRQn);
        NVIC_EnableIRQ(TIMER1_IRQn);
    }
#endif
}
