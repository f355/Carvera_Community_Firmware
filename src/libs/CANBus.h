#pragma once

#if defined(MACHINE_Z1)

#include "CAN.h"

#include <cstddef>
#include <cstdint>

class CANBus
{
public:
    struct Statistics {
        uint32_t transmitted;
        uint32_t received;
        uint32_t rx_overflow;
        uint32_t tx_failed;
        uint32_t tx_timeout;
        uint8_t tx_errors;
        uint8_t rx_errors;
    };

    CANBus(PinName rd, PinName td);
    ~CANBus();

    bool start(uint32_t bitrate);
    void stop();
    bool write(const mbed::CANMessage& message, uint32_t timeout_us);
    bool read(mbed::CANMessage& message);

    bool is_ready() const { return ready; }
    uint32_t bitrate() const { return configured_bitrate; }
    Statistics statistics();

private:
    static constexpr uint8_t rx_capacity = 16;
    static constexpr uint8_t rx_storage_size = rx_capacity + 1;

    void on_receive();
    bool abort_tx(uint32_t timeout_us);
    bool wait_for_tx_idle(uint32_t timeout_us) const;
    bool wait_for_tx_complete(uint32_t timeout_us, bool& completed) const;

    mbed::CAN can;
    LPC_CAN_TypeDef* controller;
    mbed::CANMessage rx_queue[rx_storage_size];
    volatile uint8_t rx_head;
    volatile uint8_t rx_tail;
    volatile uint32_t rx_count;
    volatile uint32_t rx_overflow_count;
    uint32_t tx_count;
    uint32_t tx_failed_count;
    uint32_t tx_timeout_count;
    uint32_t configured_bitrate;
    bool ready;
};

#endif
