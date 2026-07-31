/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "CAN.h"

#include <stdint.h>

// CAN controller wrapper with initialization and frame counters.
class CANBus
{
    public:
        CANBus(PinName rd, PinName td);

        // Configure the controller bitrate and update its ready state.
        bool frequency(int hz);

        bool write(mbed::CANMessage msg);
        bool read(mbed::CANMessage &msg);

        bool     is_ready() const     { return this->ready; }
        uint32_t get_frequency() const{ return this->freq; }
        uint32_t get_tx_count() const { return this->tx_count; }
        uint32_t get_rx_count() const { return this->rx_count; }

    private:
        mbed::CAN can;
        bool      ready;
        uint32_t  freq;
        uint32_t  tx_count;
        uint32_t  rx_count;
        uint8_t   tx_error;
        uint8_t   rx_error;
};
