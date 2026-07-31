/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "CANBus.h"

CANBus::CANBus(PinName rd, PinName td) : can(rd, td)
{
    this->tx_error = 0;
    this->rx_error = 0;
    this->ready    = false;
    this->freq     = 0;
    this->tx_count = 0;
    this->rx_count = 0;
}

bool CANBus::frequency(int hz)
{
    if (hz <= 0) return false;

    this->freq     = hz;
    this->tx_count = 0;
    this->rx_count = 0;
    this->tx_error = 0;
    this->rx_error = 0;

    this->can.reset();
    if (!this->can.frequency(this->freq)) return false;

    this->ready = true;
    return true;
}

bool CANBus::write(mbed::CANMessage msg)
{
    if (!this->ready) return false;

    if (this->can.write(msg)) {
        this->tx_count++;
        return true;
    }

    this->tx_error = this->can.tderror();
    return false;
}

bool CANBus::read(mbed::CANMessage &msg)
{
    if (!this->ready) return false;

    if (this->can.read(msg)) {
        this->rx_count++;
        return true;
    }

    this->rx_error = this->can.rderror();
    return false;
}
