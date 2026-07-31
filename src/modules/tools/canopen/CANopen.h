/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "libs/Module.h"
#include "libs/CANBus.h"
#include "CAN.h"

#include <stdint.h>

class Gcode;
class StreamOutput;

// A frame handed to this module through PublicData under canopen.send_frame.
struct canopen_frame {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
    uint8_t  type;
    uint8_t  format;
};

// What canopen.status reports back.
struct canopen_status {
    bool     enabled;
    bool     bus_ready;
    uint32_t bitrate;
    uint8_t  node_id;
    uint32_t heartbeat_ms;
    uint32_t tx_count;
    uint32_t rx_count;
    bool     have_last_rx;
    mbed::CANMessage last_rx;
};

class CANopen : public Module
{
    public:
        CANopen();

        void on_module_loaded();
        void on_main_loop(void *argument);
        void on_gcode_received(void *argument);
        void on_get_public_data(void *argument);
        void on_set_public_data(void *argument);
        void on_halt(void *argument);

    private:
        void report(StreamOutput *stream);
        void configure();
        bool start();
        void stop();
        void drain_rx();
        void send_heartbeat(uint32_t now);

        // Which node a command addresses: the N word if it carries a valid one,
        // otherwise the configured slave.
        uint8_t addressed_node(Gcode *gcode);

        // Expedited SDO. Both return false unless the bus is up and the peer
        // answers within timeout_us with a well-formed response.
        bool sdo_read (uint8_t node, uint16_t index, uint8_t sub,
                       uint32_t *value, uint8_t *len, uint32_t timeout_us);
        bool sdo_write(uint8_t node, uint16_t index, uint8_t sub,
                       uint32_t value, uint8_t size, uint32_t timeout_us);

        // Pump frames until one arrives with the wanted id, or time runs out.
        // Frames that are not the wanted one are kept as the last received.
        bool await_frame(uint32_t id, mbed::CANMessage *msg, uint32_t timeout_us);

        bool     enabled;
        bool     master;
        bool     bus_ready;
        uint32_t bitrate;
        uint8_t  node_id;
        uint8_t  slave_node_id;
        uint32_t heartbeat_ms;
        uint32_t last_heartbeat_us;
        CANBus  *can;
        mbed::CANMessage last_rx;
        bool     have_last_rx;
};
