/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "CANopen.h"

#include "libs/Kernel.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "libs/PublicDataRequest.h"
#include "libs/utils.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Gcode.h"
#include "us_ticker_api.h"

#include <string.h>

#define canopen_checksum          CHECKSUM("canopen")
#define can_enable_checksum       CHECKSUM("can_enable")
#define can_bitrate_checksum      CHECKSUM("can_bitrate")
#define can_node_id_checksum      CHECKSUM("can_node_id")
#define slave_node_id_checksum    CHECKSUM("slave_node_id")
#define can_heartbeat_ms_checksum CHECKSUM("can_heartbeat_ms")
#define can_role_checksum         CHECKSUM("can_role")

#define status_checksum           CHECKSUM("status")
#define send_frame_checksum       CHECKSUM("send_frame")

// The CAN transceiver hangs off CAN2.
#define CAN_RD_PIN  P0_4
#define CAN_TD_PIN  P0_5

// SDO client/server COB-ID bases, and the NMT error-control base the heartbeat
// is published on.
#define SDO_RX_COBID    0x600
#define SDO_TX_COBID    0x580
#define HEARTBEAT_COBID 0x700

// Every SDO exchange gets the same budget.
#define SDO_TIMEOUT_US  50000UL

// The bus test drives the outputs from the inputs, holds them, then clears.
#define BUS_TEST_HOLD_US 10000000UL

// Manufacturer object dictionary entries this module knows about.
#define OD_DIGITAL_INPUTS   0x6000
#define OD_DIGITAL_OUTPUTS  0x6001
#define OD_NODE_CONFIG      0x6900

// Baud codes as the slaves number them.
static const uint32_t baud_codes[8] = {
    10000, 20000, 50000, 125000, 250000, 500000, 800000, 1000000
};

CANopen::CANopen()
{
    this->last_rx.len    = 8;
    memset(this->last_rx.data, 0, sizeof(this->last_rx.data));
    this->node_id           = 1;
    this->heartbeat_ms      = 1000;
    this->bitrate           = 1000000;
    this->enabled           = false;
    this->bus_ready         = false;
    this->last_heartbeat_us = 0;
    this->can               = NULL;
    this->last_rx.type      = CANData;
    this->last_rx.format    = CANStandard;
    this->last_rx.id        = 0;
    this->have_last_rx      = false;
    this->master            = true;
}

void CANopen::on_module_loaded()
{
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_HALT);

    this->configure();
}

void CANopen::configure()
{
    this->enabled       = THEKERNEL->config->value(canopen_checksum, can_enable_checksum)->by_default(true)->as_bool();
    this->bitrate       = THEKERNEL->config->value(canopen_checksum, can_bitrate_checksum)->by_default(1000000)->as_number();
    this->node_id       = THEKERNEL->config->value(canopen_checksum, can_node_id_checksum)->by_default(10)->as_number();
    this->slave_node_id = THEKERNEL->config->value(canopen_checksum, slave_node_id_checksum)->by_default(1)->as_number();
    this->heartbeat_ms  = THEKERNEL->config->value(canopen_checksum, can_heartbeat_ms_checksum)->by_default(1000)->as_number();

    // Anything that is not spelled "slave" is a master.
    string role = THEKERNEL->config->value(canopen_checksum, can_role_checksum)->by_default("master")->as_string();
    this->master = (role.compare("slave") != 0);

    if (this->enabled) {
        this->start();
    } else {
        this->stop();
    }
}

bool CANopen::start()
{
    if (this->can == NULL) {
        this->can = new CANBus(CAN_RD_PIN, CAN_TD_PIN);
    }

    this->bus_ready = this->can->frequency(this->bitrate);
    if (!this->bus_ready) {
        THEKERNEL->streams->printf("CANopen: failed to start CAN controller at %lu bps\n", this->bitrate);
        return false;
    }

    this->last_heartbeat_us = us_ticker_read();
    this->have_last_rx = false;
    return true;
}

void CANopen::stop()
{
    this->bus_ready = false;
    if (this->can != NULL) {
        delete this->can;
        this->can = NULL;
    }
}

void CANopen::drain_rx()
{
    mbed::CANMessage msg;
    while (this->can->read(msg)) {
        this->last_rx = msg;
        this->have_last_rx = true;
    }
}

void CANopen::send_heartbeat(uint32_t now)
{
    if (now - this->last_heartbeat_us < this->heartbeat_ms * 1000) return;
    this->last_heartbeat_us = now;

    mbed::CANMessage msg;
    msg.id     = HEARTBEAT_COBID + this->slave_node_id;
    msg.data[0]= this->master ? 5 : 127;
    msg.len    = 1;
    msg.format = CANStandard;
    msg.type   = CANData;
    this->can->write(msg);
}

void CANopen::on_main_loop(void *argument)
{
    if (!this->enabled) return;
    if (!this->bus_ready) return;
    if (this->can == NULL) return;

    this->drain_rx();

    uint32_t now = us_ticker_read();
    if (this->heartbeat_ms == 0) return;
    this->send_heartbeat(now);
}

void CANopen::on_halt(void *argument)
{
    if (argument != nullptr) {
        this->have_last_rx = false;
    }
}

bool CANopen::await_frame(uint32_t id, mbed::CANMessage *msg, uint32_t timeout_us)
{
    uint32_t start = us_ticker_read();

    for (;;) {
        while (!this->can->read(*msg)) {
            if (us_ticker_read() - start >= timeout_us) return false;
        }

        if (msg->id == id) return true;

        // Somebody else's traffic. Remember it and keep waiting for ours.
        this->last_rx = *msg;
        this->have_last_rx = true;
    }
}

bool CANopen::sdo_read(uint8_t node, uint16_t index, uint8_t sub,
                       uint32_t *value, uint8_t *len, uint32_t timeout_us)
{
    if (!this->bus_ready || this->can == NULL) return false;

    mbed::CANMessage req;
    req.id      = SDO_RX_COBID + node;
    req.data[0] = 0x40;                 // initiate upload
    req.data[1] = index & 0xff;
    req.data[2] = index >> 8;
    req.data[3] = sub;
    req.data[4] = 0;
    req.data[5] = 0;
    req.data[6] = 0;
    req.data[7] = 0;
    req.len     = 8;
    req.format  = CANStandard;
    req.type    = CANData;

    if (!this->can->write(req)) return false;

    mbed::CANMessage rsp;
    rsp.len    = 8;
    rsp.format = CANStandard;
    rsp.type   = CANData;
    rsp.id     = 0;
    memset(rsp.data, 0, sizeof(rsp.data));

    if (!this->bus_ready || this->can == NULL) return false;
    if (!this->await_frame(SDO_TX_COBID + node, &rsp, timeout_us)) return false;

    // Must be an upload response, and expedited.
    if ((rsp.data[0] & 0xe0) != 0x40) return false;
    if ((rsp.data[0] & 0x02) == 0) return false;

    uint8_t n = (rsp.data[0] & 0x01) ? 4 - ((rsp.data[0] & 0x0f) >> 2) : 4;

    *value = 0;
    memcpy(value, &rsp.data[4], n);
    *len = n;
    return true;
}

bool CANopen::sdo_write(uint8_t node, uint16_t index, uint8_t sub,
                        uint32_t value, uint8_t size, uint32_t timeout_us)
{
    if (!this->bus_ready || this->can == NULL) return false;
    if (size - 1 >= 4) return false;

    mbed::CANMessage req;
    req.id      = SDO_RX_COBID + node;
    req.data[0] = 0x23 | ((4 - size) << 2);   // initiate expedited download
    req.data[1] = index & 0xff;
    req.data[2] = index >> 8;
    req.data[3] = sub;
    req.data[4] = 0;
    req.data[5] = 0;
    req.data[6] = 0;
    req.data[7] = 0;
    memcpy(&req.data[4], &value, size);
    req.len     = 8;
    req.format  = CANStandard;
    req.type    = CANData;

    if (!this->can->write(req)) return false;

    mbed::CANMessage rsp;
    rsp.len    = 8;
    rsp.format = CANStandard;
    rsp.type   = CANData;
    rsp.id     = 0;
    memset(rsp.data, 0, sizeof(rsp.data));

    if (!this->bus_ready || this->can == NULL) return false;
    if (!this->await_frame(SDO_TX_COBID + node, &rsp, timeout_us)) return false;

    return rsp.data[0] == 0x60;               // download response
}

uint8_t CANopen::addressed_node(Gcode *gcode)
{
    if (gcode != NULL && gcode->has_letter('N')) {
        uint32_t n = gcode->get_value('N');
        if (n - 1 < 127) return n & 0xff;
    }
    return this->slave_node_id;
}

// The configuration and the link's state, as M940 prints them.
void CANopen::report(StreamOutput *stream)
{
    stream->printf("CANopen %s\n", this->enabled ? "enabled" : "disabled");
    if (!this->enabled) return;

    stream->printf("  Bitrate: %lu\n", this->bitrate);
    stream->printf("  Node ID: %u\n", this->node_id);
    stream->printf("  Role: %s\n", this->master ? "master" : "slave");
    stream->printf("  Heartbeat: %lu ms\n", this->heartbeat_ms);
    stream->printf("  Bus ready: %s\n", this->bus_ready ? "yes" : "no");

    if (!this->bus_ready || this->can == NULL) return;
    stream->printf("  TX: %lu RX: %lu\n", this->can->get_tx_count(), this->can->get_rx_count());

    if (!this->have_last_rx) return;
    stream->printf("  Last RX ID: 0x%03X LEN:%d\n", this->last_rx.id, this->last_rx.len);
}

void CANopen::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);

    if (!gcode->has_m) return;

    if (gcode->m == 940) {
        this->report(gcode->stream);
        gcode->stream->printf("  Last heartbeat at: %lu us\n", this->last_heartbeat_us);
        gcode->stream->printf("ok\n");
        return;
    }

    if (gcode->m == 942) {
        // Loop the slave's inputs onto its outputs for a while, then clear.
        uint32_t value = 0;
        uint8_t  len   = 0;
        gcode->stream->printf("start Can testing......\n");
        if (this->sdo_read(1, OD_DIGITAL_INPUTS, 1, &value, &len, SDO_TIMEOUT_US)) {
            this->sdo_write(1, OD_DIGITAL_OUTPUTS, 1, value, 4, SDO_TIMEOUT_US);
        }
        safe_delay_us(BUS_TEST_HOLD_US);
        this->sdo_write(1, OD_DIGITAL_OUTPUTS, 1, 0, 4, SDO_TIMEOUT_US);
        gcode->stream->printf("Can test finish......\n");
        return;
    }

    if (gcode->m != 941) return;

    if (!this->enabled || !this->bus_ready || this->can == NULL) {
        gcode->stream->printf("error: CANopen disabled\n");
        return;
    }

    if ((gcode->subcode & 0xf) == 0) {
        // Raw frame: X is the id, L the length, A..H the bytes.
        if (!gcode->has_letter('X')) {
            gcode->stream->printf("error: missing X (id)\n");
            return;
        }
        uint32_t id = gcode->get_value('X');

        uint8_t data[8];
        memset(data, 0, sizeof(data));

        uint8_t len = 0;
        if (gcode->has_letter('L')) {
            len = gcode->get_value('L');
            if (len > 7) len = 8;
            for (uint8_t i = 0; i < len; i++) {
                if (gcode->has_letter('A' + i)) {
                    data[i] = gcode->get_value('A' + i);
                }
            }
        }

        mbed::CANMessage msg;
        msg.id     = id;
        msg.len    = len;
        msg.format = CANStandard;
        msg.type   = CANData;
        memcpy(msg.data, data, len);

        if (!this->can->write(msg)) {
            gcode->stream->printf("error: send failed\n");
            return;
        }
        gcode->stream->printf("ok\n");
        return;
    }

    uint8_t  node  = this->addressed_node(gcode);
    uint32_t value = 0;
    uint8_t  len   = 0;

    switch (gcode->subcode & 0xf) {
        case 1:
            if (!this->sdo_read(node, OD_NODE_CONFIG, 1, &value, &len, SDO_TIMEOUT_US)) {
                gcode->stream->printf("error: failed to read node id\n");
                return;
            }
            gcode->stream->printf("node %u id: %lu\n", node, value & 0xffff);
            break;

        case 2: {
            if (!gcode->has_letter('X')) {
                gcode->stream->printf("error: missing X (node id)\n");
                return;
            }
            uint32_t id = gcode->get_value('X');
            if (id - 1 > 126) {
                gcode->stream->printf("error: invalid node id\n");
                return;
            }
            if (!this->sdo_write(node, OD_NODE_CONFIG, 1, id, 2, SDO_TIMEOUT_US)) {
                gcode->stream->printf("error: failed to write node id\n");
                return;
            }
            gcode->stream->printf("node %u id set to %lu, power-cycle device to apply\n", node, id);
            break;
        }

        case 3: {
            if (!this->sdo_read(node, OD_NODE_CONFIG, 2, &value, &len, SDO_TIMEOUT_US)) {
                gcode->stream->printf("error: failed to read baud\n");
                return;
            }
            uint32_t code = value & 0xffff;
            if (code < 8 && baud_codes[code] != 0) {
                gcode->stream->printf("node %u baud code: %lu (%lu bps)\n", node, code, baud_codes[code]);
            } else {
                gcode->stream->printf("node %u baud code: %lu\n", node);
            }
            gcode->stream->printf("ok\n");
            return;
        }

        case 4: {
            if (!gcode->has_letter('X')) {
                gcode->stream->printf("error: missing X (baud)\n");
                return;
            }
            uint32_t want = gcode->get_value('X');
            uint32_t code = want;

            if (want >= 16) {
                // Given as a bitrate; translate to the code that carries it.
                if      (want == 10000)   code = 0;
                else if (want == 20000)   code = 1;
                else if (want == 50000)   code = 2;
                else if (want == 125000)  code = 3;
                else if (want == 250000)  code = 4;
                else if (want == 500000)  code = 5;
                else if (want == 800000)  code = 6;
                else if (want == 1000000) code = 7;
                else {
                    gcode->stream->printf("error: unsupported baud rate\n");
                    return;
                }
            }

            if (!this->sdo_write(node, OD_NODE_CONFIG, 2, code, 2, SDO_TIMEOUT_US)) {
                gcode->stream->printf("error: failed to set baud\n");
                return;
            }

            if (code < 8 && baud_codes[code] != 0) {
                gcode->stream->printf("node %u baud set to %lu bps (code %lu)\n", node, baud_codes[code], code);
            } else {
                gcode->stream->printf("node %u baud code set to %lu\n", node, code);
            }
            gcode->stream->printf("ok\n");
            return;
        }

        case 5:
            if (!this->sdo_read(node, OD_DIGITAL_OUTPUTS, 1, &value, &len, SDO_TIMEOUT_US)) {
                gcode->stream->printf("error: failed to read digital outputs\n");
                return;
            }
            gcode->stream->printf("node %u digital outputs: 0x%08lX\n", node, value);
            break;

        case 6: {
            if (!gcode->has_letter('X')) {
                gcode->stream->printf("error: missing X (output value)\n");
                return;
            }
            uint32_t out = gcode->get_value('X');
            if (!this->sdo_write(node, OD_DIGITAL_OUTPUTS, 1, out, 4, SDO_TIMEOUT_US)) {
                gcode->stream->printf("error: failed to set digital outputs\n");
                return;
            }
            gcode->stream->printf("node %u digital outputs set to 0x%08lX\n", node, out);
            break;
        }

        case 7:
            if (!this->sdo_read(node, OD_DIGITAL_INPUTS, 1, &value, &len, SDO_TIMEOUT_US)) {
                gcode->stream->printf("error: failed to read digital inputs\n");
                return;
            }
            gcode->stream->printf("node %u digital inputs: 0x%08lX\n", node, value);
            break;

        default:
            gcode->stream->printf("error: unsupported M941.%d\n", gcode->subcode & 0xf);
            return;
    }

    gcode->stream->printf("ok\n");
}

void CANopen::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if (!pdr->starts_with(canopen_checksum)) return;
    if (!pdr->second_element_is(status_checksum)) return;

    static canopen_status status;

    status.enabled      = this->enabled;
    status.bus_ready    = this->bus_ready;
    status.bitrate      = this->bitrate;
    status.node_id      = this->node_id;
    status.heartbeat_ms = this->heartbeat_ms;

    if (this->bus_ready && this->can != NULL) {
        status.tx_count = this->can->get_tx_count();
        status.rx_count = this->can->get_rx_count();
    } else {
        status.tx_count = 0;
        status.rx_count = 0;
    }

    status.have_last_rx = this->have_last_rx;
    if (this->have_last_rx) {
        status.last_rx = this->last_rx;
    }

    pdr->set_data_ptr(&status);
    pdr->set_taken();
}

void CANopen::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if (!pdr->starts_with(canopen_checksum)) return;
    if (!pdr->second_element_is(send_frame_checksum)) return;

    struct canopen_frame *frame = static_cast<struct canopen_frame *>(pdr->get_data_ptr());
    if (frame == NULL) return;

    if (!this->enabled) return;
    if (!this->bus_ready) return;
    if (this->can == NULL) return;

    uint8_t len = frame->len;
    if (len > 8) len = 8;

    mbed::CANMessage msg;
    msg.id     = frame->id;
    msg.len    = len;
    msg.type   = (CANType)frame->type;
    msg.format = (CANFormat)frame->format;
    memcpy(msg.data, frame->data, len);

    if (this->can->write(msg)) {
        pdr->set_taken();
    }
}
