/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <stdarg.h>
#include <algorithm>
using std::string;
#include "libs/Module.h"
#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "Ticker.h"
#include "SerialConsole.h"
#include "libs/RingBuffer.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "ATCHandlerPublicAccess.h"
#include "PlayerPublicAccess.h"
#include "modules/utils/player/Player.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "lpc17xx_gpdma.h"
#include "lpc17xx_uart.h"

// version definition
#define VERSION "1.0.15"

__attribute__((section("AHBSRAM0"), aligned(4))) char Serialbuff[544];
extern unsigned short crc_table[256];

__attribute__((section("AHBSRAM1"), aligned(4))) struct job_info job_info_block;

namespace {

constexpr uint32_t kRxRingSize = 0x220;
constexpr uint32_t kRxDmaBufferSize = 0x180;
constexpr uint32_t kTxRingSize = 0x10;

struct UartDmaBuffers {
    uint8_t tx_ring[kTxRingSize];
    uint8_t rx_ring[kRxRingSize];
    uint8_t rx_dma_buffer[kRxDmaBufferSize];
};

struct UartDmaState {
    uint32_t tx_head;
    uint32_t tx_tail;
    uint8_t tx_status;
    uint8_t tx_reserved[3];
    void (*tx_callback)();
    volatile uint32_t rx_write;
    volatile uint32_t rx_read;
    volatile uint32_t rx_dma_consumed;
    volatile uint8_t rx_active;
    uint8_t rx_reserved[3];
    volatile uint32_t rx_total;
    void (*rx_callback)();
};

__attribute__((section("AHBSRAM1"), aligned(4))) UartDmaBuffers uart_dma_buffers;
UartDmaState uart_dma_state;

__attribute__((noinline)) void start_uart_tx_dma()
{
    if (uart_dma_state.tx_status == 1 ||
        uart_dma_state.tx_head == uart_dma_state.tx_tail) {
        return;
    }

    const uint32_t transfer_size =
        uart_dma_state.tx_head > uart_dma_state.tx_tail
            ? uart_dma_state.tx_head - uart_dma_state.tx_tail
            : kTxRingSize - uart_dma_state.tx_tail;

    GPDMA_Channel_CFG_Type channel_config;
    channel_config.ChannelNum = 0;
    channel_config.TransferSize = transfer_size;
    channel_config.TransferWidth = 0;
    channel_config.SrcMemAddr = reinterpret_cast<uint32_t>(
        uart_dma_buffers.tx_ring + uart_dma_state.tx_tail);
    channel_config.DstMemAddr = 0;
    channel_config.TransferType = GPDMA_TRANSFERTYPE_M2P;
    channel_config.SrcConn = 0;
    channel_config.DstConn = GPDMA_CONN_UART2_Tx;
    channel_config.DMALLI = 0;
    GPDMA_Setup(&channel_config);

    LPC_GPDMACH0->DMACCConfig = 0;
    LPC_GPDMACH0->DMACCSrcAddr = reinterpret_cast<uint32_t>(
        uart_dma_buffers.tx_ring + uart_dma_state.tx_tail);
    LPC_GPDMACH0->DMACCDestAddr =
        reinterpret_cast<uint32_t>(&LPC_UART2->THR);
    LPC_GPDMACH0->DMACCLLI = 0;
    LPC_GPDMACH0->DMACCControl =
        GPDMA_DMACCxControl_TransferSize(transfer_size) |
        GPDMA_DMACCxControl_SI |
        GPDMA_DMACCxControl_I;
    LPC_GPDMACH0->DMACCConfig =
        GPDMA_DMACCxConfig_DestPeripheral(GPDMA_CONN_UART2_Tx) |
        GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_M2P) |
        GPDMA_DMACCxConfig_IE |
        GPDMA_DMACCxConfig_ITC |
        GPDMA_DMACCxConfig_E;
    uart_dma_state.tx_status = 1;
}

uint32_t dma_rx_size()
{
    uint32_t write = uart_dma_state.rx_write;
    const uint32_t read = uart_dma_state.rx_read;
    if (write < read) write += kRxRingSize;
    return write - read;
}

uint32_t dma_rx_push(const uint8_t* source, uint32_t size)
{
    const uint32_t available = kRxRingSize - dma_rx_size();
    if (size > available) size = available;
    if (size == 0) return 0;

    const uint32_t write = uart_dma_state.rx_write;
    const uint32_t first =
        std::min<uint32_t>(size, kRxRingSize - write);
    memcpy(uart_dma_buffers.rx_ring + write, source, first);
    memcpy(uart_dma_buffers.rx_ring, source + first, size - first);
    uart_dma_state.rx_write = (write + size) % kRxRingSize;
    return size;
}

uint32_t dma_rx_pop(uint8_t* destination, uint32_t size)
{
    const uint32_t used = dma_rx_size();
    if (size > used) size = used;
    if (size == 0) return 0;

    const uint32_t read = uart_dma_state.rx_read;
    const uint32_t first =
        std::min<uint32_t>(size, kRxRingSize - read);
    memcpy(destination, uart_dma_buffers.rx_ring + read, first);
    memcpy(destination + first, uart_dma_buffers.rx_ring, size - first);
    uart_dma_state.rx_read = (read + size) % kRxRingSize;
    return size;
}

void start_uart_rx_dma()
{
    LPC_GPDMACH1->DMACCConfig = 0;
    LPC_GPDMACH1->DMACCSrcAddr =
        reinterpret_cast<uint32_t>(&LPC_UART2->RBR);
    LPC_GPDMACH1->DMACCDestAddr =
        reinterpret_cast<uint32_t>(uart_dma_buffers.rx_dma_buffer);
    LPC_GPDMACH1->DMACCLLI = 0;
    LPC_GPDMACH1->DMACCControl =
        GPDMA_DMACCxControl_TransferSize(kRxDmaBufferSize) |
        GPDMA_DMACCxControl_DI |
        GPDMA_DMACCxControl_I;
    LPC_GPDMACH1->DMACCConfig =
        GPDMA_DMACCxConfig_SrcPeripheral(GPDMA_CONN_UART2_Rx) |
        GPDMA_DMACCxConfig_TransferType(GPDMA_TRANSFERTYPE_P2M) |
        GPDMA_DMACCxConfig_IE |
        GPDMA_DMACCxConfig_ITC;
    GPDMA_ChannelCmd(1, ENABLE);
}

void dma_rx_flush()
{
    NVIC_DisableIRQ(DMA_IRQn);
    for (volatile int delay = 0; delay < 10; ++delay) {
    }

    const uint32_t dma_position =
        LPC_GPDMACH1->DMACCDestAddr -
        reinterpret_cast<uint32_t>(uart_dma_buffers.rx_dma_buffer);
    if (dma_position != uart_dma_state.rx_dma_consumed) {
        const uint32_t count =
            dma_position - uart_dma_state.rx_dma_consumed;
        dma_rx_push(
            uart_dma_buffers.rx_dma_buffer +
                uart_dma_state.rx_dma_consumed,
            count);
        uart_dma_state.rx_total += count;
        uart_dma_state.rx_dma_consumed = dma_position;
    }
    NVIC_EnableIRQ(DMA_IRQn);
}

void initialize_uart_rx_dma()
{
    memset(&uart_dma_state, 0, sizeof(uart_dma_state));
    memset(uart_dma_buffers.rx_ring, 0, sizeof(uart_dma_buffers.rx_ring));
    memset(uart_dma_buffers.rx_dma_buffer, 0,
           sizeof(uart_dma_buffers.rx_dma_buffer));

    GPDMA_Init();
    if ((LPC_GPDMA->DMACConfig & GPDMA_DMACConfig_E) == 0) {
        LPC_GPDMA->DMACConfig = GPDMA_DMACConfig_E;
    }

    UART_FIFO_CFG_Type fifo_config;
    UART_FIFOConfigStructInit(&fifo_config);
    fifo_config.FIFO_DMAMode = ENABLE;
    UART_FIFOConfig(LPC_UART2, &fifo_config);

    start_uart_rx_dma();
    NVIC_SetPriority(DMA_IRQn, 5);
    NVIC_EnableIRQ(DMA_IRQn);
    uart_dma_state.rx_active = 1;
}

} // namespace

extern "C" void DMA_IRQHandler()
{
    constexpr uint32_t kTxChannelMask = 1U << 0;
    constexpr uint32_t kRxChannelMask = 1U << 1;
    const uint32_t terminal_count_status = LPC_GPDMA->DMACIntTCStat;
    const uint32_t error_status = LPC_GPDMA->DMACIntErrStat;

    if ((terminal_count_status & kTxChannelMask) != 0) {
        LPC_GPDMA->DMACIntTCClear = kTxChannelMask;
        uart_dma_state.tx_tail =
            (uart_dma_state.tx_tail + LPC_GPDMACH0->DMACCControl) &
            (kTxRingSize - 1);
        uart_dma_state.tx_status = 0;
        if (uart_dma_state.tx_head != uart_dma_state.tx_tail) {
            start_uart_tx_dma();
        } else if (uart_dma_state.tx_callback != nullptr) {
            uart_dma_state.tx_callback();
        }
    }

    if ((terminal_count_status & kRxChannelMask) != 0) {
        LPC_GPDMA->DMACIntTCClear = kRxChannelMask;
        for (volatile int delay = 0; delay < 10; ++delay) {
        }

        const uint32_t count =
            kRxDmaBufferSize - uart_dma_state.rx_dma_consumed;
        dma_rx_push(
            uart_dma_buffers.rx_dma_buffer +
                uart_dma_state.rx_dma_consumed,
            count);
        uart_dma_state.rx_total += count;
        uart_dma_state.rx_dma_consumed = 0;
        start_uart_rx_dma();
        if (uart_dma_state.rx_callback != nullptr) {
            uart_dma_state.rx_callback();
        }
    }

    if ((error_status & kTxChannelMask) != 0) {
        LPC_GPDMA->DMACIntErrClr = kTxChannelMask;
        uart_dma_state.tx_status = 2;
    }

    if ((error_status & kRxChannelMask) != 0) {
        LPC_GPDMA->DMACIntErrClr = kRxChannelMask;
        start_uart_rx_dma();
    }
}

static uint32_t last_frame_us;
static uint32_t last_version_us;

#define RX_SUPERVISION_US   10000000UL
#define VERSION_INTERVAL_US  5000000UL

// Serial reading module
// Treats every received line as a command and passes it ( via event call ) to the command dispatcher.
// The command dispatcher will then ask other modules if they can do something with it
SerialConsole::SerialConsole( PinName rx_pin, PinName tx_pin, int baud_rate ){
    this->serial = new mbed::Serial( rx_pin, tx_pin );
    this->serial->baud(baud_rate);
    initialize_uart_rx_dma();
}

// Called when the module has just been loaded
void SerialConsole::on_module_loaded() {
    query_flag = false;
    halt_flag = false;
    diagnose_flag = false;
	this->attach_irq(true);

    // We only call the command dispatcher in the main loop, nowhere else
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_IDLE);
    this->register_for_event(ON_SET_PUBLIC_DATA);

    // Add to the pack of streams kernel can call to, for example for broadcasting
    THEKERNEL->streams->append_stream(this);
}

void SerialConsole::attach_irq(bool enable_irq) {
    (void)enable_irq;
}

void SerialConsole::on_set_public_data(void *argument) {
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if(!pdr->starts_with(atc_handler_checksum)) return;

    if(pdr->second_element_is(set_serial_rx_irq_checksum)) {
        bool enable_irq = *static_cast<bool *>(pdr->get_data_ptr());
        this->attach_irq(enable_irq);
        pdr->set_taken();
    }
}


// Poll the DMA-backed receive ring and dispatch one complete frame.
void SerialConsole::on_serial_char_received() {
	uint8_t headerBuffer[2];
    uint32_t received = 0;
    uint32_t timeout_ms = 100000;	//100 ms
    uint32_t starttime = 0;
    
    if (ready())
	{
	    // wait head
	    starttime = us_ticker_read();
	    while ((received < 2) && ((us_ticker_read() - starttime) < timeout_ms) ) {
            if (ready()) {
	    		headerBuffer[0] = headerBuffer[1];
                char byte = _getc();
				received++;
	            headerBuffer[1] = byte;
	            if (received >= 2 && (headerBuffer[0] != ((HEADER >> 8) & 0xFF) || 
	                                 headerBuffer[1] != (HEADER & 0xFF))) {
	                received = 1;
	            }
	        } 
	    }
	    
	    if (received < 2){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive header\r\n", 0);
	    	return;
	    }
	    
	    // receive length	    
	    starttime = us_ticker_read();
	    while ((received < 4) && ((us_ticker_read() - starttime) < timeout_ms) ) {
            if (ready()) {
                char byte = _getc();
	        	Serialbuff[received] = byte;
	            received ++;
	        } 
	    }
	    
	    if (received < 4){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive length\r\n", 0);
	    	return;
	    }
	    
	    uint16_t data_len = (Serialbuff[2]<<8) | Serialbuff[3];
	    uint16_t total_len = 4 + data_len + 2; // header + data + crc + tail
	    
	    if (data_len > 513 || total_len > sizeof(Serialbuff)){
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive datalen error\r\n", 0);
	    	 return; 
	    }
	    
	    starttime = us_ticker_read();
        while (received < total_len) {
            if ((us_ticker_read() - starttime) > timeout_ms) {
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive data body\r\n", 0);
	    	return;
            }
            received += dma_rx_pop(
                reinterpret_cast<uint8_t *>(Serialbuff) + received,
                total_len - received);
            if (received < total_len) {
                dma_rx_flush();
	        }
	    }
	    
	    // check tail
	    uint16_t tail = (Serialbuff[total_len-2]<<8) | Serialbuff[total_len-1];
	    if (tail != FOOTER) {
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive footer\r\n", 0);
	    	return;
	    }
/*	    
	    // check CRC
	    uint16_t received_crc = (Serialbuff[total_len-4] << 8) | Serialbuff[total_len-3];
		uint16_t calculated_crc = crc16_ccitt((unsigned char *)&Serialbuff[2], data_len);
	    if (received_crc != calculated_crc) {
//	    	PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort receive wrong crc\r\n", 0);
	        return;
	    }
*/	    
        // A complete frame resets link supervision.
        last_frame_us = us_ticker_read();

		uint8_t cmdType = Serialbuff[4];
        switch(cmdType) {
            case PTYPE_CTRL_SINGLE: { 
                if(Serialbuff[5] == '?') {
	            	query_flag = true;
	        	}
	        	else if(Serialbuff[5] == 'X' - 'A' + 1) {
	            	halt_flag = true;
	        	}
	        	else if(THEKERNEL->is_feed_hold_enabled()) {
		            if(Serialbuff[5] == '!') { // safe pause
		                THEKERNEL->set_feed_hold(true);
		            }
		            else if(Serialbuff[5] == '~') { // safe resume
		                THEKERNEL->set_feed_hold(false);
		            }
		        }
                break;
            }
            case PTYPE_CTRL_MULTI: {
                struct SerialMessage message;
                message.message.assign(Serialbuff+5, data_len-3);
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                break;
            }
            case PTYPE_FILE_START: {
                struct SerialMessage message;
                message.message.assign(Serialbuff+5,data_len-3);
                message.stream = this;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
                break;
            }
            	
            case PTYPE_PLAY_DATA: {
                // Accept job lines carrying the expected line number.
                if (data_len < 10) break;
                uint32_t line_no = ((uint32_t)(uint8_t)Serialbuff[7]<<24) | ((uint32_t)(uint8_t)Serialbuff[8]<<16)
                                 | ((uint32_t)(uint8_t)Serialbuff[9]<<8)  |  (uint32_t)(uint8_t)Serialbuff[10];
                if (line_no != job_expected_line) {
                    job_line_error = 1;
                    break;
                }
                job_line_error = 0;

                // The payload aggregates several lines; split on LF.
                uint16_t payload = data_len - 9;
                uint32_t begin = 0;
                char line[0x82];
                for (uint32_t i = 0; i <= payload; i++) {
                    if (i != payload && Serialbuff[11 + i] != '\n') continue;

                    // A trailing newline does not add an extra empty line.
                    // Empty lines delimited by an actual newline are retained.
                    if (i == payload && i == begin) break;

                    memset(line, 0, sizeof(line));
                    if (i != begin) {
                        memcpy(line, Serialbuff + 11 + begin, (i - begin) + 1);
                        line[(i - begin) + 1] = '\0';
                    }
                    if (job_lines.count < job_lines.capacity) {
                        struct job_line *slot = &job_lines.slots[job_lines.head];
                        strncpy(slot->text, line, sizeof(slot->text) - 1);
                        slot->text[sizeof(slot->text) - 1] = 0;
                        slot->valid = 1;
                        job_lines.head = (job_lines.head + 1) % job_lines.capacity;
                        job_lines.count++;
                    } else {
                        THEKERNEL->streams->printf("Alarm:push queue error at line %lu\n", job_expected_line);
                    }
                    job_expected_line++;
                    begin = i + 1;
                }
                break;
            }

            case PTYPE_PLAY_GOTO_DATA: {
                memset(&job_info_block, 0, sizeof(job_info_block));
                memcpy(((unsigned char *)&job_info_block) + 1, Serialbuff + 2, data_len + 1);
                job_info_block.valid = 1;
                break;
            }

            case PTYPE_PLAY_VIEW: {
                // Pass two big-endian payload fields to the player.
                uint16_t a = (Serialbuff[5]<<8) | Serialbuff[6];
                uint32_t b = ((uint32_t)(uint8_t)Serialbuff[7]<<24) | ((uint32_t)(uint8_t)Serialbuff[8]<<16)
                           | ((uint32_t)(uint8_t)Serialbuff[9]<<8)  |  (uint32_t)(uint8_t)Serialbuff[10];
                PublicData::set_value( player_checksum, link_name_crc_checksum, 0, &a );
                PublicData::set_value( player_checksum, link_file_size_checksum, 0, &b );
            }
            /* no break: also reported under the message-type key below */

            // Report this message type.
            case 0xF4:
            case 0xF5:
                PublicData::set_value( player_checksum, link_cmd_checksum, 0, &cmdType );
                break;

            default:
            	break;
        }
	    
	}

    // Report a link timeout and restart the supervision interval.
    if (us_ticker_read() - last_frame_us > RX_SUPERVISION_US) {
        last_frame_us = us_ticker_read();
        StreamOutput::PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Ctrlboard did not receive a status response from the Mainboard (RX error)\r\n", 0);
    }
}

// Receives one framed packet without dispatching it, with a bounded wait.
int SerialConsole::receive_packet(SerialPacket* packet, int timeout_ms)
{
    (void)timeout_ms;
    constexpr uint32_t kStageTimeoutUs = 100000;
    uint32_t received = 0;
    uint32_t starttime;
    uint8_t headerBuffer[2];

    starttime = us_ticker_read();
    while ((received < 2) &&
           ((us_ticker_read() - starttime) < kStageTimeoutUs)) {
        if (ready()) {
            headerBuffer[0] = headerBuffer[1];
            const char byte = _getc();
            received++;
            headerBuffer[1] = byte;
            if (received >= 2 &&
                (headerBuffer[0] != ((HEADER >> 8) & 0xff) ||
                 headerBuffer[1] != (HEADER & 0xff))) {
                received = 1;
            }
        }
    }
    if (received < 2) return -1;

    starttime = us_ticker_read();
    while ((received < 4) &&
           ((us_ticker_read() - starttime) < kStageTimeoutUs)) {
        if (ready()) {
            Serialbuff[received] = _getc();
            received++;
        }
    }
    if (received < 4) return -1;

    const uint16_t data_len =
        (static_cast<uint16_t>(static_cast<uint8_t>(Serialbuff[2])) << 8) |
        static_cast<uint8_t>(Serialbuff[3]);
    const uint16_t total_len = 4 + data_len + 2;
    if (data_len > 0x200 || total_len > sizeof(Serialbuff)) return -4;

    starttime = us_ticker_read();
    while (received < total_len) {
        if ((us_ticker_read() - starttime) >= kStageTimeoutUs) return -1;
        received += dma_rx_pop(
            reinterpret_cast<uint8_t*>(Serialbuff) + received,
            total_len - received);
        if (received < total_len) dma_rx_flush();
    }

    const uint16_t tail =
        (static_cast<uint16_t>(
             static_cast<uint8_t>(Serialbuff[total_len - 2])) << 8) |
        static_cast<uint8_t>(Serialbuff[total_len - 1]);
    if (tail != FOOTER) return -2;

    const uint16_t received_crc =
        (static_cast<uint16_t>(
             static_cast<uint8_t>(Serialbuff[total_len - 4])) << 8) |
        static_cast<uint8_t>(Serialbuff[total_len - 3]);

    packet->header = HEADER;
    packet->length = data_len;
    packet->type = Serialbuff[4];
    if (data_len == 1) {
        packet->data[0] = 0;
    } else if (data_len >= 2) {
        memcpy(packet->data, Serialbuff + 5, data_len - 1);
    }
    packet->crc = received_crc;
    packet->footer = FOOTER;

    if (received_crc !=
        crc16_ccitt(reinterpret_cast<unsigned char*>(Serialbuff + 2),
                    data_len)) {
        return -3;
    }
    return 0;
}

unsigned int SerialConsole::crc16_ccitt(unsigned char *data, unsigned int len)
{
	unsigned char tmp;
	unsigned short crc = 0;

	for (unsigned int i = 0; i < len; i ++) {
        tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table[tmp]) & 0xffff;
	}

	return crc & 0xffff;
}

void SerialConsole::PacketMessage(char cmd, const char* s, int size)
{
    unsigned char frame[128];
    const uint16_t payload_size = size;
    const uint16_t frame_size = payload_size + 9;
    if (frame_size >= sizeof(frame) + 1) return;
	
    frame[0] = (HEADER >> 8) & 0xff;
    frame[1] = HEADER & 0xff;
    frame[2] = ((payload_size + 3) >> 8) & 0xff;
    frame[3] = (payload_size + 3) & 0xff;
    frame[4] = cmd;
    if (s != nullptr && payload_size != 0) {
        memcpy(frame + 5, s, payload_size);
    }
	
    const unsigned int crc = crc16_ccitt(frame + 2, payload_size + 3);
    frame[payload_size + 5] = (crc >> 8) & 0xff;
    frame[payload_size + 6] = crc & 0xff;
    frame[payload_size + 7] = (FOOTER >> 8) & 0xff;
    frame[payload_size + 8] = FOOTER & 0xff;
    puts(reinterpret_cast<char*>(frame), frame_size);
}


int SerialConsole::printfcmd(const char cmd, const char *format, ...)
{
	char b[64];
    char *buffer;
    // Make the message
    va_list args;
    va_start(args, format);

    int size = vsnprintf(b, 64, format, args) + 1; // we add one to take into account space for the terminating \0

    if (size < 64) {
        buffer = b;
    } else {
        buffer = new char[size];
        vsnprintf(buffer, size, format, args);
    }
    va_end(args);

//    puts(buffer, strlen(buffer));
	PacketMessage(PTYPE_DIAG_RES, buffer, strlen(buffer));

    if (buffer != b)
        delete[] buffer;

    return size - 1;
}

int SerialConsole::printf(const char *format, ...)
{
	char b[64];
    char *buffer;
    // Make the message
    va_list args;
    va_start(args, format);

    int size = vsnprintf(b, 64, format, args) + 1; // we add one to take into account space for the terminating \0

    if (size < 64) {
        buffer = b;
    } else {
        buffer = new char[size];
        vsnprintf(buffer, size, format, args);
    }
    va_end(args);

	PacketMessage(PTYPE_NORMAL_INFO, buffer, strlen(buffer));

    if (buffer != b)
        delete[] buffer;

    return size - 1;
}


void SerialConsole::on_idle(void * argument)
{	
    // Pump the receive path to update link supervision.
	on_serial_char_received();

    if (halt_flag) {
        halt_flag= false;
        THEKERNEL->set_halt_reason(MANUAL);
        THEKERNEL->call_event(ON_HALT, nullptr);
        if(THEKERNEL->is_grbl_mode()) {
            PacketMessage(PTYPE_NORMAL_INFO, "ALARM: Abort during cycle\r\n", 0);
        } else {
            PacketMessage(PTYPE_NORMAL_INFO, "HALTED, M999 or $X to exit HALT state\r\n", 0);
    }
}

    if (query_flag ) {
        query_flag = false;
        StreamOutput::PacketMessage(PTYPE_STATUS_RES,THEKERNEL->get_query_string().c_str(),0);
    }

    // Periodically send the firmware version in a PTYPE_VERSION_RES frame.
    if (us_ticker_read() - last_version_us > VERSION_INTERVAL_US) {
        StreamOutput::PacketMessage(PTYPE_VERSION_RES, VERSION, 0);
        last_version_us = us_ticker_read();
    }
}

void SerialConsole::on_main_loop(void * argument){
}

int SerialConsole::puts(const char* s, int size)
{
    size_t n = size == 0 ? strlen(s) : size;
    for (size_t i = 0; i < n; ++i) {
        this->_putc(s[i]);
    }
    return n;
}

int SerialConsole::_putc(int c)
{
    return this->serial->putc(c);
}

int SerialConsole::_getc()
{
    uint8_t byte;
    dma_rx_pop(&byte, 1);
    return byte;
}

bool SerialConsole::ready()
{
    dma_rx_flush();
    return dma_rx_size() != 0;
}
