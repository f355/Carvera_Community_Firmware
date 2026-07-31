/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Player.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "SerialConsole.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "Gcode.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "md5.h"

#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "ATCHandlerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "PlayerPublicAccess.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"
#include "StepTicker.h"
#include "Block.h"
#include "quicklz.h"
#include "modules/communication/SerialConsole.h"

#include <math.h>

#include <cstddef>
#include <cmath>
#include <algorithm>

#include "mbed.h"

#define home_on_boot_checksum             CHECKSUM("home_on_boot")
#define on_boot_gcode_checksum            CHECKSUM("on_boot_gcode")
#define on_boot_gcode_enable_checksum     CHECKSUM("on_boot_gcode_enable")
#define after_suspend_gcode_checksum      CHECKSUM("after_suspend_gcode")
#define before_resume_gcode_checksum      CHECKSUM("before_resume_gcode")
#define leave_heaters_on_suspend_checksum CHECKSUM("leave_heaters_on_suspend")
#define laser_module_clustering_checksum 	  CHECKSUM("laser_module_clustering")


// Wait between resolving the path and announcing it.
#define PLAY_START_DELAY_US    300000
// How long goto waits for a progress report before giving up.
#define GOTO_REPLY_TIMEOUT_US    1000000
// Most lines asked for in one request.
#define JOB_LINES_PER_REQUEST    0xdf
// Above this depth the engine stops asking for more.
#define JOB_LINES_HIGH_WATER    0xb3
// How long an unanswered request waits before being repeated.
#define JOB_REQUEST_RETRY_US    5000000
// How long after the last line before playing_file is cleared.
#define JOB_STOP_DELAY_US    1000000

namespace {

constexpr uint32_t kJobLineCapacity = 0xe0;
struct job_line job_line_storage[kJobLineCapacity]
    __attribute__((section("AHBSRAM1"), aligned(4)));

struct JobLineQueueInitializer {
    JobLineQueueInitializer()
    {
        job_lines.slots = job_line_storage;
        job_lines.capacity = kJobLineCapacity;
        job_lines.tail = 0;
        job_lines.head = 0;
        job_lines.count = 0;
        memset(job_line_storage, 0, sizeof(job_line_storage));
    }
};

} // namespace

struct job_line_queue job_lines;
static JobLineQueueInitializer job_line_queue_initializer;
uint32_t job_expected_line;
uint8_t job_line_error;
uint8_t job_eof;
uint8_t job_complete_pending;
uint32_t job_last_request = 100000;
uint32_t job_last_request_us;
extern unsigned short crc_table[256];
// used for XMODEM
#define WAIT_MD5  0x01
#define WAIT_FILE_VIEW  0x02
#define READ_FILE_DATA  0x03

#define MAXRETRANS 50
#define RETRYTIME  50
#define TIMEOUT_MS 10
#define RETRYTIMES 10

Player::Player()
{
    this->playing_file = false;
    this->current_file_handler = nullptr;
    this->booted = false;
    this->elapsed_secs = 0;
    this->reply_stream = nullptr;
    this->inner_playing = false;
    this->slope = 0.0;
    this->job_ending = 0;
    this->stop_at_us = 0;
    this->final_lines = 0;
    this->final_percent = 0;
    this->final_playing = 0;
}

void Player::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_HALT);

    this->on_boot_gcode = THEKERNEL->config->value(on_boot_gcode_checksum)->by_default("/sd/on_boot.gcode")->as_string();
    this->on_boot_gcode_enable = THEKERNEL->config->value(on_boot_gcode_enable_checksum)->by_default(false)->as_bool();

    this->home_on_boot = THEKERNEL->config->value(home_on_boot_checksum)->by_default(true)->as_bool();

    this->after_suspend_gcode = THEKERNEL->config->value(after_suspend_gcode_checksum)->by_default("")->as_string();
    this->before_resume_gcode = THEKERNEL->config->value(before_resume_gcode_checksum)->by_default("")->as_string();
    std::replace( this->after_suspend_gcode.begin(), this->after_suspend_gcode.end(), '_', ' '); // replace _ with space
    std::replace( this->before_resume_gcode.begin(), this->before_resume_gcode.end(), '_', ' '); // replace _ with space
    this->leave_heaters_on = THEKERNEL->config->value(leave_heaters_on_suspend_checksum)->by_default(false)->as_bool();

    this->laser_clustering = THEKERNEL->config->value(laser_module_clustering_checksum)->by_default(false)->as_bool();
}

void Player::on_halt(void* argument)
{
    this->clear_buffered_queue();

    if(argument == nullptr && this->playing_file ) {
        abort_command("1", &(StreamOutput::NullStream));
	}

	if(argument == nullptr && (THEKERNEL->is_suspending() || THEKERNEL->is_waiting())) {
		// clean up from suspend
		THEKERNEL->set_waiting(false);
		THEKERNEL->set_suspending(false);
		THEROBOT->pop_state();
		THEKERNEL->streams->printf("Suspend cleared\n");
	}
}

void Player::on_second_tick(void *)
{
    if (THEKERNEL->is_suspending() || THEKERNEL->is_tool_waiting()) return;
    if (this->playing_file || THEKERNEL->abort_finishing) this->elapsed_secs++;
}

// extract any options found on line, terminates args at the space before the first option (-v)
// eg this is a file.gcode -v
//    will return -v and set args to this is a file.gcode
string Player::extract_options(string& args)
{
    string opts;
    size_t pos= args.find(" -");
    if(pos != string::npos) {
        opts= args.substr(pos);
        args= args.substr(0, pos);
    }

    return opts;
}

void Player::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    string args = get_arguments(gcode->get_command());
    if (gcode->has_m) {
        if (gcode->m == 21) { // Dummy code; makes Octoprint happy -- supposed to initialize SD card
            // Compatibility no-op.

        } else if (gcode->m == 23) { // select file
            this->filename = "/sd/" + args; // filename is whatever is in args
            this->current_stream = nullptr;

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }
            this->current_file_handler = fopen( this->filename.c_str(), "r");

            if(this->current_file_handler == NULL) {
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
                return;

            } else {
                // get size of file
                int result = fseek(this->current_file_handler, 0, SEEK_END);
                if (0 != result) {
                    this->file_size = 0;
                } else {
                    this->file_size = ftell(this->current_file_handler);
                    fseek(this->current_file_handler, 0, SEEK_SET);
                }
                gcode->stream->printf("File opened:%s Size:%ld\r\n", this->filename.c_str(), this->file_size);
                gcode->stream->printf("File selected\r\n");
            }


            this->played_cnt = 0;
            this->played_lines = 0;
            this->queued_lines = 0;
            this->elapsed_secs = 0;
            this->playing_lines = 0;
            this->goto_line = 0;
            this->pending_link_cmd = 0;
            this->host_name_crc = 0;
            job_expected_line = 0;
            job_line_error = 0;
            job_lines.count = 0;
            job_lines.head = 0;
            job_lines.tail = 0;
            for (uint32_t i = 0; i < job_lines.capacity; i++) {
                job_lines.slots[i].valid = 0;
                job_lines.slots[i].text[0] = 0;
            }

        } else if (gcode->m == 24) { // start print
            if (this->current_file_handler != NULL) {
                this->playing_file = true;
                // this would be a problem if the stream goes away before the file has finished,
                // so we attach it to the kernel stream, however network connections from pronterface
                // do not connect to the kernel streams so won't see this FIXME
                this->reply_stream = THEKERNEL->streams;
            }

        } else if (gcode->m == 25) { // pause print
            this->playing_file = false;

        } else if (gcode->m == 26) { // Reset print. Slightly different than M26 in Marlin and the rest
            if(this->current_file_handler != NULL) {
                string currentfn = this->filename.c_str();
                unsigned long old_size = this->file_size;

                // abort the print
                abort_command("", gcode->stream);

                if(!currentfn.empty()) {
                    // reload the last file opened
                    this->current_file_handler = fopen(currentfn.c_str() , "r");

                    if(this->current_file_handler == NULL) {
                        gcode->stream->printf("file.open failed: %s\r\n", currentfn.c_str());
                    } else {
                        this->filename = currentfn;
                        this->file_size = old_size;
                        this->current_stream = nullptr;
                    }
                }
            } else {
                gcode->stream->printf("No file loaded\r\n");
            }

        } else if (gcode->m == 27) { // report print progress, in format used by Marlin
            progress_command("-b", gcode->stream);

        } else if (gcode->m == 32) { // select file and start print
            // Get filename
            this->filename = "/sd/" + args; // filename is whatever is in args including spaces
            this->current_stream = nullptr;

            if(this->current_file_handler != NULL) {
                this->playing_file = false;
                fclose(this->current_file_handler);
            }

            this->current_file_handler = fopen( this->filename.c_str(), "r");
            if(this->current_file_handler == NULL) {
                gcode->stream->printf("file.open failed: %s\r\n", this->filename.c_str());
            } else {
                this->playing_file = true;

                // get size of file
                int result = fseek(this->current_file_handler, 0, SEEK_END);
                if (0 != result) {
                        file_size = 0;
                } else {
                        file_size = ftell(this->current_file_handler);
                        fseek(this->current_file_handler, 0, SEEK_SET);
                }
            }

            this->played_cnt = 0;
            this->played_lines = 0;
            this->queued_lines = 0;
            this->elapsed_secs = 0;
            this->playing_lines = 0;
            this->goto_line = 0;
            this->pending_link_cmd = 0;
            this->host_name_crc = 0;
            job_expected_line = 0;
            job_line_error = 0;
            job_lines.count = 0;
            job_lines.head = 0;
            job_lines.tail = 0;
            for (uint32_t i = 0; i < job_lines.capacity; i++) {
                job_lines.slots[i].valid = 0;
                job_lines.slots[i].text[0] = 0;
            }

        } else if (gcode->m == 600) { // suspend print, Not entirely Marlin compliant, M600.1 will leave the heaters on
            this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream);

        } else if (gcode->m == 601) { // resume print
            this->resume_command("", gcode->stream);
        }

    }else if(gcode->has_g) {
        if(gcode->g == 28) { // homing cancels suspend
            if (THEKERNEL->is_suspending()) {
                // clean up
            	THEKERNEL->set_suspending(false);
                THEROBOT->pop_state();
            }
        }
    }
}

// When a new line is received, check if it is a command, and if it is, act upon it
void Player::on_console_line_received( void *argument )
{
    if(THEKERNEL->is_halted()) return; // if in halted state ignore any commands

    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    string possible_command = new_message.message;

    // ignore anything that is not lowercase or a letter
    if(possible_command.empty() || !islower(possible_command[0]) || !isalpha(possible_command[0])) {
        return;
    }

    string cmd = shift_parameter(possible_command);

	// new_message.stream->printf("Play Received %s\r\n", possible_command.c_str());

    // Act depending on command
    if (cmd == "play"){
        this->play_command( possible_command, new_message.stream );
    }else if (cmd == "progress"){
        this->progress_command( possible_command, new_message.stream );
    }else if (cmd == "abort") {
        this->abort_command( possible_command, new_message.stream );
    }else if (cmd == "suspend") {
        this->suspend_command( possible_command, new_message.stream );
    }else if (cmd == "resume") {
        this->resume_command( possible_command, new_message.stream );
    }else if (cmd == "goto") {
    	this->goto_command( possible_command, new_message.stream );
    }else if (cmd == "buffer") {
    	this->buffer_command( possible_command, new_message.stream );
    }else if (cmd == "upload") {
    	this->upload_command( possible_command, new_message.stream );
    }else if (cmd == "download") {
        memset(md5_str, 0, sizeof(md5_str));
    	if (possible_command.find("config.txt") != string::npos) {
        	this->test_command( possible_command, new_message.stream );
    	}
    	this->download_command( possible_command, new_message.stream );
    }
}

// Buffer gcode to queue
void Player::buffer_command( string parameters, StreamOutput *stream )
{
	this->buffered_queue.push(parameters);
	stream->printf("Command buffered: %s\r\n", parameters.c_str());
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput *stream )
{
    // Wait for the machine to reach a known position.
    if (!THEROBOT->is_homed_all_axes()) return;

    if (THEKERNEL->abort_finishing) {
        stream->printf("Abort finishing, please wait\r\n");
		return;
	}

    string options= extract_options(parameters);
    this->filename = absolute_from_relative(shift_parameter(parameters));
    this->last_filename = this->filename;

    if (this->playing_file || THEKERNEL->is_tool_waiting() || THEKERNEL->is_aborted()) {
        stream->printf("Currently printing, abort print first\r\n");
        return;
    }

    this->reset_position_if_pending();

    // Send a PTYPE_PLAY_START request identified by the path CRC-16.
    uint16_t crc = 0;
    for (unsigned int i = 0; i < this->filename.size(); i++) {
        crc = (crc_table[(((unsigned char)this->filename[i]) ^ (crc >> 8)) & 0xff] ^ (crc << 8)) & 0xffff;
    }
    this->name_crc = crc;

    safe_delay_us(PLAY_START_DELAY_US);

    char id[2];
    id[0] = (crc >> 8) & 0xFF;
    id[1] = crc & 0xFF;
    THEKERNEL->serial->PacketMessage(PTYPE_PLAY_START, id, 2);

    // Output to the current stream if we were passed the -v ( verbose ) option
    if( options.find_first_of("Vv") == string::npos ) {
        this->current_stream = nullptr;
    } else {
        this->current_stream = THEKERNEL->streams;
    }
}

// Skipped while an abort is unwinding.
void Player::reset_position_if_pending()
{
    if (THEKERNEL->abort_finishing) return;
    if (THEKERNEL->is_position_reset_pending()) {
        THEROBOT->reset_position_from_current_actuator_position();
    }
    THEKERNEL->set_position_reset_pending(false);
}

// Goto a certain line when playing a file
void Player::goto_command( string parameters, StreamOutput *stream )
{
    if (!THEKERNEL->is_tool_waiting()) {
        stream->printf("Can only jump when pausing!\r\n");
        return;
    }

    string line_str = shift_parameter(parameters);
    if (line_str.empty()) return;

        char *ptr = NULL;
        this->goto_line = strtol(line_str.c_str(), &ptr, 10);
        this->goto_line = this->goto_line < 1 ? 1 : this->goto_line;
        stream->printf("Goto line %lu...\r\n", this->goto_line);

    // Send PTYPE_PLAY_GOTO: path identifier followed by a big-endian line.
    char req[6];
    req[0] = (this->name_crc >> 8) & 0xFF;
    req[1] = this->name_crc & 0xFF;
    req[2] = (this->goto_line >> 24) & 0xFF;
    req[3] = (this->goto_line >> 16) & 0xFF;
    req[4] = (this->goto_line >> 8) & 0xFF;
    req[5] = this->goto_line & 0xFF;
    THEKERNEL->serial->PacketMessage(PTYPE_PLAY_GOTO, req, 6);

    // Start over from wherever it lands us.
    this->played_lines = 0;
    this->queued_lines = 0;
    this->played_cnt = 0;
    this->host_name_crc = 0;
    job_line_error = 0;
    job_eof = 0;
    job_expected_line = 0;
    job_lines.count = 0;
    job_lines.head = 0;
    job_lines.tail = 0;
    for (uint32_t i = 0; i < job_lines.capacity; i++) {
        job_lines.slots[i].valid = 0;
        job_lines.slots[i].text[0] = 0;
            }

    // Wait for seek progress until the target is reached or the timeout expires.
    uint32_t deadline = us_ticker_read();
    for (;;) {
        if (this->goto_line <= this->played_lines) {
            job_expected_line = this->played_lines;
            stream->printf("Info:Goto line successed,current line is :%lu.\r\n", this->played_lines);
            return;
        	}

        if (job_info_block.valid) {
            if (job_info_block.data[2] == (char)PTYPE_PLAY_GOTO_DATA &&
                this->name_crc == (uint16_t)(((uint8_t)job_info_block.data[3] << 8) | (uint8_t)job_info_block.data[4])) {
                this->played_lines = ((uint32_t)(uint8_t)job_info_block.data[5]<<24) | ((uint32_t)(uint8_t)job_info_block.data[6]<<16)
                                   | ((uint32_t)(uint8_t)job_info_block.data[7]<<8)  |  (uint32_t)(uint8_t)job_info_block.data[8];
                this->played_cnt   = ((uint32_t)(uint8_t)job_info_block.data[9]<<24) | ((uint32_t)(uint8_t)job_info_block.data[10]<<16)
                                   | ((uint32_t)(uint8_t)job_info_block.data[11]<<8) |  (uint32_t)(uint8_t)job_info_block.data[12];
                deadline = us_ticker_read();
        }
            job_info_block.valid = 0;
    }

        THEKERNEL->call_event(ON_IDLE, 0);
        if (us_ticker_read() - deadline > GOTO_REPLY_TIMEOUT_US) break;
    }

    job_expected_line = this->played_lines;
    stream->printf("Error:Goto line failed,current line is :%lu.\r\n", this->played_lines);
}

void Player::progress_command( string parameters, StreamOutput *stream )
{

    // get options
    string options = shift_parameter( parameters );
    bool sdprinting= options.find_first_of("Bb") != string::npos;

    if(!playing_file && current_file_handler != NULL) {
        if(sdprinting)
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        else
            stream->printf("SD print is paused at %lu/%lu\r\n", played_cnt, file_size);
        return;

    } else if(!playing_file) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    if(file_size > 0) {
        unsigned long est = 0;
        if(this->elapsed_secs > 10) {
            unsigned long bytespersec = played_cnt / this->elapsed_secs;
            if(bytespersec > 0)
                est = (file_size - played_cnt) / bytespersec;
        }

        float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
        // If -b or -B is passed, report in the format used by Marlin and the others.
        if (!sdprinting) {
            stream->printf("file: %s, %u %% complete, elapsed time: %02lu:%02lu:%02lu", this->filename.c_str(), (unsigned int)roundf(pcnt), this->elapsed_secs / 3600, (this->elapsed_secs % 3600) / 60, this->elapsed_secs % 60);
            if(est > 0) {
                stream->printf(", est time: %02lu:%02lu:%02lu",  est / 3600, (est % 3600) / 60, est % 60);
            }
            stream->printf("\r\n");
        } else {
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        }

    } else {
        stream->printf("File size is unknown\r\n");
    }
}

void Player::abort_command( string parameters, StreamOutput *stream )
{
    if(!playing_file && current_file_handler == NULL) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    unsigned int abort_at =
        std::max(THECONVEYOR->queued_line(), THEKERNEL->get_current_line());
    if (abort_at == 0) abort_at = this->played_lines + 1;
    if (abort_at == 0) abort_at = 1;
	
    if (!parameters.empty()) {
        this->playing_file = false;
        this->clear_buffered_queue();
        job_lines.count = 0;
        job_lines.head = 0;
        job_lines.tail = 0;
        for (uint32_t i = 0; i < job_lines.capacity; i++) {
            job_lines.slots[i].valid = 0;
            job_lines.slots[i].text[0] = 0;
        }
        THEKERNEL->set_tool_waiting(false);
        THEKERNEL->set_aborted(true);
        this->complete_abort();
        stream->printf("Aborted by halt\n");
        return;
    }

    THEKERNEL->arm_abort_filter(abort_at);
    THEKERNEL->abort_finishing = 1;
	    
	    this->playing_file = false;
    this->queued_lines = 0;
	    this->goto_line = 0;
    this->job_ending = 0;
    this->stop_at_us = 0;
	    this->clear_buffered_queue();
    this->current_stream = NULL;
    this->pending_link_cmd = 0;
    this->host_name_crc = 0;

    job_lines.count = 0;
    job_lines.head = 0;
    job_lines.tail = 0;
    for (uint32_t i = 0; i < job_lines.capacity; i++) {
        job_lines.slots[i].valid = 0;
        job_lines.slots[i].text[0] = 0;
    }
    job_line_error = 0;
    job_expected_line = 0;

    THEKERNEL->serial->PacketMessage(PTYPE_PLAY_CANCEL, NULL, 0);
    fclose(this->current_file_handler);
    this->current_file_handler = NULL;
	
    THEKERNEL->set_tool_waiting(false);
    THEKERNEL->set_aborted(true);
    THECONVEYOR->discard_queued_blocks_after(abort_at);
}

void Player::clear_buffered_queue(){
	while (!this->buffered_queue.empty()) {
		this->buffered_queue.pop();
	}
}

// Request lines using a path identifier, zero-based line index, and batch size.
void Player::request_job_lines()
{
    char req[8];
    req[0] = (this->name_crc >> 8) & 0xFF;
    req[1] = this->name_crc & 0xFF;
    req[2] = (job_expected_line >> 24) & 0xFF;
    req[3] = (job_expected_line >> 16) & 0xFF;
    req[4] = (job_expected_line >> 8) & 0xFF;
    req[5] = job_expected_line & 0xFF;
    // Request the available queue capacity.
    uint16_t want = JOB_LINES_PER_REQUEST - job_lines.count;
    req[6] = (want >> 8) & 0xFF;
    req[7] = want & 0xFF;
    THEKERNEL->serial->PacketMessage(PTYPE_PLAY_DATA, req, 8);
    job_last_request = job_expected_line;
    job_last_request_us = us_ticker_read();
    job_line_error = 0;
}

// Request lines while playback needs them and the queue has capacity.
void Player::maybe_request_job_lines()
{
    if (job_eof) return;
    if (job_lines.count > JOB_LINES_HIGH_WATER) return;
    if (job_expected_line != job_last_request) { this->request_job_lines(); return; }
    if (job_line_error)                        { this->request_job_lines(); return; }
    if (us_ticker_read() - job_last_request_us > JOB_REQUEST_RETRY_US) this->request_job_lines();
}

// Take the next buffered line, requesting more when needed.
bool Player::next_job_line(char *buf, unsigned int size)
{
    if (job_lines.count == 0) return false;

    struct job_line *slot = &job_lines.slots[job_lines.tail];
    strncpy(buf, slot->text, sizeof(slot->text) - 1);
    buf[sizeof(slot->text) - 1] = 0;
    slot->valid = 0;
    job_lines.count--;
    job_lines.tail = (job_lines.tail + 1) % job_lines.capacity;
    return true;
}

// Stop job-line streaming and discard buffered lines.
void Player::cancel_job_stream()
{
    THEKERNEL->serial->PacketMessage(PTYPE_PLAY_CANCEL, NULL, 0);
    job_line_error = 0;
    job_eof = 0;
    job_expected_line = 0;
    job_last_request = 0;
    job_lines.count = 0;
    job_lines.head = 0;
    job_lines.tail = 0;
}

void Player::stop_job_outputs()
{
    struct SerialMessage message;
    message.message = "M5";
    message.stream = THEKERNEL->streams;
    message.line = 0;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);

    if (THEKERNEL->get_laser_mode()) {
        message.message = "laserabort";
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    }
}

void Player::complete_abort()
{
    THECONVEYOR->flush_queue();
    this->stop_job_outputs();
    THEROBOT->reset_position_from_current_actuator_position();
    THEKERNEL->clear_abort_filter();
    THEKERNEL->set_aborted(false);
    THEKERNEL->abort_finishing = 0;
    this->reset_job_state();
}

// Processes a pending PTYPE_PLAY_VIEW response.
void Player::handle_pending_link_cmd()
{
    // Clear the pending link message before handling it.
    uint8_t cmd = this->pending_link_cmd;
    this->pending_link_cmd = 0;
    if (cmd != PTYPE_PLAY_VIEW) return;

    if (THEKERNEL->abort_finishing) return;

    if (this->host_name_crc != this->name_crc) {
        THEKERNEL->streams->printf("error: File name CRC check failed,please retry.\r\n");
        THEKERNEL->serial->PacketMessage(PTYPE_PLAY_CANCEL, NULL, 0);
        return;
    }

    THEKERNEL->streams->printf("  File size %ld\r\n", this->file_size);

    this->played_cnt = 0;
    this->played_lines = 0;
    this->queued_lines = 0;
    this->elapsed_secs = 0;
    this->playing_lines = 0;
    this->goto_line = 0;
    this->host_name_crc = 0;
    job_line_error = 0;
    job_eof = 0;
    job_complete_pending = 0;
    job_expected_line = 0;
    this->job_ending = 0;
    this->stop_at_us = 0;
    job_lines.count = 0;
    job_lines.head = 0;
    job_lines.tail = 0;
    for (uint32_t i = 0; i < job_lines.capacity; i++) {
        job_lines.slots[i].valid = 0;
        job_lines.slots[i].text[0] = 0;
    }

    THEROBOT->absolute_mode = true;
    THEROBOT->e_absolute_mode = true;
    THEROBOT->reset_position_from_current_actuator_position();

    char req[8];
    req[0] = (this->name_crc >> 8) & 0xFF;
    req[1] = this->name_crc & 0xFF;
    req[2] = 0;
    req[3] = 0;
    req[4] = 0;
    req[5] = 0;
    req[6] = (JOB_LINES_PER_REQUEST >> 8) & 0xFF;
    req[7] = JOB_LINES_PER_REQUEST & 0xFF;
    THEKERNEL->serial->PacketMessage(PTYPE_PLAY_DATA, req, sizeof(req));
    this->playing_file = true;
    this->reset_position_if_pending();
}

// Runs first on every main loop. While an abort is unwinding, wait for motion
// to stop, then flush and clear the abort state.
void Player::abort_finish_tick()
{
    if (!THEKERNEL->abort_finishing) return;
    if (THEKERNEL->is_halted()) {
        this->complete_abort();
        return;
    }
    if (THEKERNEL->line_depth != 0 || !THECONVEYOR->is_idle()) return;

    THECONVEYOR->flush_queue();
    this->stop_job_outputs();
    THEROBOT->reset_position_from_current_actuator_position();
    THEKERNEL->abort_finishing = 0;
    THEKERNEL->abort_line = 0;
    THEKERNEL->clear_abort_filter();
    THEKERNEL->set_aborted(false);
    this->reset_job_state();
    THEKERNEL->streams->printf("Aborted playing or paused file. \r\n");
}

// Sends the completion status three times.
void Player::broadcast_status()
{
    for (int i = 0; i < 3; i++) {
        THEKERNEL->serial->PacketMessage(PTYPE_STATUS_RES, THEKERNEL->get_query_string().c_str(), 0);
    }
}

// Clear the completed job's state.
void Player::reset_job_state()
{
    this->filename.assign("");
    this->file_size = 0;
    this->played_cnt = 0;
    this->played_lines = 0;
    this->queued_lines = 0;
    this->goto_line = 0;
    this->playing_lines = 0;
    this->final_lines = 0;
    this->final_percent = 0;
    this->final_playing = 0;
}

void Player::finish_job()
{
    // Complete playback after the source is exhausted and the queue drains.
    job_complete_pending = 1;
    THECONVEYOR->wait_for_idle();

    this->final_playing  = this->elapsed_secs;
    this->final_lines    = job_expected_line;
    this->final_percent  = 100;
    this->job_ending     = 1;
    this->broadcast_status();

    this->stop_at_us = us_ticker_read() + JOB_STOP_DELAY_US;

    if (this->reply_stream != NULL) {
        this->reply_stream->printf("Done printing file\r\n");
        this->reply_stream = NULL;
    }

    bool done = true;
    PublicData::set_value( atc_handler_checksum, set_job_complete_checksum, &done );

    job_eof = 0;
    job_complete_pending = 0;
}

void Player::on_main_loop(void *argument)
{
    this->abort_finish_tick();

    if( !this->booted ) {
        this->booted = true;
        if (this->home_on_boot) {
    		struct SerialMessage message;
    		message.message = "$H";
    		message.stream = THEKERNEL->streams;
    		message.line = 0;
    		THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
        }

        if (this->on_boot_gcode_enable) {
            this->play_command(this->on_boot_gcode, THEKERNEL->serial);
        }

    }

    if ( !this->playing_file ) {
        this->handle_pending_link_cmd();
        if (THEKERNEL->abort_finishing) return;
        this->reset_position_if_pending();
    }

    if ( this->playing_file ) {
        // Complete a stop armed after the final line.
        if (this->stop_at_us != 0) {
            if (us_ticker_read() < this->stop_at_us) return;
            this->playing_file = false;
            this->stop_at_us = 0;
            this->job_ending = 0;
            this->reset_job_state();
            this->current_file_handler = NULL;
            this->current_stream = NULL;
            job_expected_line = 0;
            job_line_error = 0;
            return;
        }

        // Suppress line processing while playback winds down.
        if (job_complete_pending) return;

        if(THEKERNEL->is_halted() || THEKERNEL->is_tool_waiting() || THEKERNEL->is_aborted() || this->inner_playing) {
            return;
        }

        // check if there are bufferd command
        while (!this->buffered_queue.empty()) {
        	THEKERNEL->streams->printf("%s\r\n", this->buffered_queue.front().c_str());
			struct SerialMessage message;
			message.message = this->buffered_queue.front();
			message.stream = THEKERNEL->streams;
			message.line = 0;
			this->buffered_queue.pop();

			// waits for the queue to have enough room
			THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            job_last_request_us = us_ticker_read();
            return;
        }

        char buf[sizeof(job_line::text)];
        if (this->next_job_line(buf, sizeof(buf))) {
            const size_t len = strlen(buf);
            bool blank = true;
            for (size_t i = 0; i < len; ++i) {
                if (buf[i] != '\t' && buf[i] != '\n' &&
                    buf[i] != '\r' && buf[i] != ' ') {
                    blank = false;
                    break;
	                            }
	                        }

            if (!blank) {
							struct SerialMessage message;
                message.message = buf;
                message.stream = this->current_stream == nullptr ?
                    &(StreamOutput::NullStream) : this->current_stream;
                message.line = this->played_lines + 1;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
                this->played_cnt += len;
            }
            this->played_lines++;
        }

        this->queued_lines = job_expected_line;
        this->maybe_request_job_lines();

        if (this->pending_link_cmd == PTYPE_PLAY_ERROR) {
            job_eof = 1;
        } else if (this->pending_link_cmd == PTYPE_PLAY_CANCEL) {
            this->playing_file = false;
            this->played_cnt = 0;
            this->played_lines = 0;
            this->queued_lines = 0;
            this->elapsed_secs = 0;
            this->goto_line = 0;
            this->job_ending = 0;
            this->stop_at_us = 0;
            job_expected_line = 0;
            job_eof = 0;
            job_line_error = 0;
            job_complete_pending = 0;
            job_lines.count = 0;
            job_lines.head = 0;
            job_lines.tail = 0;
            for (uint32_t i = 0; i < job_lines.capacity; i++) {
                job_lines.slots[i].valid = 0;
                job_lines.slots[i].text[0] = 0;
            }
            this->pending_link_cmd = 0;
							return;
						}
        this->pending_link_cmd = 0;

        if (job_lines.count == 0 && job_eof) {
            job_eof = 0;
            this->finish_job();
	                            }
    }
}

/*
bool Player::check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value)
{
	float new_slope = 0.0;
	bool is_cluster = false;
	Gcode *gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
	if (!gcode->has_m && gcode->has_g && gcode->g == 1) {
		*x_value = gcode->get_value('X');
		*y_value = gcode->get_value('Y');
		*s_value = gcode->get_value('S');
		*distance = sqrtf((*x_value) * (*x_value) + (*y_value) * (*y_value));
		if (*x_value == 0) {
			new_slope = *y_value > 0 ? 1000 : -1000;
		} else if (*y_value == 0) {
			new_slope = *x_value > 0 ? 0.001 : -0.001;
		} else {
			new_slope = *y_value / *x_value;
		}
		if ((*distance) < 1.0 && fabs (new_slope - *slope) < 0.1) {
			is_cluster = true;
		}
		*slope = new_slope;
	}
	delete gcode;

	return is_cluster;
}
*/

void Player::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(is_playing_checksum) || pdr->second_element_is(is_suspended_checksum)) {
        static bool bool_data;
        bool_data = pdr->second_element_is(is_playing_checksum) ? this->playing_file : THEKERNEL->is_suspending();
        pdr->set_data_ptr(&bool_data);
        pdr->set_taken();

    } else if(pdr->second_element_is(get_progress_checksum)) {
        static struct pad_progress p;
        if(file_size > 0 && (playing_file || THEKERNEL->abort_finishing)) {
            if (this->job_ending) {
                p.percent_complete = this->final_percent;
                p.played_lines = this->final_lines;
                p.elapsed_secs = this->final_playing;
            } else if (!this->inner_playing) {
                const Block *block = StepTicker::getInstance()->get_current_block();
                // Note to avoid a race condition where the block is being cleared we check the is_ready flag which gets cleared first,
                // as this is an interrupt if that flag is not clear then it cannot be cleared while this is running and the block will still be valid (albeit it may have finished)
                if (block != nullptr && block->is_ready && block->is_g123) {
                	this->playing_lines = block->line;
                	p.played_lines = this->playing_lines;
                } else {
                	p.played_lines = this->played_lines;
                }
        	} else {
        		p.played_lines = this->played_lines;
        	}
            if (!this->job_ending) {
            p.elapsed_secs = this->elapsed_secs;
            float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
            p.percent_complete = roundf(pcnt);
            }
            p.filename = this->filename;
            pdr->set_data_ptr(&p);
            pdr->set_taken();
        }
    } else if (pdr->second_element_is(inner_playing_checksum)) {
    	bool b = this->inner_playing;
        pdr->set_data_ptr(&b);
        pdr->set_taken();
    }
}

void Player::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(abort_play_checksum)) {
        abort_command("", &(StreamOutput::NullStream));
        pdr->set_taken();
    } else if (pdr->second_element_is(inner_playing_checksum)) {
    	bool b = *static_cast<bool *>(pdr->get_data_ptr());
    	this->inner_playing = b;
    	if (this->playing_file) pdr->set_taken();
    } else if (pdr->second_element_is(restart_job_checksum)) {
    	if (!this->last_filename.empty()) {
    		THEKERNEL->streams->printf("Job restarted: %s.\r\n", this->last_filename.c_str());
        	this->play_command(this->last_filename, &(StreamOutput::NullStream));
    	}
    } else if (pdr->second_element_is(link_cmd_checksum)) {
        this->pending_link_cmd = *static_cast<uint8_t *>(pdr->get_data_ptr());
    } else if (pdr->second_element_is(link_name_crc_checksum)) {
        this->host_name_crc = *static_cast<uint16_t *>(pdr->get_data_ptr());
    } else if (pdr->second_element_is(link_file_size_checksum)) {
        this->file_size = *static_cast<uint32_t *>(pdr->get_data_ptr());
    } else if (pdr->second_element_is(link_line_queued_checksum)) {
        this->queued_lines++;
    }
}

/**
Suspend a print in progress
1. send pause to upstream host, or pause if printing from sd
1a. loop on_main_loop several times to clear any buffered commmands
2. wait for empty queue
3. save the current position, extruder position, temperatures - any state that would need to be restored
4. retract by specifed amount either on command line or in config
5. turn off heaters.
6. optionally run after_suspend gcode (either in config or on command line)

User may jog or remove and insert filament at this point, extruding or retracting as needed

*/
void Player::suspend_command(string parameters, StreamOutput *stream )
{
    if (THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        stream->printf("Already suspended!\n");
        return;
    }

    if(!this->playing_file) {
        stream->printf("Can not suspend when not playing file!\n");
        return;
    }

    stream->printf("Suspending , waiting for queue to empty...\n");

    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THEKERNEL->conveyor->wait_for_idle();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Suspend aborted by halt\n");
        THEKERNEL->set_waiting(false);
        return;
    }

    THEKERNEL->set_waiting(false);
    THEKERNEL->set_suspending(true);

    THEKERNEL->streams->printf("now save current pos...\n");

    // save current XYZ position in WCS
    Robot::wcs_t mpos= THEROBOT->get_axis_position();
    Robot::wcs_t wpos= THEROBOT->mcs2wcs(mpos);
    saved_position[0]= std::get<X_AXIS>(wpos);
    saved_position[1]= std::get<Y_AXIS>(wpos);
    saved_position[2]= std::get<Z_AXIS>(wpos);

    // save current state
    THEROBOT->push_state();
    current_motion_mode = THEROBOT->get_current_motion_mode();

    // execute optional gcode if defined
    if(!after_suspend_gcode.empty()) {
        struct SerialMessage message;
        message.message = after_suspend_gcode;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    THEKERNEL->streams->printf("Suspended, resume to continue playing\n");
}

/**
resume the suspended print
1. restore the temperatures and wait for them to get up to temp
2. optionally run before_resume gcode if specified
3. restore the position it was at and E and any other saved state
4. resume sd print or send resume upstream
*/
void Player::resume_command(string parameters, StreamOutput *stream )
{
    if(!THEKERNEL->is_suspending()) {
        stream->printf("Not suspended\n");
        return;
    }

    stream->printf("Resuming playing...\n");

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Resume aborted by kill\n");
        THEROBOT->pop_state();
        THEKERNEL->set_suspending(false);
        return;
    }

    // execute optional gcode if defined
    if(!before_resume_gcode.empty()) {
        stream->printf("Executing before resume gcode...\n");
        struct SerialMessage message;
        message.message = before_resume_gcode;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    if (this->goto_line == 0) {
        // Restore position
        stream->printf("Restoring saved XYZ positions and state...\n");

        THEROBOT->absolute_mode = true;

        char buf[128];
        snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f Z%.3f F%.3f", saved_position[0], saved_position[1], saved_position[2], THEROBOT->from_millimeters(1000));
        struct SerialMessage message;
        message.message = buf;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );

    	if (current_motion_mode > 1) {
            snprintf(buf, sizeof(buf), "G%d", current_motion_mode - 1);
            message.message = buf;
            message.line = 0;
            THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    	}
    }

    THEROBOT->pop_state();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Resume aborted by kill\n");
        THEKERNEL->set_suspending(false);
        return;
    }

	THEKERNEL->set_suspending(false);

	stream->printf("Playing file resumed\n");
}


void Player::upload_command( string parameters, StreamOutput *stream )
{
    // Compatibility no-op.
}


void Player::test_command( string parameters, StreamOutput* stream ) {
    string filename = absolute_from_relative(shift_parameter(parameters));   
	FILE *fd = fopen(filename.c_str(), "rb");
	if (NULL != fd) {
        MD5 md5;
        char md5buf[64];
        do {
            size_t n = fread(md5buf, 1, sizeof(md5buf), fd);
            if (n > 0) md5.update(md5buf, n);
            THEKERNEL->call_event(ON_IDLE);
        } while (!feof(fd));
        strcpy(md5_str, md5.finalize().hexdigest().c_str());
        fclose(fd);
        fd = NULL;
	}
}

void Player::download_command( string parameters, StreamOutput *stream )
{
    // Compatibility no-op.
    }
