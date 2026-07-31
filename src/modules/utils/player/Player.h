/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "Module.h"

#include <stdint.h>
#include <stdio.h>
#include <string>
#include <map>
#include <vector>
#include <queue>
using std::string;

class StreamOutput;

// One received job line: 64 bytes of text and a validity flag.
struct job_line {
    char    text[0x40];
    uint8_t valid;
};

struct job_line_queue {
    struct job_line *slots;
    uint32_t capacity;
    uint32_t tail;
    uint32_t head;
    uint32_t count;
};

extern struct job_line_queue job_lines;
// Expected sequence number for the next received job line.
extern uint32_t job_expected_line;
// Sequence-error flag for received job lines.
extern uint8_t job_line_error;
// End-of-file flag latched by the streaming peer.
extern uint8_t job_eof;
// Playback completion is pending.
extern uint8_t job_complete_pending;
// Most recently requested line index and time.
extern uint32_t job_last_request;
extern uint32_t job_last_request_us;

class Player : public Module {
    public:
        Player();

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void on_main_loop( void* argument );
        void on_second_tick(void* argument);
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_gcode_received(void *argument);
        void on_halt(void *argument);

    private:
        void play_command( string parameters, StreamOutput* stream );
        void reset_position_if_pending();
        void request_job_lines();
        void maybe_request_job_lines();
        bool next_job_line(char *buf, unsigned int size);
        void handle_pending_link_cmd();
        void cancel_job_stream();
        void stop_job_outputs();
        void complete_abort();
        void abort_finish_tick();
        void broadcast_status();
        void reset_job_state();
        void finish_job();
        void progress_command( string parameters, StreamOutput* stream );
        void abort_command( string parameters, StreamOutput* stream );
        void suspend_command( string parameters, StreamOutput* stream );
        void resume_command( string parameters, StreamOutput* stream );
        void goto_command( string parameters, StreamOutput* stream );
        void buffer_command( string parameters, StreamOutput* stream );
        void upload_command( string parameters, StreamOutput* stream );
        void download_command( string parameters, StreamOutput* stream );
        void test_command(string parameters, StreamOutput* stream );
        string extract_options(string& args);

		
//		int compressfile(string sfilename, string dfilename, StreamOutput* stream);
        // 2024
        // bool check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value);

        string filename;
        string last_filename;
        string after_suspend_gcode;
        string before_resume_gcode;
        string on_boot_gcode;
        StreamOutput* current_stream;
        StreamOutput* reply_stream;

        char md5_str[64];

        std::queue<string> buffered_queue;
        void clear_buffered_queue();

        FILE* current_file_handler;
        // FILE* temp_file_handler;
        long file_size;
        unsigned long played_cnt;
        unsigned long elapsed_secs;
        unsigned long played_lines;
        // Count of queued job lines reported by the streaming peer.
        unsigned long queued_lines;
        unsigned long goto_line;
        unsigned int playing_lines;
        // Filename CRC-16 and the peer's echoed value.
        uint16_t name_crc;
        uint16_t host_name_crc;
        uint8_t current_motion_mode;
        float saved_position[3]; // only saves XYZ
        float slope;
        std::map<uint16_t, float> saved_temperatures;
        // Link message deferred to the main loop.
        uint8_t pending_link_cmd;
        // Deferred playback completion state.
        uint8_t job_ending;
        uint32_t final_lines;
        uint32_t final_percent;
        uint32_t final_playing;
        uint32_t stop_at_us;
        struct {
            bool on_boot_gcode_enable:1;
            bool booted:1;
            bool home_on_boot:1;
            bool playing_file:1;
            bool leave_heaters_on:1;
            bool override_leave_heaters_on:1;
            bool inner_playing:1;
            bool laser_clustering:1;
        };
};
