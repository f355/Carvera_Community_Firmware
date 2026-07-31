#ifndef PLAYERPUBLICACCESS_H
#define PLAYERPUBLICACCESS_H

#define player_checksum           CHECKSUM("player")
#define is_playing_checksum       CHECKSUM("is_playing")
#define is_suspended_checksum     CHECKSUM("is_suspended")
#define abort_play_checksum       CHECKSUM("abort_play")
#define get_progress_checksum     CHECKSUM("progress")
#define inner_playing_checksum    CHECKSUM("inner_playing")
#define restart_job_checksum    CHECKSUM("restart_job")

struct pad_progress {
    unsigned int percent_complete;
    unsigned long played_lines;
    unsigned long elapsed_secs;
    std::string filename;
};
#endif

#define link_cmd_checksum           CHECKSUM("play_cmd")        // 0x094c
#define link_file_size_checksum     CHECKSUM("play_filesize")   // 0x1275
#define link_name_crc_checksum      0xe3be
#define link_line_queued_checksum   0xd2cc
