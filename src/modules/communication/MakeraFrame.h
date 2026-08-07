#ifndef MAKERAFRAME_H
#define MAKERAFRAME_H

#include <cstddef>
#include <cstdint>

#include "libs/CRC16.h"

namespace makera {

constexpr uint16_t header = 0x8668;
constexpr uint16_t footer = 0x55aa;
// Matches the 544-byte frame buffer historically used by the Carvera parser.
constexpr std::size_t max_payload_size = 535;

struct Packet {
    uint16_t length;
    uint8_t type;
    uint16_t payload_length;
    uint8_t payload[max_payload_size];
    uint16_t crc;
    bool crc_valid;
};

enum class DecodeResult : uint8_t { incomplete, complete, invalid_length, invalid_footer };

class FrameDecoder {
  public:
    DecodeResult feed(uint8_t byte, Packet &packet);
    void reset();
    bool in_progress() const
    {
        return received_ != 0;
    }

  private:
    void restart_with(uint8_t byte);
    std::size_t received_ = 0;
    std::size_t expected_ = 0;
    uint8_t length_high_ = 0;
    uint8_t footer_high_ = 0;
    uint16_t calculated_crc_ = 0;
    bool header_prefix_ = false;
};

} // namespace makera

#endif
