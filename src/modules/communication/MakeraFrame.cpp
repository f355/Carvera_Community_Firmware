#include "MakeraFrame.h"

namespace makera {

void FrameDecoder::reset() {
  received_ = 0;
  expected_ = 0;
  length_high_ = 0;
  footer_high_ = 0;
  calculated_crc_ = 0;
  header_prefix_ = false;
}

void FrameDecoder::restart_with(uint8_t byte) {
  reset();
  header_prefix_ = byte == static_cast<uint8_t>(header >> 8);
}

DecodeResult FrameDecoder::decode_byte(uint8_t byte) {
  if (received_ == 0) {
    if (header_prefix_ && byte == static_cast<uint8_t>(header)) {
      received_ = 2;
      header_prefix_ = false;
    } else {
      header_prefix_ = byte == static_cast<uint8_t>(header >> 8);
    }
    return DecodeResult::incomplete;
  }

  const std::size_t position = received_++;
  if (position == 2) {
    length_high_ = byte;
    calculated_crc_ = crc16::ccitt_update(calculated_crc_, &byte, 1);
    return DecodeResult::incomplete;
  }
  if (position == 3) {
    const uint16_t length = (static_cast<uint16_t>(length_high_) << 8) | byte;
    if (length < 3 || length > max_data_size + 3) {
      restart_with(byte);
      return DecodeResult::invalid_length;
    }
    packet_.length = length;
    packet_.data_length = length - 3;
    calculated_crc_ = crc16::ccitt_update(calculated_crc_, &byte, 1);
    expected_ = static_cast<std::size_t>(length) + 6;
    return DecodeResult::incomplete;
  }

  if (position == 4) {
    packet_.type = byte;
    calculated_crc_ = crc16::ccitt_update(calculated_crc_, &byte, 1);
  } else if (position < static_cast<std::size_t>(packet_.length) + 2) {
    packet_.data[position - 5] = byte;
    calculated_crc_ = crc16::ccitt_update(calculated_crc_, &byte, 1);
  } else if (position == static_cast<std::size_t>(packet_.length) + 2) {
    packet_.crc = static_cast<uint16_t>(byte) << 8;
  } else if (position == static_cast<std::size_t>(packet_.length) + 3) {
    packet_.crc |= byte;
    packet_.crc_valid = packet_.crc == calculated_crc_;
  } else if (position == expected_ - 2) {
    footer_high_ = byte;
  }

  if (received_ < expected_) {
    return DecodeResult::incomplete;
  }

  const uint16_t received_footer = (static_cast<uint16_t>(footer_high_) << 8) | byte;
  if (position != expected_ - 1 || received_footer != footer) {
    const bool consumed_header = received_footer == header;
    restart_with(byte);
    if (consumed_header) {
      received_ = 2;
      header_prefix_ = false;
    }
    return DecodeResult::invalid_footer;
  }

  reset();
  return DecodeResult::complete;
}

}  // namespace makera
