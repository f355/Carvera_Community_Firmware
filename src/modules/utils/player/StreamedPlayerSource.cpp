#include "StreamedPlayerSource.h"

#include <cstring>

StreamedPlayerSource::StreamedPlayerSource(Line* storage, std::size_t capacity)
    : storage_(storage), capacity_(capacity) {}

void StreamedPlayerSource::begin(uint16_t file_id, unsigned long file_size, uint32_t first_line,
                                 unsigned long byte_position) {
  close();
  file_id_ = file_id;
  file_size_ = file_size;
  consumed_bytes_ = byte_position;
  next_expected_line_ = first_line;
  open_ = true;
}

StreamedPlayerSource::AcceptResult StreamedPlayerSource::accept(uint16_t file_id, uint32_t first_line,
                                                                const uint8_t* data, std::size_t size) {
  if (!open_ || eof_ || data == nullptr || size == 0) {
    return AcceptResult::invalid_state;
  }
  if (file_id != file_id_) return AcceptResult::wrong_file;
  if (first_line != next_expected_line_) return AcceptResult::sequence_error;

  std::size_t lines = 0;
  std::size_t begin = 0;
  for (std::size_t i = 0; i <= size; ++i) {
    const bool newline = i < size && data[i] == '\n';
    if (!newline && i != size) continue;
    if (i == size && begin == i) break;

    const std::size_t length = i - begin + (newline ? 1 : 0);
    if (length >= line_capacity) return AcceptResult::line_too_long;
    ++lines;
    begin = i + (newline ? 1 : 0);
  }

  if (lines > available()) return AcceptResult::queue_full;

  begin = 0;
  for (std::size_t i = 0; i <= size; ++i) {
    const bool newline = i < size && data[i] == '\n';
    if (!newline && i != size) continue;
    if (i == size && begin == i) break;

    const std::size_t length = i - begin + (newline ? 1 : 0);
    Line& slot = storage_[head_];
    std::memcpy(slot.text, data + begin, length);
    slot.text[length] = '\0';
    slot.length = static_cast<uint8_t>(length);
    head_ = (head_ + 1) % capacity_;
    ++count_;
    ++next_expected_line_;
    begin = i + (newline ? 1 : 0);
  }

  return AcceptResult::accepted;
}

bool StreamedPlayerSource::mark_end(uint16_t file_id) {
  if (!open_ || file_id != file_id_) return false;
  eof_ = true;
  return true;
}

PlayerLineSource::ReadResult StreamedPlayerSource::read(char* buffer, std::size_t size) {
  if (!open_) return ReadResult::error;
  if (count_ == 0) return eof_ ? ReadResult::end : ReadResult::waiting;

  Line& slot = storage_[tail_];
  if (buffer == nullptr || size <= slot.length) return ReadResult::error;
  std::memcpy(buffer, slot.text, static_cast<std::size_t>(slot.length) + 1);
  consumed_bytes_ += slot.length;
  slot.length = 0;
  slot.text[0] = '\0';
  tail_ = (tail_ + 1) % capacity_;
  --count_;
  return ReadResult::data;
}

bool StreamedPlayerSource::at_end() const { return open_ && eof_ && count_ == 0; }

void StreamedPlayerSource::close() {
  head_ = 0;
  tail_ = 0;
  count_ = 0;
  file_id_ = 0;
  file_size_ = 0;
  consumed_bytes_ = 0;
  next_expected_line_ = 0;
  open_ = false;
  eof_ = false;
}
