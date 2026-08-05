#ifndef STREAMEDPLAYERSOURCE_H
#define STREAMEDPLAYERSOURCE_H

#include <cstddef>
#include <cstdint>

#include "PlayerLineSource.h"

class StreamedPlayerSource final : public PlayerLineSource {
 public:
  static constexpr std::size_t line_capacity = 130;

  struct Line {
    char text[line_capacity];
    uint8_t length;
  };

  enum class AcceptResult { accepted, wrong_file, sequence_error, line_too_long, queue_full, invalid_state };

  StreamedPlayerSource() = default;
  StreamedPlayerSource(Line* storage, std::size_t capacity);

  void begin(uint16_t file_id, unsigned long file_size, uint32_t first_line = 0, unsigned long byte_position = 0);
  AcceptResult accept(uint16_t file_id, uint32_t first_line, const uint8_t* data, std::size_t size);
  bool mark_end(uint16_t file_id);

  ReadResult read(char* buffer, std::size_t size) override;
  bool at_end() const override;
  unsigned long position() const override { return consumed_bytes_; }
  void close() override;

  bool is_open() const override { return open_; }
  uint16_t file_id() const { return file_id_; }
  unsigned long file_size() const { return file_size_; }
  uint32_t next_expected_line() const { return next_expected_line_; }
  std::size_t queued() const { return count_; }
  std::size_t available() const { return capacity_ - count_; }

 private:
  Line* storage_ = nullptr;
  std::size_t capacity_ = 0;
  std::size_t head_ = 0;
  std::size_t tail_ = 0;
  std::size_t count_ = 0;
  uint16_t file_id_ = 0;
  unsigned long file_size_ = 0;
  unsigned long consumed_bytes_ = 0;
  uint32_t next_expected_line_ = 0;
  bool open_ = false;
  bool eof_ = false;
};

#endif
