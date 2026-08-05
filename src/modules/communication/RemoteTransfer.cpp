#include "RemoteTransfer.h"

namespace remote {
namespace {

uint32_t read_be32(const uint8_t* data) {
  return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
}

void write_be32(uint8_t* data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value >> 24);
  data[1] = static_cast<uint8_t>(value >> 16);
  data[2] = static_cast<uint8_t>(value >> 8);
  data[3] = static_cast<uint8_t>(value);
}

bool belongs_to(const PacketTypes& types, uint8_t type) {
  return type == types.start || type == types.view || type == types.data || type == types.finish ||
         type == types.cancel;
}

}  // namespace

Result receive_records(Transport& transport, RecordSink& sink, const PacketTypes& types, uint32_t overall_timeout_ms) {
  enum class Phase : uint8_t { start, view, data };

  const uint32_t started_ms = transport.now_ms();
  uint32_t last_progress_ms = started_ms;
  Phase phase = Phase::start;
  uint32_t record_count = 0;
  uint32_t next_index = 1;
  uint8_t request[6]{};
  uint8_t request_type = types.start;
  std::size_t request_size = 0;
  uint8_t receive_failures = 0;
  uint16_t record_payload_size = 0;

  auto send_request = [&] { transport.send(request_type, request_size == 0 ? nullptr : request, request_size); };
  auto fail = [&](Result result) {
    sink.rollback();
    transport.send(types.cancel, nullptr, 0);
    return result;
  };
  auto timeout = [&] {
    if (types.data_timeout_completes && phase == Phase::data && next_index > 1) {
      if (!sink.commit()) return fail(Result::sink_error);
      transport.send(types.cancel, nullptr, 0);
      return Result::success;
    }
    return fail(Result::timeout);
  };

  send_request();
  for (;;) {
    const uint32_t now_ms = transport.now_ms();
    const bool awaiting_terminal_timeout = types.data_timeout_completes && phase == Phase::data && next_index > 1;
    const uint32_t timeout_base_ms = awaiting_terminal_timeout ? last_progress_ms : started_ms;
    if (now_ms - timeout_base_ms >= overall_timeout_ms) {
      return timeout();
    }

    makera::Packet packet{};
    if (!transport.receive(packet, 100)) {
      ++receive_failures;
      if (receive_failures > 25) return timeout();
      if (receive_failures % 5 == 0) send_request();
      continue;
    }
    if (!belongs_to(types, packet.type)) continue;
    last_progress_ms = transport.now_ms();
    receive_failures = 0;

    if (packet.type == types.cancel) {
      sink.rollback();
      transport.send(types.finish, nullptr, 0);
      return Result::cancelled;
    }

    if (phase == Phase::start) {
      if (packet.type != types.start || packet.payload_length != 0) {
        return fail(Result::protocol_error);
      }
      request[0] = request[1] = request[2] = request[3] = 0;
      request[4] = static_cast<uint8_t>(types.record_payload_size >> 8);
      request[5] = static_cast<uint8_t>(types.record_payload_size);
      request_type = types.view;
      request_size = sizeof(request);
      phase = Phase::view;
      send_request();
      continue;
    }

    if (phase == Phase::view) {
      const uint16_t record_limit =
          packet.payload_length == 6
              ? static_cast<uint16_t>((static_cast<uint16_t>(packet.payload[4]) << 8) | packet.payload[5])
              : 0;
      if (packet.type != types.view || packet.payload_length != 6 || record_limit == 0 ||
          record_limit > types.record_payload_size) {
        return fail(Result::protocol_error);
      }
      record_payload_size = record_limit;
      record_count = read_be32(packet.payload);
      if (record_count == 0) {
        if (!sink.commit()) return fail(Result::sink_error);
        transport.send(types.finish, nullptr, 0);
        return Result::success;
      }
      write_be32(request, next_index);
      request_type = types.data;
      request_size = 4;
      phase = Phase::data;
      send_request();
      continue;
    }

    if (packet.type == types.finish) {
      return fail(Result::protocol_error);
    }
    if (packet.type != types.data || packet.payload_length <= 4 || packet.payload_length > 4 + record_payload_size) {
      return fail(Result::protocol_error);
    }

    const uint32_t index = read_be32(packet.payload);
    if (index != next_index) return fail(Result::protocol_error);

    const uint32_t accepted = sink.accept(index, packet.payload + 4, packet.payload_length - 4);
    if (accepted == 0 || accepted > record_count - (next_index - 1)) {
      return fail(accepted == 0 ? Result::sink_error : Result::protocol_error);
    }
    const uint32_t last_index = (next_index - 1) + accepted;
    if (last_index == record_count) {
      if (!sink.commit()) return fail(Result::sink_error);
      transport.send(types.finish, nullptr, 0);
      return Result::success;
    }

    next_index = last_index + 1;
    write_be32(request, next_index);
    send_request();
  }
}

}  // namespace remote
