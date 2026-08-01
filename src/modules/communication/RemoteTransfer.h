#ifndef REMOTETRANSFER_H
#define REMOTETRANSFER_H

#include <cstddef>
#include <cstdint>

#include "MakeraFrame.h"

namespace remote {

constexpr std::size_t max_record_payload = 132;

struct PacketTypes {
    uint8_t start;
    uint8_t view;
    uint8_t data;
    uint8_t finish;
    uint8_t cancel;
};

enum class Result : uint8_t {
    success,
    cancelled,
    timeout,
    protocol_error,
    sink_error
};

class Transport {
public:
    virtual ~Transport() = default;
    virtual void send(uint8_t type, const uint8_t* payload, std::size_t size) = 0;
    virtual bool receive(makera::Packet& packet, uint32_t timeout_ms) = 0;
    virtual uint32_t now_ms() const = 0;
};

class RecordSink {
public:
    virtual ~RecordSink() = default;
    virtual uint32_t accept(uint32_t index, const uint8_t* data,
                            std::size_t size) = 0;
    virtual bool commit() { return true; }
    virtual void rollback() = 0;
};

Result receive_records(Transport& transport, RecordSink& sink,
                       const PacketTypes& types,
                       uint32_t overall_timeout_ms = 2500);

} // namespace remote

#endif
