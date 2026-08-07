#include "SerialPacketTransport.h"

#include "SerialConsole.h"
#include "mbed.h"

void SerialPacketTransport::send(uint8_t type, const uint8_t *payload, std::size_t size)
{
    serial_.PacketMessage(static_cast<char>(type), reinterpret_cast<const char *>(payload), size);
}

bool SerialPacketTransport::receive(makera::Packet &packet, uint32_t timeout_ms)
{
    return serial_.receive_packet(packet, timeout_ms) == 0;
}

uint32_t SerialPacketTransport::now_ms() const
{
    return us_ticker_read() / 1000;
}
