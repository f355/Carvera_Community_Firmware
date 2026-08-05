#ifndef SERIALPACKETTRANSPORT_H
#define SERIALPACKETTRANSPORT_H

#if defined(MACHINE_Z1)

#include "RemoteTransfer.h"

class SerialConsole;

class SerialPacketTransport : public remote::Transport {
 public:
  explicit SerialPacketTransport(SerialConsole& serial) : serial_(serial) {}

  void send(uint8_t type, const uint8_t* payload, std::size_t size) override;
  bool receive(makera::Packet& packet, uint32_t timeout_ms) override;
  uint32_t now_ms() const override;

 private:
  SerialConsole& serial_;
};

#endif
#endif
