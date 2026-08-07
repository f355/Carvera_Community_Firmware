#if defined(MACHINE_Z1)

#include "RemoteFactoryProvider.h"

#include <cstring>

#include "FactorySettings.h"
#include "PublicData.h"
#include "modules/communication/SerialPacketTransport.h"
#include "utils.h"

namespace {
constexpr remote::PacketTypes factory_packets{PTYPE_FACTORY_START,
                                              PTYPE_FACTORY_VIEW,
                                              PTYPE_FACTORY_DATA,
                                              PTYPE_FACTORY_FINISH,
                                              PTYPE_FACTORY_CANCEL,
                                              remote::factory_record_payload_size,
                                              remote::CompletionPolicy::explicit_finish};
}  // namespace

RemoteFactoryProvider::RemoteFactoryProvider(const FACTORY_SET& current) : original_(current), candidate_(current) {}

remote::Result RemoteFactoryProvider::fetch(SerialConsole& serial) {
  candidate_ = original_;
  SerialPacketTransport transport(serial);
  return remote::receive_records(transport, *this, factory_packets);
}

bool RemoteFactoryProvider::changed() const { return std::memcmp(&original_, &candidate_, sizeof(candidate_)) != 0; }

uint32_t RemoteFactoryProvider::accept(uint32_t, const uint8_t* data, std::size_t size) {
  if (data == nullptr || size == 0 || size > remote::factory_record_payload_size) {
    return 0;
  }
  while (size != 0 && (data[size - 1] == '\r' || data[size - 1] == '\n' || data[size - 1] == '\0')) {
    --size;
  }
  if (size == 0) return 1;

  factory_settings::Setting setting{};
  const auto result = factory_settings::parse(
      std::string_view(reinterpret_cast<const char*>(data), size), setting);
  if (result == factory_settings::ParseResult::invalid) return 0;
  if (result == factory_settings::ParseResult::setting) {
    factory_settings::apply(candidate_, setting);
  }
  return 1;
}

bool RemoteFactoryProvider::commit() { return candidate_.MachineModel == Z1 || candidate_.MachineModel == Z1PRO; }

void RemoteFactoryProvider::rollback() { candidate_ = original_; }

#endif
