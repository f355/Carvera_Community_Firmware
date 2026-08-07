#include "RemoteFactorySettings.h"

#include <cstring>

#include "FactorySettings.h"
#include "PublicData.h"
#include "modules/communication/SerialPacketTransport.h"
#include "utils.h"

namespace {
constexpr remote::PacketTypes factory_packets{PTYPE_FACTORY_START,  PTYPE_FACTORY_VIEW,
                                              PTYPE_FACTORY_DATA,   PTYPE_FACTORY_FINISH,
                                              PTYPE_FACTORY_CANCEL, remote::factory_record_payload_size};
} // namespace

RemoteFactorySettings::RemoteFactorySettings(const FACTORY_SET &current) : original_(current), candidate_(current) {}

remote::Result RemoteFactorySettings::fetch(SerialConsole &serial)
{
    candidate_ = original_;
    SerialPacketTransport transport(serial);
    return remote::receive_records(transport, *this, factory_packets);
}

bool RemoteFactorySettings::changed() const
{
    return std::memcmp(&original_, &candidate_, sizeof(candidate_)) != 0;
}

uint32_t RemoteFactorySettings::accept(uint32_t, const uint8_t *data, std::size_t size)
{
    if (data == nullptr || size == 0 || size > remote::factory_record_payload_size) {
        return 0;
    }
    while (size != 0 && (data[size - 1] == '\r' || data[size - 1] == '\n' || data[size - 1] == '\0')) {
        --size;
    }
    if (size == 0)
        return 1;

    FactorySettings settings(candidate_);
    return settings.apply_line(std::string_view(reinterpret_cast<const char *>(data), size)) ==
                   FactorySettings::LineResult::invalid
               ? 0
               : 1;
}

bool RemoteFactorySettings::commit()
{
    return candidate_.MachineModel == Z1 || candidate_.MachineModel == Z1PRO;
}

void RemoteFactorySettings::rollback()
{
    candidate_ = original_;
}
