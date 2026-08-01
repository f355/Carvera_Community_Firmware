#if defined(MACHINE_Z1)

#include "RemoteFactoryProvider.h"

#include <cstdlib>
#include <cstring>
#include <string>

#include "checksumm.h"
#include "utils.h"
#include "modules/communication/SerialPacketTransport.h"
#include "PublicData.h"

namespace {
constexpr remote::PacketTypes factory_packets{
    PTYPE_FACTORY_START, PTYPE_FACTORY_VIEW, PTYPE_FACTORY_DATA,
    PTYPE_FACTORY_FINISH, PTYPE_FACTORY_CANCEL};

constexpr uint16_t machine_model_checksum = CHECKSUM("Machine_Model");
constexpr uint16_t a_axis_home_checksum = CHECKSUM("A_Axis_home_enable");
constexpr uint16_t c_axis_home_checksum = CHECKSUM("C_Axis_home_enable");
constexpr uint16_t atc_checksum = CHECKSUM("Atc_enable");
constexpr uint16_t ce1_expand_checksum = CHECKSUM("CE1_Expand");

void set_bit(char& value, uint8_t bit, bool enabled)
{
    uint8_t bits = static_cast<uint8_t>(value);
    if (enabled) bits |= static_cast<uint8_t>(1U << bit);
    else bits &= static_cast<uint8_t>(~(1U << bit));
    value = static_cast<char>(bits);
}
}

RemoteFactoryProvider::RemoteFactoryProvider(const FACTORY_SET& current)
    : original_(current), candidate_(current)
{
}

remote::Result RemoteFactoryProvider::fetch(SerialConsole& serial)
{
    candidate_ = original_;
    SerialPacketTransport transport(serial);
    return remote::receive_records(transport, *this, factory_packets);
}

bool RemoteFactoryProvider::changed() const
{
    return std::memcmp(&original_, &candidate_, sizeof(candidate_)) != 0;
}

uint32_t RemoteFactoryProvider::accept(uint32_t, const uint8_t* data,
                                       std::size_t size)
{
    if (data == nullptr || size == 0 || size > remote::max_record_payload) return 0;
    while (size != 0 && (data[size - 1] == '\r' || data[size - 1] == '\n' ||
                         data[size - 1] == '\0')) {
        --size;
    }
    if (size == 0) return 1;

    const std::string line(reinterpret_cast<const char*>(data), size);
    const std::size_t key_begin = line.find_first_not_of(" \t");
    if (key_begin == std::string::npos || line[key_begin] == '#') return 1;
    const std::size_t key_end = line.find_first_of(" \t", key_begin);
    if (key_end == std::string::npos) return 0;
    const std::size_t value_begin = line.find_first_not_of(" \t", key_end);
    if (value_begin == std::string::npos || line[value_begin] == '#') return 0;

    const std::string key = line.substr(key_begin, key_end - key_begin);
    char* end = nullptr;
    const long parsed = std::strtol(line.c_str() + value_begin, &end, 10);
    if (end == line.c_str() + value_begin || parsed < 0 || parsed > 255) return 0;
    while (*end == ' ' || *end == '\t' || *end == '\r') ++end;
    if (*end != '\0' && *end != '#') return 0;

    const uint16_t checksum = get_checksum(key);
    const uint8_t value = static_cast<uint8_t>(parsed);
    switch (checksum) {
        case machine_model_checksum:
            candidate_.MachineModel = value;
            break;
        case a_axis_home_checksum:
            set_bit(candidate_.FuncSetting, 0, value == 1);
            break;
        case c_axis_home_checksum:
            set_bit(candidate_.FuncSetting, 1, value == 1);
            break;
        case atc_checksum:
            set_bit(candidate_.FuncSetting, 2, value == 1);
            break;
        case ce1_expand_checksum:
            set_bit(candidate_.FuncSetting, 3, value == 1);
            break;
        default:
            break;
    }
    return 1;
}

bool RemoteFactoryProvider::commit()
{
    return candidate_.MachineModel == Z1 || candidate_.MachineModel == Z1PRO;
}

void RemoteFactoryProvider::rollback()
{
    candidate_ = original_;
}

#endif
