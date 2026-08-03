#if defined(MACHINE_Z1)

#include "RemoteConfigSource.h"

#include "FirmConfigSource.h"
#include "libs/ConfigCache.h"
#include "libs/Kernel.h"
#include "libs/StreamOutputPool.h"
#include "modules/communication/SerialPacketTransport.h"
#include "PublicData.h"
#include "utils.h"

namespace {
// TODO(f355): Remove timeout completion after Makera fixes the ESP32 firmware
// to advertise only the configuration records it actually sends.
constexpr remote::PacketTypes config_packets{
    PTYPE_CONFIG_START, PTYPE_CONFIG_VIEW, PTYPE_CONFIG_DATA,
    PTYPE_CONFIG_FINISH, PTYPE_CONFIG_CANCEL,
    remote::config_record_payload_size, true};
}

RemoteConfigSource::RemoteConfigSource(SerialConsole& serial) : serial_(serial)
{
    name_checksum = get_checksum("remote");
}

void RemoteConfigSource::transfer_values_to_cache(ConfigCache* cache)
{
    active_cache_ = cache;
    SerialPacketTransport transport(serial_);
    const remote::Result result = remote::receive_records(
        transport, *this, config_packets);
    active_cache_ = nullptr;

    if (result != remote::Result::success) {
        THEKERNEL->streams->printf(
            "ERROR: remote configuration transfer failed (%u); using embedded profile\n",
            static_cast<unsigned>(result));
        THEKERNEL->set_config_load_error(true);
    }
}

uint32_t RemoteConfigSource::accept(uint32_t, const uint8_t* data,
                                    std::size_t size)
{
    if (active_cache_ == nullptr || data == nullptr || size == 0 ||
        size > remote::config_record_payload_size) {
        return 0;
    }

    uint32_t accepted = 0;
    std::size_t begin = 0;
    for (std::size_t i = 0; i <= size; ++i) {
        const bool delimiter = i == size || data[i] == '\n' || data[i] == '\0';
        if (!delimiter) continue;
        if (i != begin) {
            const std::string line(reinterpret_cast<const char*>(data + begin),
                                   i - begin);
            const std::size_t content = line.find_first_not_of(" \t\r");
            if (content != std::string::npos && line[content] != '#' &&
                process_line_from_ascii_config(line, active_cache_) == nullptr) {
                return 0;
            }
            ++accepted;
        }
        begin = i + 1;
    }
    return accepted;
}

void RemoteConfigSource::rollback()
{
    if (active_cache_ == nullptr) return;
    active_cache_->clear();
    FirmConfigSource embedded("firm");
    embedded.transfer_values_to_cache(active_cache_);
}

bool RemoteConfigSource::is_named(uint16_t check_sum)
{
    return check_sum == name_checksum;
}

bool RemoteConfigSource::write(std::string, std::string)
{
    return false;
}

std::string RemoteConfigSource::read(uint16_t[3])
{
    return {};
}

#endif
