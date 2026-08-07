#include "Eeprom.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#include "CRC16.h"
#include "FactorySettings.h"
#include "StreamOutput.h"
#include "mbed.h"

namespace {
constexpr uint16_t eeprom_size = 4096;
constexpr uint16_t page_size = 32;
constexpr uint16_t data_address = page_size;
constexpr uint16_t factory_address = 16 * page_size;
constexpr std::size_t dump_row_size = 16;

char hex_digit(uint8_t value)
{
    return value < 10 ? static_cast<char>('0' + value) : static_cast<char>('A' + value - 10);
}

std::array<char, 4 + 1 + dump_row_size * 2 + 1> format_row(uint16_t address,
                                                           const std::array<uint8_t, dump_row_size> &bytes)
{
    std::array<char, 4 + 1 + dump_row_size * 2 + 1> result{};
    for (std::size_t i = 0; i < 4; ++i) {
        result[i] = hex_digit(static_cast<uint8_t>((address >> ((3 - i) * 4)) & 0x0f));
    }
    result[4] = ':';
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        result[5 + i * 2] = hex_digit(bytes[i] >> 4);
        result[6 + i * 2] = hex_digit(bytes[i] & 0x0f);
    }
    return result;
}
} // namespace

class Eeprom::Impl {
  public:
    Impl() : bus(P0_27, P0_28)
    {
        bus.frequency(200000);
    }

    bool read(uint16_t address, uint8_t *data, std::size_t size)
    {
        if (data == nullptr || size == 0 || size > eeprom_size || address > eeprom_size - size)
            return false;
        std::memset(data, 0xff, size);
        bus.is_timed_out();
        bus.start();
        if (!bus.write(0xa0) || !bus.write(address >> 8) || !bus.write(address)) {
            bus.stop();
            bus.is_timed_out();
            return false;
        }
        bus.start();
        if (!bus.write(0xa1)) {
            bus.stop();
            bus.is_timed_out();
            return false;
        }
        for (std::size_t i = 0; i < size; ++i) {
            data[i] = bus.read(i + 1 < size ? mbed::I2C::ACK : mbed::I2C::NoACK);
        }
        bus.stop();
        return !bus.is_timed_out();
    }

    bool write(uint16_t address, const uint8_t *data, std::size_t size, float delay_s)
    {
        std::size_t written = 0;
        while (written < size) {
            const uint16_t current = address + written;
            const std::size_t bytes = std::min<std::size_t>(page_size - current % page_size, size - written);
            bus.start();
            bool ok = bus.write(0xa0) && bus.write(current >> 8) && bus.write(current);
            for (std::size_t i = 0; ok && i < bytes; ++i)
                ok = bus.write(data[written + i]);
            bus.stop();
            if (!ok)
                return false;
            written += bytes;
            wait(delay_s);
        }
        return true;
    }

  private:
    mbed::I2C bus;
};

Eeprom::Eeprom() : impl_(new Impl) {}
Eeprom::~Eeprom()
{
    delete impl_;
}

void Eeprom::read_data(EEPROM_data &data, StreamOutput &output)
{
#if defined(MACHINE_FAMILY_Z1)
    Z1Record stored{};
    if (impl_->read(data_address, reinterpret_cast<uint8_t *>(&stored), sizeof(stored))) {
        decode_z1(stored, data);
    }
    else {
        output.printf("ERROR: EEPROM data read failed\n");
    }
#else
    if (!impl_->read(data_address, reinterpret_cast<uint8_t *>(&data), sizeof(data))) {
        data = {};
        output.printf("ERROR: EEPROM data read failed\n");
    }
#endif
    wait(0.05);
}

void Eeprom::write_data(const EEPROM_data &data, StreamOutput &output)
{
    constexpr std::size_t size = runtime_record_size();
    std::array<uint8_t, size> bytes{};
#if defined(MACHINE_FAMILY_Z1)
    Z1Record stored{};
    if (!impl_->read(data_address, reinterpret_cast<uint8_t *>(&stored), sizeof(stored))) {
        output.printf("ERROR: EEPROM data preservation read failed\n");
        return;
    }
    update_z1(data, stored);
    std::memcpy(bytes.data(), &stored, sizeof(stored));
#else
    std::memcpy(bytes.data(), &data, sizeof(data));
#endif
    if (!impl_->write(data_address, bytes.data(), bytes.size(), 0.1F)) {
        output.printf("ERROR: EEPROM data write failed\n");
    }
}

void Eeprom::erase_data(StreamOutput &output)
{
    std::array<uint8_t, runtime_record_size()> bytes{};
    if (impl_->write(data_address, bytes.data(), bytes.size(), 0.05F)) {
        output.printf("EEPROM data erase finished.\n");
    }
    else {
        output.printf("ERROR: EEPROM data erase failed.\n");
    }
}

bool Eeprom::check_data(EEPROM_data &data)
{
    bool changed = false;
    for (float *value : {&data.TLO, &data.REFMZ, &data.TOOLMZ, &data.reserve}) {
        if (std::isnan(*value)) {
            *value = 0;
            changed = true;
        }
    }
    if (!valid_tool_number(data.TOOL)) {
        data.TOOL = 0;
        changed = true;
    }
    if (data.current_wcs < 0 || data.current_wcs > 5) {
        data.current_wcs = 0;
        changed = true;
    }
    for (int wcs = 0; wcs < 6; ++wcs) {
        if (std::isnan(data.WCSrotation[wcs])) {
            data.WCSrotation[wcs] = 0;
            changed = true;
        }
        for (float &coordinate : data.WCScoord[wcs]) {
            if (std::isnan(coordinate)) {
                coordinate = 0;
                changed = true;
            }
        }
    }
    if ((data.tool_not_calibrated & ~1) != 0) {
        data.tool_not_calibrated = true;
        changed = true;
    }
    return changed;
}

void Eeprom::dump(StreamOutput &output)
{
    std::array<uint8_t, dump_row_size> bytes{};
    uint16_t crc = 0;
    for (uint16_t address = 0; address < eeprom_size; address += bytes.size()) {
        if (!impl_->read(address, bytes.data(), bytes.size())) {
            output.printf("ERROR: EEPROM read failed at %04X\r\n", address);
            return;
        }
        crc = crc16::ccitt_update(crc, bytes.data(), bytes.size());
        const auto row = format_row(address, bytes);
        output.printf("%s\r\n", row.data());
    }
    output.printf("CRC16-CCITT:%04X\r\n", crc);
}

void Eeprom::read_factory_settings(FACTORY_SET &settings)
{
    std::array<uint8_t, sizeof(FACTORY_SET) + 4> bytes{};
    const bool read = impl_->read(factory_address, bytes.data(), bytes.size());
    const uint16_t crc = crc16::ccitt(bytes.data(), sizeof(FACTORY_SET) + 2);
    const bool valid = read && bytes[0] == 0x5a && bytes[1] == 0xa5 && bytes[sizeof(FACTORY_SET) + 2] == (crc & 0xff) &&
                       bytes[sizeof(FACTORY_SET) + 3] == (crc >> 8);
    if (valid) {
        std::memcpy(&settings, bytes.data() + 2, sizeof(settings));
    }
    else {
        settings = FACTORY_SET{Machine::carvera, 0x04, 0, 0};
    }
    if (settings.MachineModel == Machine::carvera)
        settings.FuncSetting |= 0x04;
    wait(0.05);
}

bool Eeprom::write_factory_settings(const FACTORY_SET &settings)
{
    std::array<uint8_t, sizeof(FACTORY_SET) + 4> bytes{0x5a, 0xa5};
    std::memcpy(bytes.data() + 2, &settings, sizeof(settings));
    const uint16_t crc = crc16::ccitt(bytes.data(), sizeof(settings) + 2);
    bytes[sizeof(settings) + 2] = crc;
    bytes[sizeof(settings) + 3] = crc >> 8;
    return impl_->write(factory_address, bytes.data(), bytes.size(), 0.1F);
}

void Eeprom::erase_factory_settings()
{
    std::array<uint8_t, sizeof(FACTORY_SET) + 4> bytes{};
    impl_->write(factory_address, bytes.data(), bytes.size(), 0.05F);
}
