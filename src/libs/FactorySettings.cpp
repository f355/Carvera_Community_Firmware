#include "FactorySettings.h"

#include <limits>

namespace {

bool whitespace(char value)
{
    return value == ' ' || value == '\t' || value == '\r' || value == '\n';
}

} // namespace

FactorySettings::Key FactorySettings::key_from_name(std::string_view name)
{
    if (name == "Machine_Model")
        return Key::machine_model;
    if (name == "A_Axis_home_enable")
        return Key::a_axis_home;
    if (name == "C_Axis_home_enable")
        return Key::c_axis_home;
    if (name == "Atc_enable")
        return Key::atc;
    if (name == "CE1_Expand")
        return Key::ce1_expand;
    return Key::unknown;
}

void FactorySettings::set_bit(uint8_t bit, bool enabled)
{
    uint8_t bits = static_cast<uint8_t>(values_.FuncSetting);
    bits = enabled ? static_cast<uint8_t>(bits | (1U << bit)) : static_cast<uint8_t>(bits & ~(1U << bit));
    values_.FuncSetting = static_cast<char>(bits);
}

FactorySettings::LineResult FactorySettings::apply_line(std::string_view line)
{
    std::size_t cursor = 0;
    while (cursor < line.size() && whitespace(line[cursor]))
        ++cursor;
    if (cursor == line.size() || line[cursor] == '#')
        return LineResult::ignored;

    const std::size_t key_begin = cursor;
    while (cursor < line.size() && !whitespace(line[cursor]))
        ++cursor;
    if (cursor == line.size())
        return LineResult::invalid;
    const Key key = key_from_name(line.substr(key_begin, cursor - key_begin));

    while (cursor < line.size() && whitespace(line[cursor]))
        ++cursor;
    if (cursor == line.size() || line[cursor] == '#')
        return LineResult::invalid;

    unsigned value = 0;
    bool have_digit = false;
    while (cursor < line.size() && line[cursor] >= '0' && line[cursor] <= '9') {
        have_digit = true;
        value = value * 10U + static_cast<unsigned>(line[cursor] - '0');
        if (value > std::numeric_limits<uint8_t>::max())
            return LineResult::invalid;
        ++cursor;
    }
    if (!have_digit)
        return LineResult::invalid;

    while (cursor < line.size() && whitespace(line[cursor]))
        ++cursor;
    if (cursor != line.size() && line[cursor] != '#')
        return LineResult::invalid;

    switch (key) {
    case Key::machine_model:
        values_.MachineModel = static_cast<Machine>(value);
        break;
    case Key::a_axis_home:
        set_bit(0, value == 1);
        break;
    case Key::c_axis_home:
        set_bit(1, value == 1);
        break;
    case Key::atc:
        set_bit(2, value == 1);
        break;
    case Key::ce1_expand:
        set_bit(3, value == 1);
        break;
    case Key::unknown:
        return LineResult::ignored;
    }
    return LineResult::applied;
}
