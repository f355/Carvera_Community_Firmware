#pragma once

#include <cstdint>
#include <string_view>

enum class Machine : uint8_t {
    unknown = 0,
    carvera = 1,
    carvera_air = 2,
    makera_z1 = 3,
    makera_z1_pro = 4,
};

inline constexpr Machine CARVERA = Machine::carvera;
inline constexpr Machine CARVERA_AIR = Machine::carvera_air;
inline constexpr Machine Z1 = Machine::makera_z1;
inline constexpr Machine Z1PRO = Machine::makera_z1_pro;

constexpr bool is_z1(Machine model)
{
    return model == Machine::makera_z1 || model == Machine::makera_z1_pro;
}

constexpr Machine machine_model_from_name(std::string_view name)
{
#if defined(MACHINE_FAMILY_Z1)
    if (name == "Z1")
        return Machine::makera_z1;
    if (name == "Z1Pro")
        return Machine::makera_z1_pro;
#else
    if (name == "C1")
        return Machine::carvera;
    if (name == "CA1")
        return Machine::carvera_air;
#endif
    return Machine::unknown;
}
