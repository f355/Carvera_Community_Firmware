#pragma once

#include <cstdint>
#include <string_view>

inline constexpr uint8_t CARVERA = 1;
inline constexpr uint8_t CARVERA_AIR = 2;
inline constexpr uint8_t Z1 = 3;
inline constexpr uint8_t Z1PRO = 4;

constexpr uint8_t machine_model_from_name(std::string_view name)
{
#if defined(MACHINE_Z1)
    if (name == "Z1") return Z1;
    if (name == "Z1Pro") return Z1PRO;
#else
    if (name == "C1") return CARVERA;
    if (name == "CA1") return CARVERA_AIR;
#endif
    return 0;
}
