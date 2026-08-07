#ifndef EEPROM_H
#define EEPROM_H

#include <cstddef>

struct EEPROM_data {
    float TLO;
    float REFMZ;
    float TOOLMZ;
    float reserve;
    int TOOL;
    float perm_vars[20];
    bool tool_not_calibrated;
    int current_wcs;
    float WCScoord[6][4];
    float WCSrotation[6];
};

class StreamOutput;
struct FACTORY_SET;

class Eeprom {
  public:
    struct Z1Record {
        float TLO;
        float G54[3];
        float REFMZ;
        float TOOLMZ;
        float reserve;
        int TOOL;
        float G54AB[2];
    };

    static_assert(sizeof(EEPROM_data) == 228, "Unexpected runtime EEPROM record size");
    static_assert(offsetof(EEPROM_data, REFMZ) == 4, "Unexpected runtime REFMZ offset");
    static_assert(offsetof(EEPROM_data, TOOL) == 16, "Unexpected runtime TOOL offset");
    static_assert(sizeof(Z1Record) == 40, "Unexpected Z1 EEPROM layout");
    static_assert(offsetof(Z1Record, G54) == 4, "Unexpected Z1 G54 offset");
    static_assert(offsetof(Z1Record, REFMZ) == 16, "Unexpected Z1 REFMZ offset");
    static_assert(offsetof(Z1Record, TOOL) == 28, "Unexpected Z1 TOOL offset");

    static constexpr std::size_t runtime_record_size()
    {
#if defined(MACHINE_FAMILY_Z1)
        return sizeof(Z1Record);
#else
        return sizeof(EEPROM_data);
#endif
    }

    static constexpr bool valid_tool_number(int tool)
    {
        return tool >= -1;
    }

    static void decode_z1(const Z1Record &stored, EEPROM_data &runtime)
    {
        runtime = {};
        runtime.TLO = stored.TLO;
        runtime.REFMZ = stored.REFMZ;
        runtime.TOOLMZ = stored.TOOLMZ;
        runtime.reserve = stored.reserve;
        runtime.TOOL = stored.TOOL;
        runtime.WCScoord[0][0] = stored.G54[0];
        runtime.WCScoord[0][1] = stored.G54[1];
        runtime.WCScoord[0][2] = stored.G54[2];
        runtime.WCScoord[0][3] = stored.G54AB[0];
    }

    static void update_z1(const EEPROM_data &runtime, Z1Record &stored)
    {
        stored.TLO = runtime.TLO;
        stored.G54[0] = runtime.WCScoord[0][0];
        stored.G54[1] = runtime.WCScoord[0][1];
        stored.G54[2] = runtime.WCScoord[0][2];
        stored.REFMZ = runtime.REFMZ;
        stored.TOOLMZ = runtime.TOOLMZ;
        stored.reserve = runtime.reserve;
        stored.TOOL = runtime.TOOL;
        stored.G54AB[0] = runtime.WCScoord[0][3];
    }

    Eeprom();
    ~Eeprom();

    void read_data(EEPROM_data &data, StreamOutput &output);
    void write_data(const EEPROM_data &data, StreamOutput &output);
    void erase_data(StreamOutput &output);
    bool check_data(EEPROM_data &data);
    void dump(StreamOutput &output);
    void read_factory_settings(FACTORY_SET &settings);
    bool write_factory_settings(const FACTORY_SET &settings);
    void erase_factory_settings();

  private:
    class Impl;
    Impl *impl_;
};

#endif
