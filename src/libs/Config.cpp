/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

using namespace std;
#include <vector>
#include <string>

#include "libs/Kernel.h"
#include "Config.h"
#include "ConfigValue.h"
#include "ConfigSource.h"
#include "ConfigCache.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "libs/SerialMessage.h"
#include "libs/ConfigSources/FileConfigSource.h"

extern "C" caddr_t _sbrk(int);
#include "libs/ConfigSources/FirmConfigSource.h"
#include "StreamOutputPool.h"

// Add various config sources. Config can be fetched from several places.
// All values are read into a cache, that is then used by modules to read their configuration
Config::Config()
{
    // Config source for firm config found in src/config.default
    static FirmConfigSource firm_config("firm");
    this->add_source(&firm_config);

    // Config source for */config files
    const char *local_path = nullptr;
    if (file_exists("/local/config"))
        local_path = "/local/config";
    else if (file_exists("/local/config.txt"))
        local_path = "/local/config.txt";
    if (local_path != nullptr) {
        static FileConfigSource local_config(local_path, "local");
        this->add_source(&local_config);
    }

    const char *sd_path = nullptr;
    if (file_exists("/sd/config"))
        sd_path = "/sd/config";
    else if (file_exists("/sd/config.txt"))
        sd_path = "/sd/config.txt";
    if (sd_path != nullptr) {
        static FileConfigSource sd_config(sd_path, "sd");
        this->add_source(&sd_config);
    }
}

Config::Config(ConfigSource *cs)
{
    this->add_source(cs);
}

Config::~Config()
{
    config_cache_clear();
}

void Config::add_source(ConfigSource *source)
{
    if (this->config_source_count < this->config_sources.size())
        this->config_sources[this->config_source_count++] = source;
}

// Get a list of modules, used by module "pools" that look for the "enable" keyboard to find things like "moduletype.modulename.enable" as the marker of a new instance of a module
void Config::get_module_list(vector<uint16_t> *list, uint16_t family)
{
    if (!is_config_cache_loaded()) {
        THEKERNEL->streams->printf("ERROR: config cache is not loaded\n");
        THEKERNEL->set_config_load_error(true);
        list->clear();
        return;
    }
    this->config_cache.collect(family, CHECKSUM("enable"), list);
}

// Command to load config cache into buffer for multiple reads during init
void Config::config_cache_load(bool parse)
{
    // First clear the cache
    this->config_cache_clear();

    // Verify the malloc heap hasn't already grown into the config cache region.
    // _sbrk(0) returns the current top of the newlib heap, which is what every
    // allocation on this platform routes through.
    const auto heap_top = reinterpret_cast<uintptr_t>(_sbrk(0));
    const auto cache_start = this->config_cache.start_address();
    if(heap_top > cache_start) {
        THEKERNEL->streams->printf("ERROR: not enough memory to load config cache "
            "(heap=0x%x, cache=0x%x)\n", heap_top, cache_start);
        THEKERNEL->set_config_load_error(true);
        return;
    }

    this->config_cache_loaded = true;

    if(parse) {
        // For each ConfigSource in our stack
        for (size_t i = 0; i < this->config_source_count; ++i)
            this->config_sources[i]->transfer_values_to_cache(&this->config_cache);
    }
}

// Command to clear the config cache after init
void Config::config_cache_clear()
{
    if(this->config_cache_loaded) {
        // Verify the heap didn't grow into the config cache region
        const auto heap_top = reinterpret_cast<uintptr_t>(_sbrk(0));
        const auto cache_start = this->config_cache.start_address();
        if(heap_top > cache_start) {
            THEKERNEL->streams->printf("FATAL: heap collided with config cache "
                "(heap=0x%x, cache=0x%x)\n", heap_top, cache_start);
            system_reset(false);
        }

        this->config_cache.clear();
        this->config_cache_loaded = false;
    }
}

// Three ways to read a value from the config, depending on adress length
ConfigValue *Config::value(uint16_t check_sum_a, uint16_t check_sum_b, uint16_t check_sum_c )
{
    uint16_t check_sums[3];
    check_sums[0] = check_sum_a;
    check_sums[1] = check_sum_b;
    check_sums[2] = check_sum_c;
    return this->value(check_sums);
}

// Get a value from the configuration as a string
// Because we don't like to waste space in Flash with lengthy config parameter names, we take a checksum instead so that the name does not have to be stored
// See get_checksum
ConfigValue *Config::value(uint16_t check_sums[])
{
    if( !is_config_cache_loaded() ) {
        // Cache is unavailable (either failed to load due to heap collision,
        // or value() was called after config_cache_clear()). Surface the
        // condition and return the dummy so callers fall back to defaults
        // instead of dereferencing NULL.
        THEKERNEL->streams->printf("ERROR: config cache is not loaded\n");
        THEKERNEL->set_config_load_error(true);
        ConfigValue::dummy.clear();
        return &ConfigValue::dummy;
    }

    ConfigValue *result = this->config_cache.lookup(check_sums);

    if(result == NULL) {
        ConfigValue::dummy.clear();
        result = &ConfigValue::dummy;
    }

    return result;
}
