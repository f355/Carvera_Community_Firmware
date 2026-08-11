/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl
   (https://github.com/simen/grbl). Smoothie is free software: you can redistribute it and/or modify it under the terms
   of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version. Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
   General Public License for more details. You should have received a copy of the GNU General Public License along with
   Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>

#include "libs/Kernel.h"
#include "libs/Module.h"
using namespace std;
#include <vector>

#include "TemperatureControl.h"
#include "TemperatureControlPool.h"
#ifndef NO_PID_AUTOTUNE
#include "PID_Autotuner.h"
#endif
#include "Config.h"
#include "ConfigValue.h"
#include "TemperatureControlPublicAccess.h"
#include "checksumm.h"

#define enable_checksum CHECKSUM("enable")

void TemperatureControlPool::load_tools() {
  vector<uint16_t> modules;
  THEKERNEL->config->get_module_list(&modules, temperature_control_checksum);
  int cnt = 0;
  for (auto cs : modules) {
    // If module is enabled
    if (THEKERNEL->config->value(temperature_control_checksum, cs, enable_checksum)->as_bool()) {
      TemperatureControl* controller = new TemperatureControl(cs, cnt++);
      THEKERNEL->add_module(controller);
    }
  }

#ifndef NO_PID_AUTOTUNE
  // no need to create one of these if no heaters defined
  if (cnt > 0) {
    PID_Autotuner* pidtuner = new PID_Autotuner();
    THEKERNEL->add_module(pidtuner);
  }
#endif
}
