/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"

#include "modules/tools/laser/Laser.h"
#include "modules/tools/canopen/CANopen.h"
#include "modules/tools/spindle/SpindleMaker.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPool.h"
#include "modules/tools/endstops/Endstops.h"
#include "modules/tools/zprobe/ZProbe.h"
#include "modules/tools/scaracal/SCARAcal.h"
#include "RotaryDeltaCalibration.h"
#include "modules/tools/switch/SwitchPool.h"
#include "modules/tools/temperatureswitch/TemperatureSwitch.h"
#include "modules/tools/drillingcycles/Drillingcycles.h"
#include "modules/tools/atc/ATCHandler.h"
#ifndef NO_UTILS_WIFI
#include "modules/utils/wifi/WifiProvider.h"
#endif
#include "modules/robot/Conveyor.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/configurator/Configurator.h"
#include "modules/utils/player/Player.h"
#include "modules/utils/mainbutton/MainButton.h"
#include "Config.h"
#include "StepTicker.h"
#include "SlowTicker.h"
#include "Robot.h"

#include "libs/gpio.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

#include "StreamOutputPool.h"
#include "ToolManager.h"

#include "platform_memory.h"

#include "mbed.h"

GPIO leds[4] = {
    GPIO(P4_29),
    GPIO(P4_28),
	GPIO(P0_4),
    GPIO(P1_17)
};

void init() {

    // Default pins to low status
    for (int i = 0; i < 4; i++){
        leds[i].output();
        leds[i]= 0;
    }


    GPIO beep = GPIO(P1_14);
    beep.output();
    beep = 0;

    Kernel* kernel = new Kernel();

    SimpleShell::version_command("", kernel->streams);

    #ifdef NONETWORK
        kernel->streams->printf("NETWORK is disabled\r\n");
    #endif

    // Create and add main modules
    kernel->add_module( new(AHB0) Player() );

    #ifndef NO_TOOLS_CANOPEN
    kernel->add_module( new(AHB0) CANopen() );
    #endif

    // ATC Handler
    kernel->add_module( new(AHB0) ATCHandler() );

    kernel->add_module( new(AHB0) MainButton() );
    // Wifi Provider
    #ifndef NO_UTILS_WIFI
    kernel->add_module( new(AHB0) WifiProvider() );
    #endif


    // these modules can be completely disabled in the Makefile by adding to EXCLUDE_MODULES
    #ifndef NO_TOOLS_SWITCH
    SwitchPool *sp= new SwitchPool();
    sp->load_tools();
    delete sp;
    #endif

    #ifndef NO_TOOLS_EXTRUDER
    // NOTE this must be done first before Temperature control so ToolManager can handle Tn before temperaturecontrol module does
    ExtruderMaker *em= new(AHB0) ExtruderMaker();
    em->load_tools();
    delete em;
    #endif

    // #ifndef NO_TOOLS_TEMPERATURECONTROL
    // Note order is important here must be after extruder so Tn as a parameter will get executed first
    TemperatureControlPool *tp= new(AHB0) TemperatureControlPool();
    tp->load_tools();
    delete tp;

    // #endif
    #ifndef NO_TOOLS_ENDSTOPS
    kernel->add_module( new(AHB0) Endstops() );
    #endif
    #ifndef NO_TOOLS_LASER
    kernel->add_module( new(AHB0) Laser() );
    #endif

    #ifndef NO_TOOLS_SPINDLE
    SpindleMaker *sm = new(AHB0) SpindleMaker();
    sm->load_spindle();
    delete sm;
    //kernel->add_module( new(AHB0) Spindle() );
    #endif
    #ifndef NO_UTILS_PANEL
    // kernel->add_module( new(AHB0) Panel() );
    #endif
    #ifndef NO_TOOLS_ZPROBE
    kernel->add_module( new(AHB0) ZProbe() );
    #endif
    #ifndef NO_TOOLS_SCARACAL
    kernel->add_module( new(AHB0) SCARAcal() );
    #endif
    #ifndef NO_TOOLS_ROTARYDELTACALIBRATION
    kernel->add_module( new(AHB0) RotaryDeltaCalibration() );
    #endif
//    #ifndef NONETWORK
//    kernel->add_module( new Network() );
//    #endif
    #ifndef NO_TOOLS_TEMPERATURESWITCH
    // Must be loaded after TemperatureControl
    kernel->add_module( new(AHB0) TemperatureSwitch() );
    #endif
    #ifndef NO_TOOLS_DRILLINGCYCLES
    kernel->add_module( new(AHB0) Drillingcycles() );
    #endif
    // clear up the config cache to save some memory
    kernel->config->config_cache_clear();

    if(kernel->is_using_leds()) {
        // set some leds to indicate status... led0 init done, led1 mainloop running, led2 idle loop running
        leds[0]= 1; // indicate we are done with init
    }

    // start the timers and interrupts
    THEKERNEL->conveyor->start(THEROBOT->get_number_registered_motors());
    THEKERNEL->step_ticker->start();
    THEKERNEL->slow_ticker->start();
}

int main()
{
    init();

    uint16_t cnt= 0;
    // Main loop
    while(1){
        if(THEKERNEL->is_using_leds()) {
            // flash led 2 to show we are alive
            leds[1]= (cnt++ & 0x1000) ? 1 : 0;
        }
        THEKERNEL->call_event(ON_MAIN_LOOP);
        THEKERNEL->call_event(ON_IDLE);
    }
}
