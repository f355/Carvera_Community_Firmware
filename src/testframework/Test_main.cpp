#include <iostream>
#include <string>
#include <vector>

#include "SerialConsole.h"
#include "gpio.h"
#include "utils.h"

GPIO leds[5] = {GPIO(P1_18), GPIO(P1_19), GPIO(P1_20), GPIO(P1_21), GPIO(P4_28)};

#include "easyunit/test.h"
#include "easyunit/testharness.h"

int main() {
  Kernel* kernel = new Kernel();

  printf("Starting tests...\n");

  TestRegistry::runAndPrint();

  kernel->serial->printf("Done\n");

  // drop back into DFU upload
  kernel->serial->printf("Entering DFU flash mode...\n");
  system_reset(true);

  for (;;) {
  }
}
