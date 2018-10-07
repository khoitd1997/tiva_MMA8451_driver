#include "mma8451.h"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_i2c.h"

#include "inc/hw_ints.h"

#include "inc/hw_memmap.h"

#include "inc/hw_types.h"

#include "driverlib/gpio.h"

#include "driverlib/i2c.h"

#include "driverlib/interrupt.h"

#include "driverlib/pin_map.h"

#include "driverlib/sysctl.h"

#include "driverlib/uart.h"

#include "utils/uartstdio.h"

int main(void) {
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  for (uint32_t delayIndex = 0; delayIndex < 50000; ++delayIndex) {
    // delay
  }

  mma8451Init();
  mma8451Reset();
  for (uint32_t delayIndex = 0; delayIndex < 50000; ++delayIndex) {
    // delay
  }
  mma8451Configure();

  for (;;) {
    // mma8451Configure();
    mma8451ReadAccelData();
    for (uint32_t delayIndex = 0; delayIndex < 50000; ++delayIndex) {
      // delay
    }
  }
  return 0;
}
