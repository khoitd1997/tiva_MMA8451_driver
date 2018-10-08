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

#include "debug_utils/swo_segger.h"
#include "utils/uartstdio.h"

// void motionIntHandler(void);

void motionIntHandler(void) {
  SWO_PrintString("Got the interrupt\n");
  // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
  mma8451ReadAccelData();
}

int main(void) {
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  for (uint32_t delayIndex = 0; delayIndex < 50000; ++delayIndex) {
    // delay
  }

  if (false == SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (false == SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {
      // wait for it to be ready
    }
  }

  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

  // use PD2 for interrupt testing

  if (false == SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (false == SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {
      // wait for it to be ready
    }
  }

  GPIOIntDisable(GPIO_PORTD_BASE, GPIO_PIN_2);
  GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_2);
  GPIOIntRegister(GPIO_PORTD_BASE, motionIntHandler);
  GPIOIntRegisterPin(GPIO_PORTD_BASE, GPIO_PIN_2, motionIntHandler);
  GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_2);
  GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_LOW_LEVEL);

  mma8451Init();
  mma8451Reset();
  for (uint32_t delayIndex = 0; delayIndex < 50000; ++delayIndex) {
    // delay
  }
  IntMasterEnable();
  GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_2);
  mma8451Configure();

  for (;;) {
    for (uint32_t delayIndex = 0; delayIndex < 50000; ++delayIndex) {
      // delay
    }
  }
  return 0;
}
