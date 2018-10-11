#include "mma8451.h"
#include "mma8451_info.h"

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

// use PD2 for interrupt testing
#define MMA8451_INT_PORT GPIO_PORTF_BASE
#define MMA8451_INT_PIN GPIO_PIN_2

void motionIntHandler(void) {
  SWO_PrintString("Got the interrupt\n");
  mma8451ReadAccelData();
  GPIOIntClear(GPIO_PORTD_BASE, MMA8451_INT_PIN);
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

  GPIOPinTypeGPIOOutput(MMA8451_INT_PORT, MMA8451_INT_PIN);
  GPIOPinWrite(MMA8451_INT_PORT, MMA8451_INT_PIN, 0);

  if (false == SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (false == SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {
      // wait for it to be ready
    }
  }

  GPIOIntDisable(GPIO_PORTD_BASE, MMA8451_INT_PIN);
  GPIOIntClear(GPIO_PORTD_BASE, MMA8451_INT_PIN);
  GPIOIntRegister(GPIO_PORTD_BASE, motionIntHandler);
  GPIOIntRegisterPin(GPIO_PORTD_BASE, MMA8451_INT_PIN, motionIntHandler);
  GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, MMA8451_INT_PIN);
  GPIOIntTypeSet(GPIO_PORTD_BASE, MMA8451_INT_PIN, GPIO_FALLING_EDGE);

  mma8451Init();

  for (uint32_t delayIndex = 0; delayIndex < 50000; ++delayIndex) {
    // delay
  }
  IntMasterEnable();
  GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_2);

  {
    Mma8451ModeInterruptCfg motionIntCfg = {
        .canWakeupSensor = true,
        .isRoutedPin1    = false,
    };

    Mma8451MotionFreefallCfg motionCfg = {
        .eventLatchEnabled  = true,
        .isMotionMode       = true,
        .eventOnXEnabled    = true,
        .eventOnYEnabled    = true,
        .eventOnZEnabled    = false,
        .isDbounceClearMode = false,
        .thresholdVal       = 30U,
        .countCriteria      = 10U,
        .interruptCfg       = &motionIntCfg,
    };

    Mma8451GeneralInterruptCfg generalIntCfg = {
        .isOpenDrainPin = false,
        .isActiveHigh   = false,
        .fifoBlocked    = false,
    };

    Mma8451SleepCfg sleepCfg = {
        .autoSleepEnabled      = true,
        .sleepInterruptEnabled = true,
        .isRoutedPin1          = true,
        .sleepCount            = 15U,
        .sleepSmplFreq         = MMA8451_ASLP_RATE_50_HZ,
        .sleepPwrMode          = MMA8451_SLEEP_MODE_SAMPL_MODE_NORMAL,
    };

    Mma8451Cfg mm8451Cfg = {.isReducedNoiseMode    = false,
                            .isFastReadMode        = false,
                            .isFullScaleActiveMode = true,
                            .activeSmplFreq        = MMA8451_ODR_800_HZ,
                            .activeMode            = MMA8451_ACTIVE_MODE_SAMPL_MODE_NORMAL,
                            .dataRange             = MMA8451_RANGE_4G,
                            .fifoMode              = MMA8451_FIFO_MOST_RECENT_MODE,
                            .motionCfg             = &motionCfg,
                            .sleepCfg              = &sleepCfg,
                            .generalInterruptCfg   = &generalIntCfg};

    mma8451Configure(mm8451Cfg);
  }

  for (;;) {
    for (uint32_t delayIndex = 0; delayIndex < 50000; ++delayIndex) {
      // delay
    }
    mma8451ReadAccelData();
  }
  return 0;
}
