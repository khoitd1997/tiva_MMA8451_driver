#include "mma8451.h"
#include "mma8451_info.h"
#include "mma8451_utils.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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
#include "tiva_utils/bit_manipulation.h"

static void mma8451ConvertAccelData(uint8_t*       rawData,
                                    int32_t*       outputData,
                                    const uint32_t totalAxis,
                                    const bool     is14Bit) {
  assert(totalAxis > 0);
  uint32_t highBitIndex = 0;
  for (uint32_t axisIndex = 0; axisIndex < totalAxis; ++axisIndex) {
    if (is14Bit) {
      highBitIndex          = axisIndex * 2;
      outputData[axisIndex] = (rawData[axisIndex * 2] << 6) + (rawData[axisIndex * 2 + 1] >> 2);
    } else {
      highBitIndex          = axisIndex;
      outputData[axisIndex] = rawData[axisIndex];
    }
    if (rawData[highBitIndex] > 0x7FU) {
      // sign extend then convert to negative number
      outputData[axisIndex] =
          (-1) * (int32_t)(CONV_TWO_COMPLETMENT(0xFFFFC000U | outputData[axisIndex]));
    }
  }
}

static void mma8451ConverToG(const int32_t*     accelCountData,
                             float*             outputData,
                             const uint32_t     totalAxis,
                             const Mma8451Range rangeFlag,
                             const bool         is14Bit) {
  assert(NULL != accelCountData);
  assert(NULL != outputData);

  float convCoeff = 0.0f;
  switch (rangeFlag) {
    case MMA8451_RANGE_2G:
      if (is14Bit) {
        convCoeff = MMA8451_COEFF_CONV_2G_14Bit;
      } else {
        convCoeff = MMA8451_COEFF_CONV_2G_8Bit;
      }
      break;
    case MMA8451_RANGE_4G:
      if (is14Bit) {
        convCoeff = MMA8451_COEFF_CONV_4G_14Bit;
      } else {
        convCoeff = MMA8451_COEFF_CONV_4G_8Bit;
      }
      break;
    case MMA8451_RANGE_8G:
      if (is14Bit) {
        convCoeff = MMA8451_COEFF_CONV_8G_14Bit;
      } else {
        convCoeff = MMA8451_COEFF_CONV_8G_8Bit;
      }
      break;
    default:
      assert(0 == 1);
      break;
  }

  for (uint32_t axisIndex = 0; axisIndex < totalAxis; ++axisIndex) {
    outputData[axisIndex] = accelCountData[axisIndex] * convCoeff;
  }
}

void mma8451Init(void) {
  if (false == SysCtlPeripheralReady(MMA8451_I2C_PERIPH)) {
    SysCtlPeripheralEnable(MMA8451_I2C_PERIPH);
    while (false == SysCtlPeripheralReady(MMA8451_I2C_PERIPH)) {
      // wait for it to be ready
    }
  }

  if (false == SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (false == SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {
      // wait for it to be ready
    }
  }

  GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);

  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

  I2CMasterInitExpClk(MMA8451_I2C_BASE, SysCtlClockGet(), false);

  mma8451Reset();
  // do dummy receive
  I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, true);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
  mma8451WaitMaster();
}

uint32_t mma8451ReadAccelData(void) {
  mma8451WaitBus();

  uint8_t  accelRawData[MMA8451_BUFFER_LEN + 2] = {0};
  int32_t  accelFinalData[3]                    = {0};
  uint32_t sysStatus                            = 0;

  mma8451ReadReg(MMA8451_MOTION_SRC_ADDR);  // read to clear flag
  sysStatus = mma8451ReadReg(MMA8451_SYSMOD_ADDR);
  if ((sysStatus & 0x02)) {
    SWO_PrintString("Sleep Mode Found, not reading data\n");
  } else if ((sysStatus & 0x01)) {
    SWO_PrintString("Wake Mode Found\n");

    mma8451ReadRegList(MMA8451_DATA_X_MSB, accelRawData, MMA8451_BUFFER_LEN + 2);
    mma8451ConvertAccelData(accelRawData, accelFinalData, 3, true);

    char dataString[100] = "";

    sprintf(dataString,
            "x: %d, y: %d, z: %d\n",
            accelFinalData[0],
            accelFinalData[1],
            accelFinalData[2]);

    SWO_PrintString(dataString);
  }
  return 0;
}

void mma8451Configure(const Mma8451Cfg mma8451Config) {
  // some reg has their own buf since are used in multiple settings
  uint8_t ctrlReg3Buf = 0;
  uint8_t ctrlReg4Buf = 0;
  uint8_t ctrlReg5Buf = 0;

  uint8_t tempRegBuf = 0;

  // xyz data config
  tempRegBuf = 0;
  tempRegBuf |= ((NULL != mma8451Config.hpfCfg) ? MMA8451_HPF_ENABLED : 0);
  tempRegBuf |= mma8451Config.dataRange;
  mma8451WriteReg(MMA8451_XYZ_CFG_ADR, tempRegBuf);

  // hpf cutoff
  tempRegBuf = 0;
  if (NULL != mma8451Config.hpfCfg) { tempRegBuf |= (mma8451Config.hpfCfg)->hpfFreq; }
  mma8451WriteReg(MMA8451_HP_FILTER_CFG_ADDR, tempRegBuf);

  // fifo setup
  tempRegBuf = 0;
  tempRegBuf |= mma8451Config.fifoMode;
  mma8451WriteReg(MMA8451_SETUP_FIFO_ADDR, tempRegBuf);

  // offset register
  if (NULL != mma8451Config.offsetCfg) {
    mma8451WriteReg(MMA8451_OFFSET_X, (mma8451Config.offsetCfg)->offsetX);
    mma8451WriteReg(MMA8451_OFFSET_Y, (mma8451Config.offsetCfg)->offsetY);
    mma8451WriteReg(MMA8451_OFFSET_Z, (mma8451Config.offsetCfg)->offsetZ);
  }

  // motion and freefall stuffs
  if (NULL != (mma8451Config.motionCfg)) {
    tempRegBuf = 0;
    tempRegBuf |=
        (((mma8451Config.motionCfg)->eventLatchEnabled) ? MMA8451_EVENT_LATCH_ENABLED : 0);
    tempRegBuf |= (((mma8451Config.motionCfg)->isMotionMode) ? MMA8451_MOTION_MODE : 0);
    tempRegBuf |= (((mma8451Config.motionCfg)->eventOnXEnabled) ? MMA8451_X_EVENT_ENABLED : 0);
    tempRegBuf |= (((mma8451Config.motionCfg)->eventOnYEnabled) ? MMA8451_Y_EVENT_ENABLED : 0);
    tempRegBuf |= (((mma8451Config.motionCfg)->eventOnZEnabled) ? MMA8451_Z_EVENT_ENABLED : 0);
    mma8451WriteReg(MMA8451_MOTION_CFG_ADDR, tempRegBuf);

    tempRegBuf = 0;
    tempRegBuf |= (((mma8451Config.motionCfg)->isDbounceClearMode) ? MMA8451_DBC_MODE_CLR : 0);
    tempRegBuf |= (mma8451Config.motionCfg)->thresholdVal;
    mma8451WriteReg(MMA8451_MOTION_THRESHOLD_ADDR, tempRegBuf);

    tempRegBuf = 0;
    tempRegBuf |= (mma8451Config.motionCfg)->countCriteria;
    mma8451WriteReg(MMA8451_MOTION_DEBOUNCE_ADDR, tempRegBuf);

    if (NULL != (mma8451Config.motionCfg)->interruptCfg) {
      ctrlReg3Buf |= ((mma8451Config.motionCfg)->interruptCfg->canWakeupSensor
                          ? MMA8451_MOTION_WAKEUPINT_ENABLED
                          : 0);

      // if this struct exists, assume user wants interrupt
      ctrlReg4Buf |= MMA8451_MOTION_INT_ENABLED;
      ctrlReg5Buf |=
          (((mma8451Config.motionCfg)->interruptCfg)->isRoutedPin1 ? MMA8451_MOTION_INT_PIN_INT1
                                                                   : 0);
    }
  }

  // control reg 2
  tempRegBuf = 0;
  if (NULL != mma8451Config.sleepCfg) {
    if ((mma8451Config.sleepCfg)->autoSleepEnabled) { tempRegBuf |= MMA8451_AUTO_SLEEP_ENABLED; }
    tempRegBuf |= (mma8451Config.sleepCfg)->sleepPwrMode;

    if ((mma8451Config.sleepCfg)->sleepInterruptEnabled) {
      ctrlReg4Buf |= MMA8451_WAKEUP_INT_ENABLED;
      ctrlReg5Buf |= ((mma8451Config.sleepCfg)->isRoutedPin1) ? MMA8451_WAKEUP_INT_PIN_INT1 : 0;
    }
  }
  tempRegBuf |= mma8451Config.activeMode;
  mma8451WriteReg(MMA8451_CTRL_REG2, tempRegBuf);

  // control reg 4
  mma8451WriteReg(MMA8451_CTRL_REG4, ctrlReg4Buf);

  // control reg 5
  mma8451WriteReg(MMA8451_CTRL_REG5, ctrlReg5Buf);

  // control reg 3 general interrupt
  if (NULL != mma8451Config.generalInterruptCfg) {
    ctrlReg3Buf |=
        ((mma8451Config.generalInterruptCfg)->isActiveHigh ? MMA8451_INT_POLARITY_HIGH : 0);
    ctrlReg3Buf |=
        ((mma8451Config.generalInterruptCfg)->isOpenDrainPin ? MMA8451_INT_PAD_OPEN_DRAIN : 0);
    ctrlReg3Buf |=
        ((mma8451Config.generalInterruptCfg)->fifoBlocked ? MMA8451_INT_FIFO_BLOCKED : 0);
  }
  mma8451WriteReg(MMA8451_CTRL_REG3, ctrlReg3Buf);

  // control reg1, must be the last one
  tempRegBuf = 0;
  if (NULL != mma8451Config.sleepCfg) { tempRegBuf |= (mma8451Config.sleepCfg)->sleepSmplFreq; }
  tempRegBuf |= (mma8451Config.isFastReadMode ? MMA8451_FAST_READ_MODE : 0);
  tempRegBuf |= (mma8451Config.isReducedNoiseMode ? MMA8451_REDUCED_NOISE_MODE : 0);
  tempRegBuf |= (mma8451Config.isFullScaleActiveMode ? MMA8451_ACTIVE_MODE : 0);
  tempRegBuf |= (mma8451Config.activeSmplFreq);
  mma8451WriteReg(MMA8451_CTRL_REG1, tempRegBuf);

  // old code
  mma8451WriteReg(MMA8451_SETUP_FIFO_ADDR, MMA8451_FIFO_MOST_RECENT_MODE);
  mma8451WriteReg(MMA8451_HP_FILTER_CFG_ADDR, 0);
  mma8451WriteReg(MMA8451_XYZ_CFG_ADR, MMA8451_RANGE_2G | MMA8451_HPF_DISABLED);

  mma8451WriteReg(MMA8451_CTRL_REG2,
                  MMA8451_ACTIVE_MODE_SAMPL_MODE_HIGH_RES | MMA8451_SLEEP_MODE_SAMPL_MODE_NORMAL |
                      MMA8451_AUTO_SLEEP_ENABLED);

  // configure motion detection
  mma8451WriteReg(MMA8451_MOTION_THRESHOLD_ADDR, 8U);
  mma8451WriteReg(MMA8451_MOTION_DEBOUNCE_ADDR, 10U);
  mma8451WriteReg(MMA8451_MOTION_CFG_ADDR,
                  MMA8451_EVENT_LATCH_ENABLED | MMA8451_MOTION_MODE | MMA8451_X_EVENT_ENABLED |
                      MMA8451_Y_EVENT_ENABLED);

  // configure sleep mode
  mma8451WriteReg(MMA8451_ASLP_COUNT_ADDR, 15U);

  // interrupt config
  mma8451WriteReg(MMA8451_CTRL_REG4, MMA8451_WAKEUP_INT_ENABLED | MMA8451_MOTION_INT_ENABLED);
  mma8451WriteReg(MMA8451_CTRL_REG5, MMA8451_WAKEUP_INT_PIN_INT1);
  mma8451WriteReg(
      MMA8451_CTRL_REG3,
      MMA8451_INT_PAD_PUSH_PULL | MMA8451_INT_POLARITY_LOW | MMA8451_MOTION_WAKEUPINT_ENABLED);

  // changing modes should be last
  mma8451WriteReg(MMA8451_CTRL_REG1,
                  MMA8451_ACTIVE_MODE | MMA8451_ODR_800_HZ | MMA8451_ASLP_RATE_12_5_HZ);
}

void mma8451Reset(void) { mma8451WriteReg(MMA8451_CTRL_REG2, 0b01000000); }