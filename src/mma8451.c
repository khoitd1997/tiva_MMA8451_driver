#include "mma8451.h"
#include "mma8451_info.h"

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

#define MMA8451_I2C_BASE I2C0_BASE
#define MMA8451_I2C_PERIPH SYSCTL_PERIPH_I2C0
#define MMA8451_BUFFER_LEN 4

#define CONV_TWO_COMPLETMENT(x) ((~(x)) + 1)
#define SIGN_EXTEND(num, bits) (((~0) << (bits)) | (num))

static void mma8451WaitMaster(void) {
  while (I2CMasterBusy(MMA8451_I2C_BASE)) {
    // wait until the master is not busy
  }
}

static void mma8451WaitBus(void) {
  while (I2CMasterBusBusy(MMA8451_I2C_BASE)) {
    // wait until the bus is not busy
  }
}

static void mma8451WriteReg(const uint8_t regAddr, const uint8_t dataToWrite) {
  mma8451WaitBus();

  I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, false);
  I2CMasterDataPut(MMA8451_I2C_BASE, regAddr);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  mma8451WaitMaster();

  I2CMasterDataPut(MMA8451_I2C_BASE, dataToWrite);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
  mma8451WaitMaster();
}

static uint8_t mma8451ReadReg(const uint8_t regAddr) {
  mma8451WaitBus();

  I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, false);
  I2CMasterDataPut(MMA8451_I2C_BASE, regAddr);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  mma8451WaitMaster();

  I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, true);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
  mma8451WaitMaster();

  return I2CMasterDataGet(MMA8451_I2C_BASE);
}

static void mma8451ReadRegList(const uint8_t startRegAddr,
                               uint8_t*      recvData,
                               const uint8_t totalData) {
  assert(NULL != recvData);
  assert(totalData > 1);

  mma8451WaitBus();

  I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, false);
  I2CMasterDataPut(MMA8451_I2C_BASE, startRegAddr);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  mma8451WaitMaster();

  I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, true);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
  mma8451WaitMaster();
  recvData[0] = I2CMasterDataGet(MMA8451_I2C_BASE);

  uint8_t dataIndex = 1;
  for (dataIndex = 1; dataIndex < totalData - 1; ++dataIndex) {
    I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, true);
    I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    mma8451WaitMaster();
    recvData[dataIndex] = I2CMasterDataGet(MMA8451_I2C_BASE);
  }

  I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, true);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
  mma8451WaitMaster();
  recvData[dataIndex] = I2CMasterDataGet(MMA8451_I2C_BASE);
}

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

  // do dummy receive
  I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, true);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
  mma8451WaitMaster();
}

uint32_t mma8451ReadAccelData(void) {
  mma8451WaitBus();

  uint8_t accelRawData[MMA8451_BUFFER_LEN + 2] = {0};
  int32_t accelFinalData[3]                    = {0};
  mma8451ReadRegList(MMA8451_DATA_X_MSB, accelRawData, MMA8451_BUFFER_LEN + 2);

  mma8451ConvertAccelData(accelRawData, accelFinalData, 3, true);

  char dataString[100] = "";

  sprintf(
      dataString, "x: %d, y: %d, z: %d\n", accelFinalData[0], accelFinalData[1], accelFinalData[2]);
  SWO_PrintString(dataString);

  return 0;
}

void mma8451Configure(void) {
  mma8451WriteReg(MMA8451_SETUP_FIFO_ADDR, MMA8451_FIFO_MOST_RECENT_MODE);
  mma8451WriteReg(MMA8451_XYZ_CFG_ADR, MMA8451_RANGE_8G);

  // changing modes should be last
  mma8451WriteReg(MMA8451_CTRL_REG1, MMA8451_ACTIVE_MODE);

  uint32_t tempBuf        = mma8451ReadReg(MMA8451_SETUP_FIFO_ADDR);
  uint32_t errCode        = I2CMasterErr(MMA8451_I2C_BASE);
  char     dataString[50] = "";

  sprintf(dataString, "status: %d, error: %d\n", tempBuf, errCode);
  SWO_PrintString(dataString);
}

void mma8451Reset(void) { mma8451WriteReg(MMA8451_CTRL_REG2, 0b01000000); }