#include "mma8451_utils.h"
#include "mma8451_info.h"

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

void mma8451WriteReg(const uint8_t regAddr, const uint8_t dataToWrite) {
  mma8451WaitBus();

  I2CMasterSlaveAddrSet(MMA8451_I2C_BASE, MMA8451_ADDR, false);
  I2CMasterDataPut(MMA8451_I2C_BASE, regAddr);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  mma8451WaitMaster();

  I2CMasterDataPut(MMA8451_I2C_BASE, dataToWrite);
  I2CMasterControl(MMA8451_I2C_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
  mma8451WaitMaster();
}

uint8_t mma8451ReadReg(const uint8_t regAddr) {
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

void mma8451ReadRegList(const uint8_t startRegAddr, uint8_t* recvData, const uint8_t totalData) {
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

void mma8451WaitMaster(void) {
  while (I2CMasterBusy(MMA8451_I2C_BASE)) {
    // wait until the master is not busy
  }
}

void mma8451WaitBus(void) {
  while (I2CMasterBusBusy(MMA8451_I2C_BASE)) {
    // wait until the bus is not busy
  }
}
