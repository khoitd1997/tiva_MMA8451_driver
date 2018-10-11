#ifndef _MMA8451_UTILS_H
#define _MMA8451_UTILS_H
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

void    mma8451WriteReg(const uint8_t regAddr, const uint8_t dataToWrite);
uint8_t mma8451ReadReg(const uint8_t regAddr);
void    mma8451ReadRegList(const uint8_t startRegAddr, uint8_t* recvData, const uint8_t totalData);
void    mma8451WaitMaster(void);
void    mma8451WaitBus(void);
#endif