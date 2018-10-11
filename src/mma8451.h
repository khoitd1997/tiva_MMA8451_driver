#ifndef _MMA_8451_H
#define _MMA_8451_H

#include <stdbool.h>
#include <stdint.h>
#include "mma8451_info.h"

// having this struct implies enabling interrupt
typedef struct {
  bool canWakeupSensor;
  bool isRoutedPin1;
} Mma8451ModeInterruptCfg;

typedef struct {
  uint8_t offsetX;
  uint8_t offsetY;
  uint8_t offsetZ;
} Mma8451Offset;

typedef struct {
  bool isOpenDrainPin;
  bool isActiveHigh;
  bool fifoBlocked;
} Mma8451GeneralInterruptCfg;

typedef struct {
  bool                autoSleepEnabled;
  bool                sleepInterruptEnabled;
  bool                isRoutedPin1;
  uint8_t             sleepCount;
  Mma8451SleepFreq    sleepSmplFreq;
  Mma8451SleepPwrMode sleepPwrMode;

} Mma8451SleepCfg;

typedef struct {
  bool    eventLatchEnabled;
  bool    isMotionMode;
  bool    eventOnXEnabled;
  bool    eventOnYEnabled;
  bool    eventOnZEnabled;
  bool    isDbounceClearMode;
  uint8_t thresholdVal;
  uint8_t countCriteria;

  // optional config section
  Mma8451ModeInterruptCfg* interruptCfg;
} Mma8451MotionFreefallCfg;

typedef struct {
  Mma8451FilterCutoffFreq hpfFreq;
} highPassFilterCfg;

typedef struct {
  bool                  isReducedNoiseMode;
  bool                  isFastReadMode;
  bool                  isFullScaleActiveMode;
  Mma8451ActiveSmplFreq activeSmplFreq;
  Mma8451ActiveMode     activeMode;
  Mma8451Range          dataRange;

  Mma8451FifoMode fifoMode;

  // optional config section, supply a pointer to activate them
  highPassFilterCfg*          hpfCfg;
  Mma8451MotionFreefallCfg*   motionCfg;
  Mma8451SleepCfg*            sleepCfg;
  Mma8451GeneralInterruptCfg* generalInterruptCfg;
  Mma8451Offset*              offsetCfg;

} Mma8451Cfg;

void                    mma8451Init(void);
uint32_t                mma8451ReadAccelData(void);
void                    mma8451Reset(void);
void                    mma8451SelfTest(void);
Mma8451StatusSystemMode mma8451GetSystemStatus(void);
uint32_t                mm8451GetID(void);
void                    mma8451Configure(const Mma8451Cfg mma8451Config);

#endif