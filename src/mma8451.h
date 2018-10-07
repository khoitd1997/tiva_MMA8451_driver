#ifndef _MMA_8451_H
#define _MMA_8451_H

#include <stdint.h>

void     mma8451Init(void);
uint32_t mma8451ReadAccelData(void);
void     mma8451Configure(void);
void     mma8451Reset(void);

#endif