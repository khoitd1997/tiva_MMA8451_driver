#ifndef _MMA_8451_INFO_H
#define _MMA_8451_INFO_H

#define MMA8451_ADDR 0x1D  // the default of the breakout board

#define MMA8451_FIFO_STATUS_ADDR 0x00  // fifo status
#define MMA8451_ID_ADDR 0x0D
#define MMA8451_SETUP_FIFO_ADDR 0x09
#define MMA8451_TRIG_CFG_ADDR 0x0A
#define MMA8451_SYSMOD_ADDR 0x0B
#define MMA8451_XYZ_CFG_ADR 0x0E
#define MMA8451_HP_FILTER_CFG_ADDR 0x0F

// motion register
#define MMA8451_MOTION_CFG_ADDR 0x15
#define MMA8451_MOTION_SRC_ADDR 0x16
#define MMA8451_MOTION_THRESHOLD_ADDR 0x17
#define MMA8451_MOTION_DEBOUNCE_ADDR 0x18

// data register
#define MMA8451_DATA_X_MSB 0x01
#define MMA8451_DATA_X_LSB 0x02
#define MMA8451_DATA_Y_MSB 0x03
#define MMA8451_DATA_Y_LSB 0x04
#define MMA8451_DATA_Z_MSB 0x05
#define MMA8451_DATA_Z_LSB 0x06

// sleep/wake register
#define MMA8451_ASLP_COUNT_ADDR 0x29

// control register
#define MMA8451_CTRL_REG1 0x2A
#define MMA8451_CTRL_REG2 0x2B
#define MMA8451_CTRL_REG3 0x2C
#define MMA8451_CTRL_REG4 0x2D
#define MMA8451_CTRL_REG5 0x2E

// offset register
#define MMA8451_OFFSET_X 0x2F
#define MMA8451_OFFSET_Y 0x30
#define MMA8451_OFFSET_Z 0x31

#define MMA8451_ACTIVE_MODE 0b1

#define MMA8451_REDUCED_NOISE_MODE 0b100
#define MMA8451_FAST_READ_MODE 0b10

typedef enum {
  MMA8451_FIFO_DISABLED_MODE    = 0b00000000,
  MMA8451_FIFO_MOST_RECENT_MODE = 0b01000000,
  MMA8451_FIFO_STOP_MODE        = 0b10000000,
  MMA8451_FIFO_TRIGGER_MODE     = 0b11000000
} Mma8451FifoMode;

#define MMA8451_EVENT_LATCH_ENABLED 0b10000000
#define MMA8451_MOTION_MODE 0b01000000
#define MMA8451_FREE_FALL_MODE 0b00000000
#define MMA8451_Z_EVENT_ENABLED 0b00100000
#define MMA8451_Y_EVENT_ENABLED 0b00010000
#define MMA8451_X_EVENT_ENABLED 0b00001000

typedef enum {
  MMA8451_RANGE_2G = 0,
  MMA8451_RANGE_4G = 0b01,
  MMA8451_RANGE_8G = 0b10
} Mma8451Range;

typedef enum {
  MMA8451_HIGHEST_CUTOFF_FREQ,
  MMA8451_MEDIUM_CUTOFF_FREQ,
  MMA8451_LOW_CUTOFF_FREQ,
  MMA8451_LOWEST_CUTOFF_FREQ
} Mma8451FilterCutoffFreq;

#define MMA8451_HPF_ENABLED 0b10000
#define MMA8451_HPF_DISABLED 0b0

#define MMA8451_COEFF_CONV_2G_14Bit 0.00024414f
#define MMA8451_COEFF_CONV_4G_14Bit 0.00048828125f
#define MMA8451_COEFF_CONV_8G_14Bit 0.0009765625f
#define MMA8451_COEFF_CONV_2G_8Bit 0.015625f
#define MMA8451_COEFF_CONV_4G_8Bit 0.03125f
#define MMA8451_COEFF_CONV_8G_8Bit 0.0625f

typedef enum {
  MMA8451_ODR_800_HZ  = 0b00000000,
  MMA8451_ODR_400_HZ  = 0b00001000,
  MMA8451_ODR_200_HZ  = 0b00010000,
  MMA8451_ODR_100_HZ  = 0b00011000,
  MMA8451_ODR_50_HZ   = 0b00100000,
  MMA8451_ODR_12_5_HZ = 0b00101000,
  MMA8451_ODR_6_25_HZ = 0b00110000,
  MMA8451_ODR_1_56_HZ = 0b00111000
} Mma8451ActiveSmplFreq;

typedef enum {
  MMA8451_ASLP_RATE_50_HZ   = 0b00000000,
  MMA8451_ASLP_RATE_12_5_HZ = 0b01000000,
  MMA8451_ASLP_RATE_6_25_HZ = 0b10000000,
  MMA8451_ASLP_RATE_1_56_HZ = 0b11000000
} Mma8451SleepFreq;

#define MMA8451_DBC_MODE_DEC 0b00000000
#define MMA8451_DBC_MODE_CLR 0b10000000

typedef enum {
  MMA8451_SLEEP_MODE_SAMPL_MODE_NORMAL    = 0b00000000,
  MMA8451_SLEEP_MODE_SAMPL_MODE_LNLP      = 0b00001000,
  MMA8451_SLEEP_MODE_SAMPL_MODE_HIGH_RES  = 0b00010000,
  MMA8451_SLEEP_MODE_SAMPL_MODE_LOW_POWER = 0b00011000
} Mma8451SleepPwrMode;

typedef enum {
  MMA8451_ACTIVE_MODE_SAMPL_MODE_NORMAL    = 0b00000,
  MMA8451_ACTIVE_MODE_SAMPL_MODE_LNLP      = 0b00001,
  MMA8451_ACTIVE_MODE_SAMPL_MODE_HIGH_RES  = 0b00010,
  MMA8451_ACTIVE_MODE_SAMPL_MODE_LOW_POWER = 0b00011
} Mma8451ActiveMode;

#define MMA8451_AUTO_SLEEP_ENABLED 0b100

#define MMA8451_FIFO_GATE_BYPASSED 0b0
#define MMA8451_FIFO_FATE_BLOCKED 0b10000000

// wakeup source
#define MMA8451_MOTION_WAKEUPINT_ENABLED 0b1000

// interrupt pin config
#define MMA8451_INT_POLARITY_HIGH 0b10
#define MMA8451_INT_POLARITY_LOW 0b00
#define MMA8451_INT_PAD_PUSH_PULL 0b0
#define MMA8451_INT_PAD_OPEN_DRAIN 0b1

#define MMA8451_WAKEUP_INT_ENABLED 0b10000000
#define MMA8451_MOTION_INT_ENABLED 0b100

#define MMA8451_WAKEUP_INT_PIN_INT1 0b10000000
#define MMA8451_WAKEUP_INT_PIN_INT2 0b0
#define MMA8451_MOTION_INT_PIN_INT1 0b100
#define MMA8451_MOTION_INT_PIN_INT2 0b0

#define MMA8451_INT_FIFO_BLOCKED 0b10000000

typedef enum {
  MMA8451_STANDBY_MODE = 0,
  MMA8451_WAKE_MODE    = 0b01,
  MMA8451_SLEEP_MODE   = 0b10
} Mma8451StatusSystemMode;

#endif

#define MMA8451_I2C_BASE I2C0_BASE
#define MMA8451_I2C_PERIPH SYSCTL_PERIPH_I2C0
#define MMA8451_BUFFER_LEN 4

#define CONV_TWO_COMPLETMENT(x) ((~(x)) + 1)
#define SIGN_EXTEND(num, bits) (((~0) << (bits)) | (num))