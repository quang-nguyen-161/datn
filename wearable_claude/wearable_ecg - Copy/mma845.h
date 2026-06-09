#ifndef MMA8452_H
#define MMA8452_H

#include "nrf_delay.h"
#include "nrf_drv_twi.h"

#include <stdbool.h>
#include <stdint.h>
#include "math.h"

///////////////////////////////////
// MMA8452Q Register Definitions //
///////////////////////////////////
typedef enum
{
    STATUS_MMA8452Q = 0x00,
    OUT_X_MSB = 0x01,
    OUT_X_LSB = 0x02,
    OUT_Y_MSB = 0x03,
    OUT_Y_LSB = 0x04,
    OUT_Z_MSB = 0x05,
    OUT_Z_LSB = 0x06,
    SYSMOD = 0x0B,
    INT_SOURCE = 0x0C,
    WHO_AM_I = 0x0D,
    XYZ_DATA_CFG = 0x0E,
    HP_FILTER_CUTOFF = 0x0F,
    PL_STATUS = 0x10,
    PL_CFG = 0x11,
    PL_COUNT = 0x12,
    PL_BF_ZCOMP = 0x13,
    P_L_THS_REG = 0x14,
    FF_MT_CFG = 0x15,
    FF_MT_SRC = 0x16,
    FF_MT_THS = 0x17,
    FF_MT_COUNT = 0x18,
    TRANSIENT_CFG = 0x1D,
    TRANSIENT_SRC = 0x1E,
    TRANSIENT_THS = 0x1F,
    TRANSIENT_COUNT = 0x20,
    PULSE_CFG = 0x21,
    PULSE_SRC = 0x22,
    PULSE_THSX = 0x23,
    PULSE_THSY = 0x24,
    PULSE_THSZ = 0x25,
    PULSE_TMLT = 0x26,
    PULSE_LTCY = 0x27,
    PULSE_WIND = 0x28,
    ASLP_COUNT = 0x29,
    CTRL_REG1 = 0x2A,
    CTRL_REG2 = 0x2B,
    CTRL_REG3 = 0x2C,
    CTRL_REG4 = 0x2D,
    CTRL_REG5 = 0x2E,
    OFF_X = 0x2F,
    OFF_Y = 0x30,
    OFF_Z = 0x31
} MMA8452Q_Register;

////////////////////////////////
// MMA8452Q Misc Declarations //
////////////////////////////////
typedef enum
{
    SCALE_2G = 2,
    SCALE_4G = 4,
    SCALE_8G = 8
} MMA8452Q_Scale; // Possible full-scale settings

typedef enum
{
    ODR_800,
    ODR_400,
    ODR_200,
    ODR_100,
    ODR_50,
    ODR_12,
    ODR_6,
    ODR_1
} MMA8452Q_ODR; // possible data rates

// Possible portrait/landscape settings
#define PORTRAIT_U 0
#define PORTRAIT_D 1
#define LANDSCAPE_R 2
#define LANDSCAPE_L 3
#define LOCKOUT 0x40
#define MMA8452Q_DEFAULT_ADDRESS 0x1C

// Posible SYSMOD (system mode) States
#define SYSMOD_STANDBY 0x00
#define SYSMOD_WAKE 0x01
#define SYSMOD_SLEEP 0x02

bool MMA8452Q_init( uint8_t addr, MMA8452Q_Scale fsr, MMA8452Q_ODR odr);

void MMA8452Q_read();
uint8_t MMA8452Q_available();
uint8_t MMA8452Q_readTap();
uint8_t MMA8452Q_readPL();


int16_t MMA8452Q_getX();
int16_t MMA8452Q_getY();
int16_t MMA8452Q_getZ();

float MMA8452Q_getCalculatedX();
float MMA8452Q_getCalculatedY();
float MMA8452Q_getCalculatedZ();

bool isRight();
bool isLeft();
bool isUp();
bool isDown();
bool isFlat();

void MMA8452Q_setScale(MMA8452Q_Scale fsr);
void MMA8452Q_setDataRate(MMA8452Q_ODR odr);

void MMA8452Q_standby();
void MMA8452Q_setActive();
bool MMA8452Q_isActive();
void MMA8452Q_setupPL();
void MMA8452Q_setupTap(uint8_t xThs, uint8_t yThs, uint8_t zThs);
bool MMA8452Q_writeRegister(MMA8452Q_Register reg, uint8_t data);
bool MMA8452Q_readRegister(MMA8452Q_Register reg, uint8_t *dest);
bool MMA8452Q_readRegisters(MMA8452Q_Register reg, uint8_t *buffer, uint8_t len);

float read_accelerator(void);
void set_current_accelerator(float current_acc);
float get_current_accelerator(void);
uint32_t MMA8452Q_get_i2c_error_count(void);
#endif