#ifndef _TMP117_V2_H_
#define _TMP117_V2_H_

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include <stdlib.h>

#define MyTMP117_GND 0x48
#define MyTMP117_VCC 0x49
#define MyTMP117_SDA 0x4A
#define MyTMP117_SCL 0x4B

#define MyTMP117_TempReg        0x00
#define MyTMP117_ConfigReg      0x01
#define MyTMP117_TempHighLimit  0x02
#define MyTMP117_TempLowLimit   0x03
#define MyTMP117_EEPROM_Unlock  0x04
#define MyTMP117_EEPROM1        0x05
#define MyTMP117_EEPROM2        0x06
#define MyTMP117_Temp_Offset    0x07
#define MyTMP117_EEPROM3        0x08
#define MyTMP117_ID_Reg         0x0F

#define TMP117_DEVICE_ID    0x0117
#define TMP117_RESOLUTION   0.0078125f
#define TMP117_READ_ERROR   0xFFFF

#define TMP117_TEMP_MIN     (-55.0f)
#define TMP117_TEMP_MAX     (150.0f)
#define TMP117_TEMP_INVALID (-999.0f)

#define TMP117_CONTINUOUS_CONVERSION_MODE 0x00
#define TMP117_ONE_SHOT_MODE              0x03
#define TMP117_SHUTDOWN_MODE              0x01

typedef enum {
    NOAVE = 0,
    AVE8,
    AVE32,
    AVE64
} TMP117_AVE;

uint16_t TMP117_read_register(uint8_t reg);
bool     TMP117_write_register(uint8_t reg, uint8_t msb, uint8_t lsb);

void     tmp117_set_Config(uint8_t first, uint8_t second);
void     tmp117_set_Temp_Offset(uint8_t first, uint8_t second);
void     tmp117_set_LowLimit(uint8_t first, uint8_t second);
void     tmp117_set_HighLimit(uint8_t first, uint8_t second);
bool     tmp117_Init(void);
uint16_t tmp117_get_Config(void);
void     tmp117_set_Averaging(TMP117_AVE ave);
float    tmp117_get_Temp(void);
void     tmp117_shutdown_mode(void);
void     tmp117_Continious_mode(void);
void     tmp117_wake_oneshot(void);
float    tmp117_read_temperature(void);

#endif /* _TMP117_V2_H_ */
