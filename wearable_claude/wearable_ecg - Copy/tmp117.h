#ifndef      _TMP117_V2_H_
#define      _TMP117_V2_H_	

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"

#define     TMP117_GND_ADDRESS 0x48		//	GND
#define     TMP117_VCC_ADDRESS 0x49    //	VCC
#define     TMP117_SDA_ADDRESS 0x4A    //	SDA
#define     TMP117_SCL_ADDRESS 0x4B    //	SCL

/*------------Pointer Registers-----------------------*/

#define     TMP117_TEMP_REG			 			0x00 
#define     TMP117_CONFIG_REG			 		0x01
#define     TMP117_TEMP_HIGH_LIMIT  	0x02
#define     TMP117_TEMP_LOW_LIMIT  	  0x03
#define     TMP117_EEPROM_UNLOCK      0x04
#define     TMP117_EEPROM1            0x05
#define     TMP117_EEPROM2        	  0x06
#define     TMP117_TEMP_OFFSET        0x07
#define     TMP117_EEPROM3            0x08
#define     TMP117_ID_REG             0x0F

#define TMP117_DEVICE_ID 									0x0117			// Value found in the device ID register on reset (page 24 Table 3 of datasheet)
#define TMP117_RESOLUTION 								0.0078125f	// Resolution of the device, found on (page 1 of datasheet)
#define TMP117_CONTINUOUS_CONVERSION_MODE 0b00				// Continuous Conversion Mode
#define TMP117_ONE_SHOT_MODE							0b11				// One Shot Conversion Mode
#define TMP117_SHUTDOWN_MODE 							0b01				// Shutdown Conversion Mode


typedef enum 
{
	NOAVE = 0, 
	AVE8, 
	AVE32, 
	AVE64
} TMP117_AVE;    //Averaging mode No Average, Average 8,32,64

uint16_t tmp117_read_register(uint8_t reg);
void tmp117_write_register(uint16_t reg, uint8_t value1, uint8_t value2);
void tmp117_set_Config(uint8_t first,uint8_t second); //this function will set the configuration register
void tmp117_set_Temp_Offset(uint8_t first,uint8_t second); //this function will set the temperature offset value
void tmp117_set_LowLimit(uint8_t first,uint8_t second); //this function will set the low limit for alert
void tmp117_set_HighLimit(uint8_t first,uint8_t second); //this function will set the the high limit for alert
bool tmp117_Init(); //this function will initialize the sensor using custom parameters
uint16_t tmp117_get_Config(); //this function will return the configuration register value
void tmp117_set_Averaging(TMP117_AVE ave); //set the high and low limit register for alert
float tmp117_get_temp(); //This function will return the temp in float.
void tmp117_shutdown_mode(void);
void tmp117_continuous_mode(void);

#endif
