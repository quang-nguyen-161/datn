#include "tmp117.h"

#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

extern volatile bool m_xfer_done;   // Reference it (no definition)

uint8_t tmp117_addr = TMP117_GND_ADDRESS;

// TWI instance
static const nrf_drv_twi_t m_twi_tmp117 = NRF_DRV_TWI_INSTANCE(1);

// Indicates if operation on TWI has ended
extern volatile bool m_xfer_done;

uint16_t tmp117_read_register(uint8_t reg)
{
    uint16_t value = 0;
    uint8_t buffer[2] = {0};
    ret_code_t err_code;

    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi_tmp117, tmp117_addr, &reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);

    while (!m_xfer_done);

    m_xfer_done = false;
		
    err_code = nrf_drv_twi_rx(&m_twi_tmp117, tmp117_addr, buffer, 2);
    APP_ERROR_CHECK(err_code);

    while (!m_xfer_done);


    value = ((buffer[0] << 8) | buffer[1]);
    return value;
}

void tmp117_write_register(uint16_t reg, uint8_t value1, uint8_t value2)
{
  ret_code_t err_code;
  static uint8_t buffer[3];
  buffer[0] = reg;
  buffer[1] = value1;
  buffer[2] = value2;
  err_code = nrf_drv_twi_tx(&m_twi_tmp117, tmp117_addr, buffer, sizeof(buffer), false);
  APP_ERROR_CHECK(err_code);
  nrf_delay_ms(1);
}

void tmp117_set_Config(uint8_t first,uint8_t second) //this function will set the configuration register
{
      tmp117_write_register(TMP117_CONFIG_REG, first, second);
}

void tmp117_set_Temp_Offset(uint8_t first, uint8_t second) //this function will set the temperature offset value
{
      //ret_code_t err_code;
      //static uint8_t buf[3];
      //buf[0]=MyTMP117_Temp_Offset;
      //buf[1]=first;     
      //buf[2]=second;    
      ////err_code = nrf_drv_twi_tx(&m_twi_tmp117, MyTMP117_DeviceID, buf, 2, false);
      //err_code = nrf_drv_twi_tx(&m_twi_tmp117, MyTMP117_DeviceID, buf, sizeof(buf), false);
      //APP_ERROR_CHECK(err_code);
      //nrf_delay_ms(1);
      tmp117_write_register(TMP117_TEMP_OFFSET,first,second);
}

void tmp117_set_LowLimit(uint8_t first,uint8_t second)//this function will set the low limit for alert
{
      tmp117_write_register(TMP117_TEMP_LOW_LIMIT,first,second);
}

void tmp117_set_HighLimit(uint8_t first,uint8_t second) //this function will set the the high limit for alert
{ 
      tmp117_write_register(TMP117_TEMP_HIGH_LIMIT,first,second);
}

bool tmp117_Init() //this function will initialize the sensor using custom parameters
{

    uint16_t device_id = 0;
    device_id = tmp117_read_register(TMP117_ID_REG);
	
	NRF_LOG_INFO("true ID: %x", device_id);
	NRF_LOG_FLUSH();
    //make sure the device ID reported by the TMP is correct
    //should always be 0x0117
    if (device_id != TMP117_DEVICE_ID)
    {
		NRF_LOG_INFO("False ID: %d", device_id);
			NRF_LOG_FLUSH();
        return false;
    }
    else
    {
		tmp117_set_Config(0x02,0x20);
		tmp117_set_Temp_Offset(0x00,0x00); 			
		tmp117_set_LowLimit(0x28,0x16);           //Set 22 Celcius
		tmp117_set_HighLimit(0x76,0x80);          //Set 60 Celcius
		NRF_LOG_INFO("tmp init ok");
		NRF_LOG_FLUSH();
		return true;
    }
}
void tmp117_shutdown_mode() 
{
      tmp117_write_register(TMP117_CONFIG_REG,0x01,0x00);
	NRF_LOG_INFO("tmp shutdown mode");
		NRF_LOG_FLUSH();
}

void tmp117_continuous_mode() 
{
      tmp117_write_register(TMP117_CONFIG_REG,0x00, 0x00);
		NRF_LOG_INFO("tmp continuous mode");
		NRF_LOG_FLUSH();
}

uint16_t tmp117_get_Config()
{
      uint16_t Config = tmp117_read_register(TMP117_CONFIG_REG);
      return Config;
}

void tmp117_set_Averaging(TMP117_AVE ave)
{
	uint16_t reg_value = tmp117_get_Config();
	reg_value &= ~ ((1UL << 6) | (1UL << 5));     
	reg_value = reg_value | (( ave & 0x03) << 5);    
	uint8_t first = (reg_value & 0x0F); 
	uint8_t second = reg_value >> 8;
	tmp117_set_Config(first, second);
}
 
float tmp117_get_temp()
{
      float Temp = TMP117_RESOLUTION * tmp117_read_register(TMP117_TEMP_REG);
      return Temp;
}