#include "tmp117_v2.h"

#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_pwr_mgmt.h"

uint8_t MyTMP117_DeviceID = MyTMP117_GND;

extern const nrf_drv_twi_t m_twi;

uint16_t TMP117_read_register(uint8_t reg)
{
    uint8_t buffer[2] = {0};
    ret_code_t err_code;

    err_code = nrf_drv_twi_tx(&m_twi, MyTMP117_DeviceID, &reg, sizeof(reg), false);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_WARNING("TMP117 I2C TX fail reg=0x%02X err=0x%X", reg, err_code);
        return TMP117_READ_ERROR;
    }

    err_code = nrf_drv_twi_rx(&m_twi, MyTMP117_DeviceID, buffer, 2);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_WARNING("TMP117 I2C RX fail reg=0x%02X err=0x%X", reg, err_code);
        return TMP117_READ_ERROR;
    }

    return ((uint16_t)(buffer[0] << 8) | buffer[1]);
}

bool TMP117_write_register(uint8_t reg, uint8_t msb, uint8_t lsb)
{
    ret_code_t err_code;
    uint8_t buffer[3];
    buffer[0] = reg;
    buffer[1] = msb;
    buffer[2] = lsb;
    err_code = nrf_drv_twi_tx(&m_twi, MyTMP117_DeviceID, buffer, sizeof(buffer), false);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_WARNING("TMP117 I2C write fail reg=0x%02X err=0x%X", reg, err_code);
        return false;
    }
    nrf_delay_ms(1);
    return true;
}

void tmp117_set_Config(uint8_t first, uint8_t second)
{
    TMP117_write_register(MyTMP117_ConfigReg, first, second);
}

void tmp117_set_Temp_Offset(uint8_t first, uint8_t second)
{
    TMP117_write_register(MyTMP117_Temp_Offset, first, second);
}

void tmp117_set_LowLimit(uint8_t first, uint8_t second)
{
    TMP117_write_register(MyTMP117_TempLowLimit, first, second);
}

void tmp117_set_HighLimit(uint8_t first, uint8_t second)
{
    TMP117_write_register(MyTMP117_TempHighLimit, first, second);
}

bool tmp117_Init(void)
{
    uint16_t device_id = TMP117_read_register(MyTMP117_ID_Reg);

    if (device_id == TMP117_READ_ERROR) {
        NRF_LOG_ERROR("TMP117: Cannot communicate via I2C");
        return false;
    }

    NRF_LOG_INFO("TMP117 ID: 0x%04X", device_id);

    if (device_id != TMP117_DEVICE_ID) {
        NRF_LOG_ERROR("TMP117: Wrong ID=0x%04X, expected 0x%04X", device_id, TMP117_DEVICE_ID);
        return false;
    }

    tmp117_set_Config(0x02, 0x20);
    tmp117_set_Temp_Offset(0x00, 0x00);
    tmp117_set_LowLimit(0x16, 0x80);
    tmp117_set_HighLimit(0x1E, 0x00);
    return true;
}

void tmp117_shutdown_mode(void)
{
    TMP117_write_register(MyTMP117_ConfigReg, 0x01, 0x00);
}

void tmp117_Continious_mode(void)
{
    TMP117_write_register(MyTMP117_ConfigReg, 0x00, 0x00);
}

uint16_t tmp117_get_Config(void)
{
    return TMP117_read_register(MyTMP117_ConfigReg);
}

void tmp117_set_Averaging(TMP117_AVE ave)
{
    uint16_t reg_value = tmp117_get_Config();
    if (reg_value == TMP117_READ_ERROR) return;

    reg_value &= ~((1UL << 6) | (1UL << 5));
    reg_value |= ((ave & 0x03) << 5);

    uint8_t msb = (uint8_t)(reg_value >> 8);
    uint8_t lsb = (uint8_t)(reg_value & 0xFF);
    tmp117_set_Config(msb, lsb);
}

float tmp117_get_Temp(void)
{
    uint16_t raw = TMP117_read_register(MyTMP117_TempReg);
    if (raw == TMP117_READ_ERROR) {
        NRF_LOG_WARNING("TMP117: Read temp failed");
        return TMP117_TEMP_INVALID;
    }

    float temp = TMP117_RESOLUTION * (int16_t)raw;

    if (temp < TMP117_TEMP_MIN || temp > TMP117_TEMP_MAX) {
        NRF_LOG_WARNING("TMP117: Temp out of range: %d C", (int)temp);
    }

    return temp;
}

void tmp117_wake_oneshot(void)
{
    uint16_t cfg = tmp117_get_Config();
    if (cfg == TMP117_READ_ERROR) return;

    cfg &= ~(0x03 << 8);
    cfg |= (0x03 << 8);

    tmp117_set_Config((uint8_t)(cfg >> 8), (uint8_t)(cfg & 0xFF));
}

float tmp117_read_temperature(void)
{
    float temp = tmp117_get_Temp();
    NRF_LOG_INFO("TMP117 Temp = " NRF_LOG_FLOAT_MARKER " C", NRF_LOG_FLOAT(temp));
    return temp;
}
