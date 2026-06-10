#include "tmp117_v2.h"
#include "nrf_log_ctrl.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_pwr_mgmt.h"

uint8_t MyTMP117_DeviceID = MyTMP117_GND;

#include "main.h"   /* m_twi, m_xfer_done, m_xfer_error, twi_wait() */

uint16_t TMP117_read_register(uint8_t reg)
{
    uint8_t buffer[2] = {0};

    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_tx(&m_twi, MyTMP117_DeviceID, &reg, sizeof(reg), false) != NRF_SUCCESS
        || !twi_wait()) {
        NRF_LOG_WARNING("TMP117 TX fail reg=0x%02X", reg);
        return TMP117_READ_ERROR;
    }

    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_rx(&m_twi, MyTMP117_DeviceID, buffer, 2) != NRF_SUCCESS
        || !twi_wait()) {
        NRF_LOG_WARNING("TMP117 RX fail reg=0x%02X", reg);
        return TMP117_READ_ERROR;
    }

    return ((uint16_t)(buffer[0] << 8) | buffer[1]);
}

bool TMP117_write_register(uint8_t reg, uint8_t msb, uint8_t lsb)
{
    uint8_t buffer[3] = {reg, msb, lsb};

    m_xfer_done = false; m_xfer_error = false;
    if (nrf_drv_twi_tx(&m_twi, MyTMP117_DeviceID, buffer, sizeof(buffer), false) != NRF_SUCCESS
        || !twi_wait()) {
        NRF_LOG_WARNING("TMP117 write fail reg=0x%02X", reg);
        return false;
    }
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
    /* Auto-scan all four ADDR-pin addresses */
    static const uint8_t addrs[4] = {
        MyTMP117_GND, MyTMP117_VCC, MyTMP117_SDA, MyTMP117_SCL
    };
    uint16_t device_id = TMP117_READ_ERROR;
    for (int i = 0; i < 4; i++) {
        MyTMP117_DeviceID = addrs[i];
        device_id = TMP117_read_register(MyTMP117_ID_Reg);
        if (device_id == TMP117_DEVICE_ID) break;
    }

    if (device_id != TMP117_DEVICE_ID) {
        NRF_LOG_WARNING("TMP117: Not found");
        return false;
    }

    NRF_LOG_INFO("TMP117 found at 0x%02X", MyTMP117_DeviceID);

    tmp117_set_Config(0x02, 0x20);
		
    tmp117_set_Temp_Offset(0x00, 0x00);
		nrf_delay_ms(100);
		
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
