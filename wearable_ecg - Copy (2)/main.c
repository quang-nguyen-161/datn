#include <stdint.h>
#include <string.h>

#include "boards.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "main.h"
#include "ble_app.h"
#include "cus_service.h"
#include "ecg.h"
#include "max.h"            /* timer2_init(), max30102_setup(), max30102_cal() */
#include "mma845.h"         /* MMA8452Q_init(), MMA8452Q_read() */
#include "tmp117.h"         /* tmp117_Init(), tmp117_alert_init(), tmp117_poll() */
#include "device_mode.h"
#include "flash_user.h"

/* ================================================================
 *  Peripheral instances — declared extern in main.h
 * ================================================================ */
#define TWI_SCL_PIN  29
#define TWI_SDA_PIN  28

nrf_drv_twi_t    m_twi      = NRF_DRV_TWI_INSTANCE(1);
volatile bool    m_xfer_done = false;
nrf_drv_spi_t    m_lcd_spi  = NRF_DRV_SPI_INSTANCE(0);
sensor_data_t    g_sensor   = {0};

static void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
    if (p_event->type == NRF_DRV_TWI_EVT_DONE) { m_xfer_done = true; }
}

void twi_init(void)
{
    const nrf_drv_twi_config_t cfg = {
        .scl                = TWI_SCL_PIN,
        .sda                = TWI_SDA_PIN,
        .frequency          = NRF_DRV_TWI_FREQ_400K,       /* 400 kHz */
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
        .clear_bus_init     = false
    };
    ret_code_t err = nrf_drv_twi_init(&m_twi, &cfg, twi_handler, NULL);
    APP_ERROR_CHECK(err);
    nrf_drv_twi_enable(&m_twi);
}

/* ================================================================
 *  BLE packet helpers
 * ================================================================ */
#define ECG_PKT_NORMAL  10      /* samples per BLE packet in CONTINUOUS/PERIODIC */
#define ECG_PKT_ECG     50      /* samples per BLE packet in ECG mode             */

static int16_t  s_ecg_buf[ECG_PKT_ECG];
static uint8_t  s_ecg_idx = 0;

static void send_vitals_packet(void)
{
    /* Byte 0: 0x11 = vitals report
     * Byte 1: HR PPG (bpm)
     * Byte 2: SpO2 (%)
     * Bytes 3-4: temp_c × 100 as uint16 big-endian
     * Byte 5: ECG HR (bpm, from on-device R-peak)
     * Byte 6: current mode
     */
    uint8_t pkt[7];
    pkt[0] = 0x11;
    pkt[1] = g_sensor.hr_ppg;
    pkt[2] = g_sensor.spo2;
    uint16_t temp_raw = (uint16_t)(g_sensor.temp * 100.0f);
    pkt[3] = (uint8_t)(temp_raw >> 8);
    pkt[4] = (uint8_t)(temp_raw & 0xFF);
    pkt[5] = g_sensor.hr_ecg;
    pkt[6] = (uint8_t)g_device_mode;
    ble_app_send(pkt, sizeof(pkt));
}

/* ================================================================
 *  Entry point
 * ================================================================ */
int main(void)
{
    log_init();
    timer_init();
    power_management_init();
    twi_init();

    /* Flash / FDS — init before reading saved mode */
    m_record_init();

    ble_app_init();

    /* Sensors */
    max30102_setup();
    MMA8452Q_init(0x1C, SCALE_2G, ODR_100);
    mma8452q_alert_init();  /* configures GPIOTE freefall interrupt if MMA8452Q_INT1_PIN is set */
    tmp117_Init();

    timer2_init();          /* free-running µs counter on TIMER2 */
    ecg_init();             /* SAADC + PPI → 250 Hz autonomous ECG sampling */

    /* Load saved mode from FDS, start sensor timer */
    device_mode_init();

    ble_app_advertising_start();

    /* ── Main loop ─────────────────────────────────────────── */
    while (1)
    {
        /* ── ECG sample ready (all modes) ── */
        if (g_ecg_ready)
        {
            g_ecg_ready = false;
            float filtered = ecg_process(g_ecg_raw);  /* updates g_ecg */

            uint8_t pkt_size = (g_device_mode == MODE_ECG) ? ECG_PKT_ECG : ECG_PKT_NORMAL;
            s_ecg_buf[s_ecg_idx++] = (int16_t)filtered;

            if (s_ecg_idx >= pkt_size)
            {
                s_ecg_idx = 0;
                if (ble_app_is_connected())
                {
                    uint8_t pkt[1 + ECG_PKT_ECG * sizeof(int16_t)];
                    pkt[0] = 0x10;  /* TX type: ECG stream */
                    memcpy(pkt + 1, s_ecg_buf, pkt_size * sizeof(int16_t));
                    uint16_t len = 1 + pkt_size * sizeof(int16_t);
                    ret_code_t err = ble_app_send(pkt, len);
                    if (err != NRF_SUCCESS          &&
                        err != NRF_ERROR_INVALID_STATE &&
                        err != NRF_ERROR_RESOURCES  &&
                        err != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                    {
                        APP_ERROR_HANDLER(err);
                    }
                }
            }
        }

        /* ── Sensor tick (CONTINUOUS and PERIODIC modes) ── */
        if (g_sensor_tick)
        {
            g_sensor_tick = false;

            m_xfer_done = true;     /* guard: only start if bus is free */
            max30102_cal();         /* reads MAX30102 FIFO, updates g_ppg via RF algo */
            MMA8452Q_read();        /* updates g_accel */
            tmp117_poll();          /* reads TMP117 if ALERT fired, updates g_temp */

            if (ble_app_is_connected())
            {
                send_vitals_packet();
            }
        }

        /* ── Idle ── */
        if (!g_ecg_ready && !g_sensor_tick)
        {
            idle_state_handle();
        }
    }
}
