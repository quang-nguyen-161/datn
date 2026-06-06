#ifndef BLE_APP_H
#define BLE_APP_H

#include <stdint.h>
#include <stdbool.h>
#include "cus_service.h"

/* ── Public API ── */

void     ble_app_init(void);        /* stack + GAP + GATT + services + adv + conn_params */
void     ble_app_advertising_start(void);
uint16_t ble_app_conn_handle(void);
bool     ble_app_is_connected(void);

/* Change connection interval at runtime. min_ms / max_ms in milliseconds.
 * If connected, requests update immediately; otherwise takes effect on next connection. */
void     ble_app_set_conn_interval(uint16_t min_ms, uint16_t max_ms);

/* Send one raw notification on the TX characteristic.
 * Returns NRF_SUCCESS or NRF_ERROR_RESOURCES (caller must retry). */
uint32_t ble_app_send(uint8_t const *data, uint16_t len);

/* Access to the custom service instance (needed by ecg.c to call ble_cus_data_send) */
extern ble_cus_t m_cus;

/* System helpers used by main (also init'd here) */
void log_init(void);
void timer_init(void);
void power_management_init(void);
void idle_state_handle(void);

#endif /* BLE_APP_H */
