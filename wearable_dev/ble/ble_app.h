#ifndef BLE_APP_H
#define BLE_APP_H

#include <stdint.h>
#include <stdbool.h>

/* ══════════════════════════════════════════════════════════════
 *  BLE — stack init, GAP/GATT, advertising, custom service
 *  (init chain called explicitly from main(), wearable_claude-style —
 *   no separate ble_app module)
 * ════════════════════════════════════════════════════════════ */
void ble_stack_init(void);
void gap_params_init(void);
void gatt_init(void);
void services_init(void);
void advertising_init(void);
void conn_params_init(void);
void advertising_start(void);

/* Thin accessors over the BLE connection state */
uint16_t ble_app_conn_handle(void);
bool     ble_app_is_connected(void);
bool     ble_app_ready_to_send(void);   /* connected AND MTU exchange completed */
uint32_t ble_app_send(uint8_t const *data, uint16_t len);

/* Change connection interval at runtime. min_ms / max_ms in milliseconds.
 * If connected, requests update immediately; otherwise takes effect on next connection. */
void     ble_app_set_conn_interval(uint16_t min_ms, uint16_t max_ms);

#endif /* BLE_APP_H */
