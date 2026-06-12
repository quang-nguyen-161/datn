#ifndef __FLASH_USER_H__
#define __FLASH_USER_H__

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "fds.h"
#include "app_timer.h"
#include "app_error.h"
#include "fds_example.h"

#define NRF_LOG_MODULE_NAME app
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


extern bool volatile m_fds_initialized;

extern void fds_evt_handler(fds_evt_t const * p_evt);
extern const char *fds_err_str(ret_code_t ret);

void wait_for_fds_ready(void);
ret_code_t m_record_init(void);
ret_code_t m_record_write(uint32_t fid, uint32_t key, uint8_t * data, uint32_t len);
ret_code_t m_record_update(uint32_t fid, uint32_t key, uint8_t * data, uint32_t len);
ret_code_t m_record_delete(uint32_t fid, uint32_t key);
ret_code_t m_record_gc(void);
ret_code_t m_record_read(uint32_t fid, uint32_t key, uint8_t * data, uint32_t *len);
ret_code_t m_record_list_all(void);

/* ══════════════════════════════════════════════════════════════
 *  Persisted configuration — operating mode, sensor params,
 *  and vital thresholds (cmd.h / device_mode.h globals)
 * ════════════════════════════════════════════════════════════ */

/* Call once after m_record_init(): copies the loaded (or default)
 * FDS record into the runtime globals (g_device_mode, g_period_ms,
 * g_cmd_*, g_ppg_*, g_vital_interval_ms, g_thr_*). */
void flash_user_load_config(void);

/* Marks the in-flash config as stale. Call after any change to the
 * globals above (mode/period switch, gateway CMD_* commands). Cheap
 * and non-blocking — actual flash write happens in flash_user_save_config(). */
void flash_user_mark_dirty(void);

/* True if flash_user_mark_dirty() was called since the last successful save. */
bool flash_user_config_dirty(void);

/* Packs the runtime globals into the config record and queues an FDS
 * update. Non-blocking. Call periodically from the main loop only when
 * m_fds_initialized && flash_user_config_dirty(). */
void flash_user_save_config(void);

#endif
