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

static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

/* Dummy configuration data. */
static configuration_t m_dummy_cfg =
{
    .config1_on  = false,
    .config2_on  = true,
    .boot_count  = 0x0,
    .device_name = "dummy",
};

/* A record containing dummy configuration data. */
static fds_record_t const m_dummy_record =
{
    .file_id           = CONFIG_FILE,
    .key               = CONFIG_REC_KEY,
    .data.p_data       = &m_dummy_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(m_dummy_cfg) + 3) / sizeof(uint32_t),
};

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;


extern void fds_evt_handler(fds_evt_t const * p_evt);
extern const char *fds_err_str(ret_code_t ret);

void wait_for_fds_ready(void);
ret_code_t m_record_init();
ret_code_t m_record_write(uint32_t fid, uint32_t key, uint8_t * data, uint32_t len);
ret_code_t m_record_update(uint32_t fid, uint32_t key, uint8_t * data, uint32_t len);
ret_code_t m_record_delete(uint32_t fid, uint32_t key);
ret_code_t m_record_gc();
ret_code_t m_record_read(uint32_t fid, uint32_t key, uint8_t * data, uint32_t *len);
ret_code_t m_record_list_all(void);
#endif