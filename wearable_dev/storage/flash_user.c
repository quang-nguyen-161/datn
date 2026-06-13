#include "flash_user.h"
#include "device_mode.h"
#include "cmd.h"

bool volatile m_fds_initialized;
static bool   s_fds_init_failed;
static volatile bool s_fds_write_done;
static volatile bool s_cfg_dirty;

static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

/* Default configuration data written on first boot. */
static configuration_t m_dummy_cfg =
{
    .config1_on  = false,
    .config2_on  = true,
    .boot_count  = 0x0,
    .device_name = "ECG_dev",
    .last_mode   = 0,       /* MODE_CONTINUOUS */
    .period_ms_lo = 10000,  /* 10 s default periodic interval (low word) */
    .period_ms_hi = 0,

    .ecg_sample_us   = 4000U,
    .ecg_pkt_samples = 50U,

    .ppg_sample_freq = 100U,
    .ppg_red_ma      = 6U,
    .ppg_ir_ma       = 6U,

    .vital_interval_ms = 1000U,

    .thr_ppg  = {30, 100, 40, 50, 60, 130},
    .thr_ecg  = {40, 100, 50, 120, 40, 130},
    .thr_spo2 = {90, 100, 90, 100, 88, 100},
    .thr_temp = {361, 372, 355, 385, 350, 395},
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

extern const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const * err_str[] =
    {
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
    };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}

extern void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->result == NRF_SUCCESS)
    {
        NRF_LOG_GREEN("Event: %s received (NRF_SUCCESS)",
                      fds_evt_str[p_evt->id]);
    }
    else
    {
        NRF_LOG_GREEN("Event: %s received (%s)",
                      fds_evt_str[p_evt->id],
                      fds_err_str(p_evt->result));
    }

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                m_fds_initialized = true;
            }
            else
            {
                NRF_LOG_ERROR("FDS init failed (0x%08X).", p_evt->result);
                s_fds_init_failed = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
            s_fds_write_done = true;
        } break;

        case FDS_EVT_UPDATE:
        {
            s_fds_write_done = true;
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
        } break;

        default:
            break;
    }
}



void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}

void wait_for_fds_ready(void)
{
    while (!m_fds_initialized && !s_fds_init_failed)
    {
        power_manage();
    }
}

ret_code_t m_record_init(void)
{
		ret_code_t rc;
	 /* Register first to receive an event when initialization is complete. */
    (void) fds_register(fds_evt_handler);
	
    NRF_LOG_INFO("Initializing fds...");

    rc = fds_init();
    APP_ERROR_CHECK(rc);

    /* Wait for fds to initialize. */
    wait_for_fds_ready();

    if (s_fds_init_failed)
    {
        NRF_LOG_ERROR("FDS init failed — skipping config load.");
        return NRF_ERROR_INTERNAL;
    }

    NRF_LOG_INFO("Reading flash usage statistics...");

    fds_stat_t stat = {0};

    rc = fds_stat(&stat);
    APP_ERROR_CHECK(rc);

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
        fds_flash_record_t config = {0};

        rc = fds_record_open(&desc, &config);
        if (rc != NRF_SUCCESS)
        {
            /* CRC mismatch or corrupted record — delete it and write a fresh default. */
            NRF_LOG_WARNING("FDS: record open failed (0x%08X), resetting to defaults.", rc);
            fds_record_delete(&desc);
            rc = NRF_ERROR_NOT_FOUND;
        }
        else
        {
            /* Guard against struct growing between firmware versions:
             * old record has fewer words → memcpy past its end reads garbage. */
            uint32_t stored_bytes   = config.p_header->length_words * sizeof(uint32_t);
            uint32_t expected_bytes = sizeof(configuration_t);

            if (stored_bytes >= expected_bytes)
            {
                memcpy(&m_dummy_cfg, config.p_data, expected_bytes);
            }
            else
            {
                /* Old firmware wrote a smaller struct — copy what is there and
                 * keep defaults for the new fields so they are not garbage. */
                NRF_LOG_WARNING("FDS: config size mismatch (stored=%u expected=%u), "
                                "new fields reset to defaults.", stored_bytes, expected_bytes);
                memcpy(&m_dummy_cfg, config.p_data, stored_bytes);
                m_dummy_cfg.last_mode    = 0;
                m_dummy_cfg.period_ms_lo = 10000;
                m_dummy_cfg.period_ms_hi = 0;
            }

            NRF_LOG_INFO("Config found, boot_count=%d.", m_dummy_cfg.boot_count);
            m_dummy_cfg.boot_count++;

            rc = fds_record_close(&desc);
            if (rc != NRF_SUCCESS)
            {
                NRF_LOG_WARNING("FDS: record close failed (0x%08X).", rc);
            }

            s_fds_write_done = false;
            rc = fds_record_update(&desc, &m_dummy_record);
            if (rc == FDS_ERR_NO_SPACE_IN_FLASH)
            {
                NRF_LOG_WARNING("FDS: no space, triggering GC.");
                fds_gc();
            }
            else if (rc == NRF_SUCCESS)
            {
                /* Wait for the async write to complete so sensor init and
                 * device_mode_init() never race against a pending FDS write. */
                while (!s_fds_write_done) { power_manage(); }
            }
            else
            {
                NRF_LOG_WARNING("FDS: record update failed (0x%08X).", rc);
            }
            return rc;
        }
    }

    if (rc == NRF_ERROR_NOT_FOUND)
    {
        /* No valid record (first boot or deleted after corruption). */
        NRF_LOG_INFO("FDS: writing default config.");

        s_fds_write_done = false;
        rc = fds_record_write(&desc, &m_dummy_record);
        if (rc == FDS_ERR_NO_SPACE_IN_FLASH)
        {
            NRF_LOG_WARNING("FDS: no space for write, triggering GC.");
            fds_gc();
        }
        else if (rc == NRF_SUCCESS)
        {
            while (!s_fds_write_done) { power_manage(); }
        }
        else
        {
            NRF_LOG_WARNING("FDS: record write failed (0x%08X).", rc);
        }
    }
		
		return rc;
}

ret_code_t m_record_write(uint32_t fid, uint32_t key, uint8_t * data, uint32_t len)
{
	ret_code_t rc;
	
	fds_record_t record = {
		.file_id = fid,
		.key = key,
		.data.p_data = data,
		.data.length_words = BYTES_TO_WORDS(len)
	};
	
	rc = fds_record_write(NULL, &record);
	
        if (rc == FDS_ERR_NO_SPACE_IN_FLASH)
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(rc);
            NRF_LOG_INFO("write success");
        }
        return rc;
}

ret_code_t m_record_update(uint32_t fid, uint32_t key, uint8_t * data, uint32_t len)
{
		ret_code_t rc;
		
		fds_record_t record = {
		.file_id = fid,
		.key = key,
		.data.p_data = data,
		.data.length_words = BYTES_TO_WORDS(len)
	};
	
	
		fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    rc = fds_record_find(fid, key, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
        rc = fds_record_update(&desc, &record);
        if (rc == FDS_ERR_NO_SPACE_IN_FLASH)
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else if (rc != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(rc);
        }
        else
        {
            NRF_LOG_INFO("update success");
        }
    }
    else
    {
        /* Record vanished (e.g. deleted after a corrupted-record reset that
         * never recreated it) — create it now so future updates succeed. */
        NRF_LOG_WARNING("m_record_update: record not found (0x%08X), creating it.", rc);
        rc = fds_record_write(NULL, &record);
        if (rc == FDS_ERR_NO_SPACE_IN_FLASH)
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else if (rc != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(rc);
        }
        else
        {
            NRF_LOG_INFO("record created");
        }
    }
    return rc;
}

ret_code_t m_record_gc(void)
{
	return fds_gc();
}

ret_code_t m_record_delete(uint32_t fid, uint32_t key)
{
	ret_code_t rc;
	
	fds_record_desc_t desc = {0};
	fds_find_token_t  tok	 = {0};
	
	rc = fds_record_find(fid, key, &desc, &tok);
	
	if (rc == NRF_SUCCESS)
	{
		rc = fds_record_delete(&desc);
	}
	return rc;
}

ret_code_t m_record_read(uint32_t fid, uint32_t key, uint8_t * data, uint32_t *len)
{
		ret_code_t rc;
	
	fds_record_desc_t desc = {0};
	fds_find_token_t  tok	 = {0};
	fds_flash_record_t record;
	
	rc = fds_record_find(fid, key, &desc, &tok);
	
    if (rc == NRF_SUCCESS)
    {
        rc = fds_record_open(&desc, &record);
        if (rc != NRF_SUCCESS)
        {
            NRF_LOG_WARNING("FDS: m_record_read open failed (0x%08X).", rc);
            return rc;
        }
        uint32_t stored = record.p_header->length_words * BYTES_PER_WORD;
        if (stored < *len) { *len = stored; }
        memcpy(data, record.p_data, *len);
        rc = fds_record_close(&desc);
    }

    return rc;
}

ret_code_t m_record_list_all(void)
{
    ret_code_t rc;
    fds_record_desc_t desc = {0};
    fds_find_token_t tok = {0};

    NRF_LOG_INFO("Listing all valid FDS records...");

    // Reset search token before starting a new search
    memset(&tok, 0x00, sizeof(fds_find_token_t));

    // Find the first record
    while (fds_record_iterate(&desc, &tok) == NRF_SUCCESS)
    {
        fds_flash_record_t record;
        rc = fds_record_open(&desc, &record);
        if (rc == NRF_SUCCESS)
        {
            NRF_LOG_INFO("File ID: 0x%04X | Record Key: 0x%04X | Record ID: 0x%04X | Length: %d bytes",
                         record.p_header->file_id,
                         record.p_header->record_key,
                         record.p_header->record_id,
                         record.p_header->length_words * 4);
            
            fds_record_close(&desc);
        }
        else
        {
            NRF_LOG_WARNING("Failed to open record (error %d)", rc);
        }
    }

    NRF_LOG_INFO("End of record list.");
		return rc;
}

/* ══════════════════════════════════════════════════════════════
 *  Persisted configuration — operating mode, sensor params,
 *  and vital thresholds
 * ════════════════════════════════════════════════════════════ */
void flash_user_load_config(void)
{
    if (m_dummy_cfg.last_mode < MODE_COUNT)
    {
        g_device_mode = (device_mode_t)m_dummy_cfg.last_mode;
    }
    if (m_dummy_cfg.period_ms_lo != 0)
    {
        g_period_ms = m_dummy_cfg.period_ms_lo;
    }
    if (m_dummy_cfg.period_ms_hi != 0)
    {
        g_capture_ms = m_dummy_cfg.period_ms_hi;   /* reused field: PERIODIC capture window (ms) */
    }

    if (m_dummy_cfg.ecg_sample_us != 0)
    {
        g_cmd_sample_us = m_dummy_cfg.ecg_sample_us;
    }
    if (m_dummy_cfg.ecg_pkt_samples != 0)
    {
        g_cmd_pkt_samples = m_dummy_cfg.ecg_pkt_samples;
    }

    if (m_dummy_cfg.ppg_sample_freq != 0)
    {
        g_ppg_sample_freq = m_dummy_cfg.ppg_sample_freq;
    }
    g_ppg_red_ma = m_dummy_cfg.ppg_red_ma;
    g_ppg_ir_ma  = m_dummy_cfg.ppg_ir_ma;

    if (m_dummy_cfg.vital_interval_ms != 0)
    {
        g_vital_interval_ms = m_dummy_cfg.vital_interval_ms;
    }

    g_ecg_stream_enabled = m_dummy_cfg.config1_on;   /* reused legacy bool: ECG stream flag */

    g_thr_ppg_norm_min = m_dummy_cfg.thr_ppg[0];
    g_thr_ppg_norm_max = m_dummy_cfg.thr_ppg[1];
    g_thr_ppg_warn_min = m_dummy_cfg.thr_ppg[2];
    g_thr_ppg_warn_max = m_dummy_cfg.thr_ppg[3];
    g_thr_ppg_dang_min = m_dummy_cfg.thr_ppg[4];
    g_thr_ppg_dang_max = m_dummy_cfg.thr_ppg[5];

    g_thr_ecg_norm_min = m_dummy_cfg.thr_ecg[0];
    g_thr_ecg_norm_max = m_dummy_cfg.thr_ecg[1];
    g_thr_ecg_warn_min = m_dummy_cfg.thr_ecg[2];
    g_thr_ecg_warn_max = m_dummy_cfg.thr_ecg[3];
    g_thr_ecg_dang_min = m_dummy_cfg.thr_ecg[4];
    g_thr_ecg_dang_max = m_dummy_cfg.thr_ecg[5];

    g_thr_spo2_norm_min = m_dummy_cfg.thr_spo2[0];
    g_thr_spo2_norm_max = m_dummy_cfg.thr_spo2[1];
    g_thr_spo2_warn_min = m_dummy_cfg.thr_spo2[2];
    g_thr_spo2_warn_max = m_dummy_cfg.thr_spo2[3];
    g_thr_spo2_dang_min = m_dummy_cfg.thr_spo2[4];
    g_thr_spo2_dang_max = m_dummy_cfg.thr_spo2[5];

    g_thr_temp_norm_min = m_dummy_cfg.thr_temp[0];
    g_thr_temp_norm_max = m_dummy_cfg.thr_temp[1];
    g_thr_temp_warn_min = m_dummy_cfg.thr_temp[2];
    g_thr_temp_warn_max = m_dummy_cfg.thr_temp[3];
    g_thr_temp_dang_min = m_dummy_cfg.thr_temp[4];
    g_thr_temp_dang_max = m_dummy_cfg.thr_temp[5];

    NRF_LOG_INFO("FDS: config loaded — mode=%u period=%ums ecg=%uus/%usmp ppg=%uHz vital=%ums",
                 (unsigned)g_device_mode, (unsigned)g_period_ms,
                 (unsigned)g_cmd_sample_us, (unsigned)g_cmd_pkt_samples,
                 (unsigned)g_ppg_sample_freq, (unsigned)g_vital_interval_ms);
}

void flash_user_mark_dirty(void)
{
    s_cfg_dirty = true;
}

bool flash_user_config_dirty(void)
{
    return s_cfg_dirty;
}

void flash_user_save_config(void)
{
    m_dummy_cfg.last_mode    = (uint8_t)g_device_mode;
    m_dummy_cfg.period_ms_lo = g_period_ms;
    m_dummy_cfg.period_ms_hi = g_capture_ms;   /* reused field: PERIODIC capture window (ms) */

    m_dummy_cfg.ecg_sample_us   = g_cmd_sample_us;
    m_dummy_cfg.ecg_pkt_samples = g_cmd_pkt_samples;

    m_dummy_cfg.ppg_sample_freq = g_ppg_sample_freq;
    m_dummy_cfg.ppg_red_ma      = g_ppg_red_ma;
    m_dummy_cfg.ppg_ir_ma       = g_ppg_ir_ma;

    m_dummy_cfg.vital_interval_ms = g_vital_interval_ms;

    m_dummy_cfg.config1_on = g_ecg_stream_enabled;   /* reused legacy bool: ECG stream flag */

    m_dummy_cfg.thr_ppg[0] = g_thr_ppg_norm_min;
    m_dummy_cfg.thr_ppg[1] = g_thr_ppg_norm_max;
    m_dummy_cfg.thr_ppg[2] = g_thr_ppg_warn_min;
    m_dummy_cfg.thr_ppg[3] = g_thr_ppg_warn_max;
    m_dummy_cfg.thr_ppg[4] = g_thr_ppg_dang_min;
    m_dummy_cfg.thr_ppg[5] = g_thr_ppg_dang_max;

    m_dummy_cfg.thr_ecg[0] = g_thr_ecg_norm_min;
    m_dummy_cfg.thr_ecg[1] = g_thr_ecg_norm_max;
    m_dummy_cfg.thr_ecg[2] = g_thr_ecg_warn_min;
    m_dummy_cfg.thr_ecg[3] = g_thr_ecg_warn_max;
    m_dummy_cfg.thr_ecg[4] = g_thr_ecg_dang_min;
    m_dummy_cfg.thr_ecg[5] = g_thr_ecg_dang_max;

    m_dummy_cfg.thr_spo2[0] = g_thr_spo2_norm_min;
    m_dummy_cfg.thr_spo2[1] = g_thr_spo2_norm_max;
    m_dummy_cfg.thr_spo2[2] = g_thr_spo2_warn_min;
    m_dummy_cfg.thr_spo2[3] = g_thr_spo2_warn_max;
    m_dummy_cfg.thr_spo2[4] = g_thr_spo2_dang_min;
    m_dummy_cfg.thr_spo2[5] = g_thr_spo2_dang_max;

    m_dummy_cfg.thr_temp[0] = g_thr_temp_norm_min;
    m_dummy_cfg.thr_temp[1] = g_thr_temp_norm_max;
    m_dummy_cfg.thr_temp[2] = g_thr_temp_warn_min;
    m_dummy_cfg.thr_temp[3] = g_thr_temp_warn_max;
    m_dummy_cfg.thr_temp[4] = g_thr_temp_dang_min;
    m_dummy_cfg.thr_temp[5] = g_thr_temp_dang_max;

    ret_code_t rc = m_record_update(CONFIG_FILE, CONFIG_REC_KEY,
                                     (uint8_t *)&m_dummy_cfg, sizeof(m_dummy_cfg));
    if (rc == FDS_ERR_NO_SPACE_IN_FLASH)
    {
        NRF_LOG_WARNING("FDS: no space, triggering GC before retry.");
        fds_gc();
        return; /* leave dirty — retry on next loop pass */
    }
    s_cfg_dirty = (rc != NRF_SUCCESS);
}

