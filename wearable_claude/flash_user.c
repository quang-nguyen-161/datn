#include "flash_user.h"

bool volatile m_fds_initialized;

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
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
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
    while (!m_fds_initialized)
    {
        power_manage();
    }
}

ret_code_t m_record_init()
{
		ret_code_t rc;
	 /* Register first to receive an event when initialization is complete. */
    (void) fds_register(fds_evt_handler);
	
    NRF_LOG_INFO("Initializing fds...");

    rc = fds_init();
    APP_ERROR_CHECK(rc);

    /* Wait for fds to initialize. */
    wait_for_fds_ready();

    

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
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t config = {0};

        /* Open the record and read its contents. */
        rc = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(rc);

        /* Copy the configuration from flash into m_dummy_cfg. */
        memcpy(&m_dummy_cfg, config.p_data, sizeof(configuration_t));

        NRF_LOG_INFO("Config file found, updating boot count to %d.", m_dummy_cfg.boot_count);

        /* Update boot count. */
        m_dummy_cfg.boot_count++;

        /* Close the record when done reading. */
        rc = fds_record_close(&desc);
        APP_ERROR_CHECK(rc);

        /* Write the updated record to flash. */
        rc = fds_record_update(&desc, &m_dummy_record);
        if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(rc);
        }
    }
    else
    {
        /* System config not found; write a new one. */
        NRF_LOG_INFO("Writing config file...");

        rc = fds_record_write(&desc, &m_dummy_record);
        if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(rc);
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
	
        if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
						NRF_LOG_INFO("write success");
            APP_ERROR_CHECK(rc);
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
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t config = {0};

   
        /* Write the updated record to flash. */
        rc = fds_record_update(&desc, &record);
				
        if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
			}
        else
        {
						
            APP_ERROR_CHECK(rc);
						NRF_LOG_INFO("update success");
        }
				return rc;
}

ret_code_t m_record_gc()
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
		*len = record.p_header->length_words*BYTES_PER_WORD;
		memcpy(data, record.p_data, *len);
		rc = fds_record_close(&desc);
		NRF_LOG_INFO("read success");
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

