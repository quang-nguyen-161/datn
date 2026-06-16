/**
 * Copyright (c) 2017 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef FDS_EXAMPLE_H__
#define FDS_EXAMPLE_H__

#include <stdint.h>
#include <stdbool.h>
/* File ID and Key used for the configuration record. */

#define CONFIG_FILE     (0x8010)
#define CONFIG_REC_KEY  (0x7010)

/* Colors used to print on the console. */

#define COLOR_GREEN     "\033[1;32m"
#define COLOR_YELLOW    "\033[1;33m"
#define COLOR_CYAN      "\033[1;36m"

/* Macros to print on the console using colors. */

#define NRF_LOG_CYAN(...)   NRF_LOG_INFO(COLOR_CYAN   __VA_ARGS__)
#define NRF_LOG_YELLOW(...) NRF_LOG_INFO(COLOR_YELLOW __VA_ARGS__)
#define NRF_LOG_GREEN(...)  NRF_LOG_INFO(COLOR_GREEN  __VA_ARGS__)


/* Device configuration stored in flash. */
typedef struct
{
    uint32_t boot_count;        /* incremented on every power-up                */
    char     device_name[16];   /* BLE advertised name (null-terminated)        */
    bool     config1_on;        /* legacy fields — keep for backwards compat    */
    bool     config2_on;
    uint8_t  last_mode;         /* device_mode_t: 0=CONTINUOUS,1=PERIODIC,2=ECG */
    uint8_t  _pad0;             /* explicit padding — keeps struct 4-byte aligned */
    uint16_t period_ms_lo;      /* periodic mode wake-to-wake interval (ms)     */
    uint16_t period_ms_hi;      /* periodic mode capture window (ms)            */

    /* ECG sampling config — CMD_ECG_CFG (cmd.h: g_cmd_sample_us / g_cmd_pkt_samples) */
    uint32_t ecg_sample_us;
    uint16_t ecg_pkt_samples;

    /* PPG (MAX30102) config — CMD_PPG_CFG */
    uint16_t ppg_sample_freq;    /* Hz */
    uint8_t  ppg_red_ma;
    uint8_t  ppg_ir_ma;

    /* Vital reporting interval — CMD_VITAL_CFG */
    uint16_t vital_interval_ms;

    /* LCD dashboard refresh interval (CONTINUOUS mode) — CMD_VITAL_CFG bytes 4-5 */
    uint16_t lcd_interval_ms;

    /* Vital thresholds — CMD_THR. Layout matches cmd.h g_thr_* globals:
     * each array is {norm_min, norm_max, warn_min, warn_max, dang_min, dang_max}. */
    uint8_t  thr_ppg[6];         /* PPG heart rate (bpm)        */
    uint8_t  thr_ecg[6];         /* ECG heart rate (bpm)        */
    uint8_t  thr_spo2[6];        /* SpO2 (%)                    */
    uint16_t thr_temp[6];        /* Temperature (×10 °C)        */

    /* PPG HR source LED — CMD_PPG_CFG byte 5. Appended at the end so older
     * (smaller) records stay backward compatible — see flash_user.c load. */
    uint8_t  ppg_hr_source;      /* 0=IR, 1=RED */
} configuration_t;


/* Defined in main.c */

void delete_all_begin(void);

/* Defined in cli.c */

void cli_init(void);
void cli_start(void);
void cli_process(void);
bool record_delete_next(void);


#endif
