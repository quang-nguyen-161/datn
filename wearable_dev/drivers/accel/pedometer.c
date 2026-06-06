#include "pedometer.h"
#include "nrf_log.h"
#include <string.h>

void pedometer_reset(pedometer_t *p)
{
    if (!p) return;
    bool saved_calib = p->calibration_mode;
    memset(p, 0, sizeof(pedometer_t));
    p->calibration_mode   = saved_calib;
    p->threshold          = PEDOMETER_INITIAL_THRESHOLD;
    p->recent_peak_avg    = PEDOMETER_INITIAL_THRESHOLD * 2.0f;
    p->recent_valley_avg  = 0.0f;
}

static void update_dynamic_threshold(pedometer_t *p, float peak_height)
{
    p->recent_peak_avg = (PEDOMETER_THRESHOLD_ALPHA * peak_height) +
                         ((1.0f - PEDOMETER_THRESHOLD_ALPHA) * p->recent_peak_avg);

    float midpoint      = (p->recent_peak_avg + p->recent_valley_avg) * 0.5f;
    float new_threshold = midpoint * 0.6f;

    if (new_threshold < PEDOMETER_THRESHOLD_MIN) new_threshold = PEDOMETER_THRESHOLD_MIN;
    if (new_threshold > PEDOMETER_THRESHOLD_MAX) new_threshold = PEDOMETER_THRESHOLD_MAX;

    p->threshold = new_threshold;
    p->peak_count_total++;
}

static bool intervals_are_regular(const uint32_t *intervals, uint8_t count)
{
    if (count < 2) return true;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) sum += intervals[i];
    float avg = (float)sum / count;
    if (avg < 1.0f) return false;

    for (uint8_t i = 0; i < count; i++) {
        float ratio = (float)intervals[i] / avg;
        if (ratio < (1.0f - PEDOMETER_INTERVAL_TOLERANCE) ||
            ratio > (1.0f + PEDOMETER_INTERVAL_TOLERANCE)) {
            return false;
        }
    }
    return true;
}

static void add_step_to_cadence(pedometer_t *p, uint32_t interval_ms)
{
    p->step_intervals[p->interval_idx] = interval_ms;
    p->interval_idx = (p->interval_idx + 1) % PEDOMETER_CADENCE_WINDOW;
    if (p->interval_count < PEDOMETER_CADENCE_WINDOW) {
        p->interval_count++;
    }

    if (p->interval_count >= 2) {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < p->interval_count; i++) {
            sum += p->step_intervals[i];
        }
        float avg_ms = (float)sum / p->interval_count;
        if (avg_ms > 1.0f) {
            float raw_cadence = 60000.0f / avg_ms;
            if (p->smooth_cadence < 1.0f) {
                p->smooth_cadence = raw_cadence;
            } else {
                p->smooth_cadence = (PEDOMETER_CADENCE_EMA_ALPHA * raw_cadence) +
                                    ((1.0f - PEDOMETER_CADENCE_EMA_ALPHA) * p->smooth_cadence);
            }
        }
    }
}

bool pedometer_update(pedometer_t *p, float ac_mag, uint32_t timestamp_ms)
{
    if (!p) return false;

    p->prev_ac = p->curr_ac;
    p->curr_ac = ac_mag;

    if (p->prev_ac < 0.001f && p->curr_ac < 0.001f) return false;

    if (ac_mag < p->recent_valley_avg || p->recent_valley_avg < 0.001f) {
        p->recent_valley_avg = (0.1f * ac_mag) + (0.9f * p->recent_valley_avg);
    }

    bool was_rising  = p->rising;
    p->rising        = (ac_mag >= p->prev_ac);
    bool peak_detected = (was_rising && !p->rising && p->prev_ac > p->threshold);

    if (!peak_detected) {
        if (p->calibration_mode && (timestamp_ms % 200 < 20)) {
            NRF_LOG_INFO("[PEDO] ac=%d thr=%d %s pend=%d",
                         (int)(ac_mag * 100), (int)(p->threshold * 100),
                         p->rising ? "R" : "F", p->pending_steps);
        }
        return false;
    }

    update_dynamic_threshold(p, p->prev_ac);

    uint32_t interval = timestamp_ms - p->last_pending_time_ms;

    if (p->last_pending_time_ms > 0 && interval < PEDOMETER_MIN_STEP_INTERVAL_MS) {
        return false;
    }

    if (p->pending_steps < PEDOMETER_VALIDATION_COUNT) {
        if (p->pending_steps > 0) {
            p->pending_intervals[p->pending_steps - 1] = interval;
        }
        p->pending_steps++;
        p->last_pending_time_ms = timestamp_ms;

        if (p->pending_steps >= PEDOMETER_VALIDATION_COUNT) {
            if (intervals_are_regular(p->pending_intervals, PEDOMETER_VALIDATION_COUNT - 1)) {
                p->step_count       += p->pending_steps;
                p->calib_step_count += p->pending_steps;
                p->last_step_time_ms = timestamp_ms;

                for (uint8_t i = 0; i < p->pending_steps - 1; i++) {
                    add_step_to_cadence(p, p->pending_intervals[i]);
                }

                p->pending_steps = 1;
                return true;
            } else {
                p->pending_steps        = 1;
                p->last_pending_time_ms = timestamp_ms;
                return false;
            }
        }
        return false;
    }

    if (interval > PEDOMETER_MAX_STEP_INTERVAL_MS) {
        p->pending_steps        = 1;
        p->last_pending_time_ms = timestamp_ms;
        p->interval_count       = 0;
        p->interval_idx         = 0;
        p->smooth_cadence       = 0.0f;
        return false;
    }

    p->step_count++;
    p->calib_step_count++;
    p->last_step_time_ms    = timestamp_ms;
    p->last_pending_time_ms = timestamp_ms;
    add_step_to_cadence(p, interval);

    if (p->calibration_mode) {
        NRF_LOG_INFO("[PEDO] STEP #%d int=%dms cad=%d",
                     p->step_count, interval, (int)p->smooth_cadence);
    }

    return true;
}

uint32_t pedometer_get_steps(const pedometer_t *p)
{
    if (!p) return 0;
    return p->step_count;
}

float pedometer_get_cadence(const pedometer_t *p, uint32_t now_ms)
{
    if (!p || p->interval_count < 2) return 0.0f;

    if (p->last_step_time_ms > 0 &&
        (now_ms - p->last_step_time_ms) > PEDOMETER_MAX_STEP_INTERVAL_MS) {
        return 0.0f;
    }

    return p->smooth_cadence;
}

float pedometer_get_threshold(const pedometer_t *p)
{
    if (!p) return 0.0f;
    return p->threshold;
}

void pedometer_set_calibration(pedometer_t *p, bool enable)
{
    if (!p) return;
    p->calibration_mode = enable;
    if (enable) {
        p->calib_step_count = 0;
        NRF_LOG_INFO("[PEDO] Calibration ON");
    } else {
        NRF_LOG_INFO("[PEDO] Calibration OFF (%d steps)", p->calib_step_count);
    }
}

bool pedometer_is_calibrating(const pedometer_t *p)
{
    if (!p) return false;
    return p->calibration_mode;
}
