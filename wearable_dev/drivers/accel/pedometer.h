#ifndef PEDOMETER_H
#define PEDOMETER_H

#include <stdbool.h>
#include <stdint.h>

#define PEDOMETER_MIN_STEP_INTERVAL_MS  250
#define PEDOMETER_MAX_STEP_INTERVAL_MS  2000

#define PEDOMETER_INITIAL_THRESHOLD     0.4f
#define PEDOMETER_THRESHOLD_MIN         0.15f
#define PEDOMETER_THRESHOLD_MAX         3.0f
#define PEDOMETER_THRESHOLD_ALPHA       0.2f

#define PEDOMETER_VALIDATION_COUNT      3
#define PEDOMETER_INTERVAL_TOLERANCE    0.5f

#define PEDOMETER_CADENCE_WINDOW        8
#define PEDOMETER_CADENCE_EMA_ALPHA     0.3f

typedef struct {
    uint32_t step_count;
    uint32_t last_step_time_ms;

    float    prev_ac;
    float    curr_ac;
    bool     rising;

    float    threshold;
    float    recent_peak_avg;
    float    recent_valley_avg;
    uint16_t peak_count_total;

    uint8_t  pending_steps;
    uint32_t pending_intervals[PEDOMETER_VALIDATION_COUNT];
    uint32_t last_pending_time_ms;

    uint32_t step_intervals[PEDOMETER_CADENCE_WINDOW];
    uint8_t  interval_idx;
    uint8_t  interval_count;
    float    smooth_cadence;

    bool     calibration_mode;
    uint32_t calib_step_count;
} pedometer_t;

void     pedometer_reset(pedometer_t *p);
bool     pedometer_update(pedometer_t *p, float ac_mag, uint32_t timestamp_ms);
uint32_t pedometer_get_steps(const pedometer_t *p);
float    pedometer_get_cadence(const pedometer_t *p, uint32_t now_ms);
float    pedometer_get_threshold(const pedometer_t *p);
void     pedometer_set_calibration(pedometer_t *p, bool enable);
bool     pedometer_is_calibrating(const pedometer_t *p);

#endif /* PEDOMETER_H */
