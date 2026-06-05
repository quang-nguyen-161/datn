#include "accel_filter.h"
#include <string.h>
#include <math.h>

static float get_median_of_3(float a, float b, float c) {
    if (a > b) {
        if (b > c) return b;
        else if (a > c) return c;
        else return a;
    } else {
        if (a > c) return a;
        else if (b > c) return c;
        else return b;
    }
}

void accel_filter_reset(accel_filter_t *f) {
    if (!f) return;
    memset(f, 0, sizeof(accel_filter_t));
}

float accel_filter_update(accel_filter_t *f, float raw) {
    if (!f) return raw;

    // ── Stage 1: Startup / Median buffer fill ──
    if (f->sample_count < ACCEL_FILTER_SETTLE_COUNT) {
        f->median_buf[f->sample_count] = raw;
        f->sample_count++;

        if (f->sample_count < ACCEL_FILTER_MEDIAN_SIZE) {
            return raw;
        }
    } else {
        // Shift median buffer (FIFO)
        f->median_buf[0] = f->median_buf[1];
        f->median_buf[1] = f->median_buf[2];
        f->median_buf[2] = raw;
        // Keep counting for gravity settle (cap to prevent overflow)
        if (f->sample_count < 65535) f->sample_count++;
    }

    // ── Stage 2: Median filter — removes spike noise ──
    float median = get_median_of_3(f->median_buf[0], f->median_buf[1], f->median_buf[2]);

    // ── Stage 3: First stable sample init ──
    if (!f->initialized) {
        f->ema_value = median;
        f->last_output = median;
        f->gravity_estimate = median;   // Init gravity estimate
        f->initialized = true;
        f->ac_component = 0.0f;
        return median;
    }

    // ── Stage 4: Rate limiter — cap max change per sample ──
    float diff = median - f->last_output;
    if (diff > ACCEL_FILTER_RATE_LIMIT) {
        median = f->last_output + ACCEL_FILTER_RATE_LIMIT;
    } else if (diff < -ACCEL_FILTER_RATE_LIMIT) {
        median = f->last_output - ACCEL_FILTER_RATE_LIMIT;
    }

    // ── Stage 5: EMA smoothing (fast) ──
    f->ema_value = (ACCEL_FILTER_EMA_ALPHA * median) +
                   ((1.0f - ACCEL_FILTER_EMA_ALPHA) * f->ema_value);

    f->last_output = f->ema_value;

    // ── Stage 6: Gravity removal (IIR high-pass) ──
    // Gravity estimate: very slow EMA (α=0.02) → tracks only DC component
    // When still: gravity_estimate ≈ magnitude ≈ 9.6-9.8
    // When moving: gravity_estimate stays near baseline, AC shows deviation
    f->gravity_estimate = (ACCEL_GRAVITY_ALPHA * f->ema_value) +
                          ((1.0f - ACCEL_GRAVITY_ALPHA) * f->gravity_estimate);

    // AC component = |filtered - gravity| = movement-only signal
    f->ac_component = fabsf(f->ema_value - f->gravity_estimate);

    // Mark gravity as settled after ~50 samples (1 second at 50Hz)
    if (f->sample_count >= 50) {
        f->gravity_settled = true;
    }

    return f->ema_value;
}

float accel_filter_get(const accel_filter_t *f) {
    if (!f || !f->initialized) return 0.0f;
    return f->ema_value;
}

float accel_filter_get_ac(const accel_filter_t *f) {
    if (!f || !f->initialized) return 0.0f;
    return f->ac_component;
}

bool accel_filter_is_stable(const accel_filter_t *f) {
    if (!f) return false;
    return f->initialized;
}
