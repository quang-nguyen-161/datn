#ifndef ACCEL_FILTER_H
#define ACCEL_FILTER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Accelerometer Filter — 4-stage pipeline
 *
 * Stage 1: Median(3) — spike removal
 * Stage 2: EMA(α=0.5) — noise smoothing
 * Stage 3: Rate Limiter(±2 m/s²) — anti-glitch
 * Stage 4: Gravity Removal (IIR high-pass) — tách DC component (gravity)
 *
 * Output:
 *   - filtered_magnitude: Accel đã smooth (hiển thị)
 *   - ac_component: Chỉ phần dao động do chuyển động (cho pedometer)
 */

#define ACCEL_FILTER_MEDIAN_SIZE    3
#define ACCEL_FILTER_SETTLE_COUNT   3       // Samples before filter is stable
#define ACCEL_FILTER_EMA_ALPHA      0.5f    // Higher α = faster response, preserves step peaks
#define ACCEL_FILTER_RATE_LIMIT     2.0f    // Max ±2 m/s² per sample (anti-glitch)
#define ACCEL_GRAVITY_ALPHA         0.02f   // Very slow EMA for gravity estimate (~50 samples to settle)

typedef struct {
    /* Median + EMA + Rate limiter */
    float    median_buf[ACCEL_FILTER_MEDIAN_SIZE];
    float    ema_value;
    float    last_output;
    uint16_t sample_count;
    bool     initialized;

    /* Gravity removal (high-pass filter) */
    float    gravity_estimate;      // Slow-moving average ≈ gravity
    float    ac_component;          // magnitude - gravity = movement only
    bool     gravity_settled;       // True after enough samples
} accel_filter_t;

/**
 * Reset filter state
 */
void accel_filter_reset(accel_filter_t *f);

/**
 * Feed new raw accel magnitude, returns filtered magnitude
 */
float accel_filter_update(accel_filter_t *f, float raw);

/**
 * Get last filtered magnitude (for display)
 */
float accel_filter_get(const accel_filter_t *f);

/**
 * Get AC component — movement-only signal with gravity removed (for pedometer)
 * Always ≥ 0 when moving, ≈ 0 when still
 */
float accel_filter_get_ac(const accel_filter_t *f);

/**
 * Check if filter has settled
 */
bool accel_filter_is_stable(const accel_filter_t *f);

#endif // ACCEL_FILTER_H
