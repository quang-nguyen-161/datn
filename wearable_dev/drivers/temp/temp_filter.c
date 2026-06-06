#include "temp_filter.h"
#include <string.h>

static float get_median_of_3(float a, float b, float c)
{
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

void temp_filter_reset(temp_filter_t *f)
{
    if (!f) return;
    memset(f, 0, sizeof(temp_filter_t));
}

float temp_filter_update(temp_filter_t *f, float raw_temp)
{
    if (!f) return raw_temp;

    if (f->sample_count < TEMP_FILTER_SETTLE_COUNT) {
        f->median_buf[f->sample_count] = raw_temp;
        f->sample_count++;

        if (f->sample_count < TEMP_FILTER_MEDIAN_SIZE) {
            return raw_temp;
        }
    } else {
        f->median_buf[0] = f->median_buf[1];
        f->median_buf[1] = f->median_buf[2];
        f->median_buf[2] = raw_temp;
    }

    float median = get_median_of_3(f->median_buf[0], f->median_buf[1], f->median_buf[2]);

    if (!f->initialized) {
        f->ema_value   = median;
        f->last_output = median;
        f->initialized = true;
        return median;
    }

    float diff = median - f->last_output;
    if (diff > TEMP_FILTER_RATE_LIMIT) {
        median = f->last_output + TEMP_FILTER_RATE_LIMIT;
    } else if (diff < -TEMP_FILTER_RATE_LIMIT) {
        median = f->last_output - TEMP_FILTER_RATE_LIMIT;
    }

    f->ema_value   = (TEMP_FILTER_EMA_ALPHA * median) + ((1.0f - TEMP_FILTER_EMA_ALPHA) * f->ema_value);
    f->last_output = f->ema_value;

    return f->ema_value;
}

float temp_filter_get(const temp_filter_t *f)
{
    if (!f || !f->initialized) return 0.0f;
    return f->ema_value;
}

bool temp_filter_is_stable(const temp_filter_t *f)
{
    if (!f) return false;
    return f->initialized;
}
