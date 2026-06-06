#ifndef _TEMP_FILTER_H_
#define _TEMP_FILTER_H_

#include <stdint.h>
#include <stdbool.h>

#define TEMP_FILTER_EMA_ALPHA     0.3f
#define TEMP_FILTER_MEDIAN_SIZE   3
#define TEMP_FILTER_RATE_LIMIT    0.5f
#define TEMP_FILTER_SETTLE_COUNT  3

typedef struct {
    float    median_buf[TEMP_FILTER_MEDIAN_SIZE];
    uint8_t  median_idx;
    float    ema_value;
    float    last_output;
    uint8_t  sample_count;
    bool     initialized;
} temp_filter_t;

void  temp_filter_reset(temp_filter_t *f);
float temp_filter_update(temp_filter_t *f, float raw_temp);
float temp_filter_get(const temp_filter_t *f);
bool  temp_filter_is_stable(const temp_filter_t *f);

#endif /* _TEMP_FILTER_H_ */
