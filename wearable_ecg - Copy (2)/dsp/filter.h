#ifndef __FILTER_H__
#define __FILTER_H__
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ── Legacy IIR structures (ring-buffer API) ── */

typedef struct {
    float b0, b1, b2;
    float a1, a2;
} iir2_typedef;

typedef struct {
    float b0, b1;
    float a1;
} iir1_typedef;

extern iir1_typedef FilterBuLp1_5_100Hz;
extern iir1_typedef FilterBuHp1_0p5_100Hz;
extern iir1_typedef FilterBuLp1_25_200Hz;
extern iir1_typedef FilterBuHp1_0p5_200Hz;
extern iir2_typedef FilterBuLp2_25_200Hz;
extern iir2_typedef FilterBuHp2_0p5_200Hz;
extern iir2_typedef FilterNotch_50Hz_200Hz;
extern iir2_typedef FilterBuLp2_5_100Hz;

#define BUF_SIZE 8
typedef struct {
    float x[BUF_SIZE];
    float y[BUF_SIZE];
    uint8_t head;
} iir2_state;

#define BUF1_SIZE 2
typedef struct {
    float x[BUF1_SIZE];
    float y[BUF1_SIZE];
    uint8_t head;
} iir1_state;

/* ── DC-removal sliding-mean filter ── */
#define DC_WINDOW 125   /* ~0.5 s at 250 Hz */
typedef struct {
    float buf[DC_WINDOW];
    float dc_comp;
    int   idx;
    float sum;
} dc_filter_t;

/* ── NLMS adaptive filter ── */
#define NLMS_TAPS 32
typedef struct {
    float    w[NLMS_TAPS];
    float    x[NLMS_TAPS];
    uint16_t idx;
    float    mu;
    float    eps;
} nlms_t;

#define ALE_DELAY 15
typedef struct {
    float    buf[ALE_DELAY];
    uint16_t idx;
} delay_t;

/* ── Direct-Form II transposed biquad (primary DSP path) ── */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float s1, s2;
} biquad_df2t_t;

typedef struct {
    float b0, b1;
    float a1;
    float s1;
} iir1_df2t_t;

/* ── Function declarations ── */

float bu_filter_1st_5Hz(float x, float x1, float y1);
float bu_filter_1st_10Hz(float x, float x1, float y1);
float bu_filter_2nd_5Hz(float x, float x1, float x2, float y1, float y2);
float bu_filter_2nd_10Hz(float x, float x1, float x2, float y1, float y2);

float iir1_filter(iir1_typedef *iir1, float x, float x1, float y1);
float iir2_filter(iir2_typedef *iir2, float x, float x1, float x2, float y1, float y2);
float iir1_filter_rb(iir1_typedef *iir1, iir1_state *s, float x);
float iir2_filter_rb(iir2_typedef *iir2, iir2_state *s, float x);

void  nlms_init(nlms_t *f, float mu, float eps);
float nlms_step(nlms_t *f, float x_new, float d);
void  delay_init(delay_t *d);
float delay_process(delay_t *d, float x);
float ale_process(nlms_t *nlms, delay_t *delay, float x);

void  biquad_init(biquad_df2t_t *f,
                  float b0, float b1, float b2,
                  float a1, float a2);
float biquad_step(biquad_df2t_t *f, float x);
void  iir1_init(iir1_df2t_t *f, float b0, float b1, float a1);
float iir1_step(iir1_df2t_t *f, float x);

#endif /* __FILTER_H__ */
