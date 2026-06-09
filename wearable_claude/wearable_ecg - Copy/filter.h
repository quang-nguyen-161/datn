#ifndef __FILTER_H__
#define __FILTER_H__
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* DC removal filter (moving-average baseline subtraction) */
/*
#define DC_WINDOW   125     /* ~0.5 s at 250 Hz 

typedef struct {
    float    buf[DC_WINDOW];
    float    sum;
    float    dc_comp;
    uint16_t idx;
} dc_filter_t;
*/
typedef struct
{	
	
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;
} iir2_typedef;

typedef struct 
{
	float b0;
	float b1;
	
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

#define NLMS_TAPS 16	   // filter length (tune this)

typedef struct
{
    float w[NLMS_TAPS];   // adaptive weights
    float x[NLMS_TAPS];   // input buffer (ring buffer)
    uint16_t idx;         // head index
    float mu;             // step size (0 < mu < 1)
    float eps;            // small value to avoid divide-by-zero
} nlms_t;

#define ALE_DELAY 5   // tune this (5~20 typical for ECG)

typedef struct {
    float buf[ALE_DELAY];
    uint16_t idx;
} delay_t;


float bu_filter_1st_5Hz(float x, float x1, float y1);
float bu_filter_1st_10Hz(float x, float x1, float y1);
float bu_filter_2nd_5Hz(float x, float x1, float x2, float y1, float y2);
float bu_filter_2nd_10Hz(float x, float x1, float x2, float y1, float y2);

float iir1_filter(iir1_typedef *iir1, float x, float x1, float y1);
float iir2_filter(iir2_typedef *iir2, float x, float x1, float x2, float y1, float y2);

float iir1_filter_rb(iir1_typedef *iir1, iir1_state *s, float x);
float iir2_filter_rb(iir2_typedef *iir2, iir2_state *s, float x);
void nlms_init(nlms_t *f, float mu, float eps);
float nlms_step(nlms_t *f, float x_new, float d);
void delay_init(delay_t *d);
float delay_process(delay_t *d, float x);
float ale_process(nlms_t *nlms, delay_t *delay, float x);

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
void biquad_init(biquad_df2t_t *f,
                 float b0, float b1, float b2,
                 float a1, float a2);
float biquad_step(biquad_df2t_t *f, float x);
void iir1_init(iir1_df2t_t *f, float b0, float b1, float a1);
float iir1_step(iir1_df2t_t *f, float x);

/* Savitzky-Golay smoothing filter (window=11, polynomial order=3) */
#define SG_WINDOW   11

typedef struct {
    float   buf[SG_WINDOW];
    uint8_t head;
} sg_filter_t;

void  sg_init(sg_filter_t *f);
float sg_step(sg_filter_t *f, float x);

#endif