#include "filter.h"

//coefficients

// fs = 100, fc = 10
// 1st order
const float b0_bu1_10=0.24523727525278557; 
const float b1_bu1_10=0.24523727525278557; 
const float a1_bu1_10=-0.5095254494944289;

// 2nd order
const float b0_bu2_10=0.09549150281252626; 
const float b1_bu2_10=0.19098300562505252; 
const float b2_bu2_10=0.09549150281252626; 
const float a1_bu2_10=-1.2024070509724416; 
const float a2_bu2_10=0.5843730622225466;

// fs = 100, fc = 5
// 1st order
const float b0_bu1_5=0.13672873599731955;
const float b1_bu1_5=0.13672873599731955;
const float a1_bu1_5=-0.726542528005361;

// 2nd order
const float b0_bu2_5=0.02447174185242321;
const float b1_bu2_5=0.04894348370484642;
const float b2_bu2_5=0.02447174185242321;
const float a1_bu2_5=-1.6836050203658963;
const float a2_bu2_5=0.7814919877755896;

float bu_filter_1st_5Hz(float x, float x1, float y1)
{
    float y = b0_bu1_5*x + b1_bu1_5*x1 - a1_bu1_5*y1;
    return y;
}

float bu_filter_2nd_5Hz(float x, float x1, float x2, float y1, float y2)
{
		float y = b0_bu2_5*x + b1_bu2_5*x1 + b2_bu2_5*x2 - a1_bu2_5*y1 - a2_bu2_5*y2;
		return y;
}

float bu_filter_1st_10Hz(float x, float x1, float y1)
{
    float y = b0_bu1_10*x + b1_bu1_10*x1 - a1_bu1_10*y1;
    return y;
}

float bu_filter_2nd_10Hz(float x, float x1, float x2, float y1, float y2)
{
		float y = b0_bu2_10*x + b1_bu2_10*x1 + b2_bu2_10*x2 - a1_bu2_10*y1 - a2_bu2_10*y2;
		return y;
}

iir1_typedef FilterBuLp1_5_100Hz = 
{
    .b0 = 0.13672873599731955f,
    .b1 = 0.13672873599731955f,
    .a1 = -0.726542528005361f,

};

iir1_typedef FilterBuHp1_0p5_100Hz = 
{
    .b0 = 0.9845337085968967f,
    .b1 = -0.9845337085968967f,
    .a1 = 0.9690674171937933f,

};

iir1_typedef FilterBuLp1_25_200Hz = 
{
    .b0 = 0.2928932188134524f,
    .b1 = 0.2928932188134524f,
    .a1 = -0.41421356237309503f,

};

iir2_typedef FilterBuLp2_5_100Hz = 
{
    .b0 = 0.02447174185242321f,
    .b1 = 0.04894348370484642f,
    .b2 = 0.02447174185242321f,
    .a1 = -1.6836050203658963f,
    .a2 = 0.7814919877755896f,
};

iir1_typedef FilterBuHp1_0p5_200Hz = 
{
    .b0 = 0.9922070637080485f,
    .b1 = -0.9922070637080485f,
    .a1 = 0.984414127416097f,
};

iir2_typedef FilterBuLp2_25_200Hz = 
{
    .b0 = 0.1464466094067262f,
    .b1 = 0.2928932188134524f,
    .b2 = 0.1464466094067262f,
    .a1 = -0.9142135623730949f,
    .a2 = 0.49999999999999994f,
};

iir2_typedef FilterBuHp2_0p5_200Hz = 
{
    .b0 = 0.9999383162408303f,
    .b1 = -1.9998766324816606f,
    .b2 = 0.9999383162408303f,
    .a1 = -1.9886465143778842f,
    .a2 = 0.9888932494145627f,
};

iir2_typedef FilterNotch_50Hz_200Hz = 
{
    .b0 = 0.9270403427317333f,
    .b1 = 0.0f,
    .b2 = 0.9270403427317333f,
    .a1 = 0.0f,
    .a2 = 0.8540806854634666f,
};

float iir1_filter(iir1_typedef *iir1, float x, float x1, float y1)
{
	float y = iir1->b0*x + iir1->b1*x1 - iir1->a1*y1;
    return y;
}

float iir2_filter(iir2_typedef *iir2, float x, float x1, float x2, float y1, float y2)
{
	float y = iir2->b0*x + iir2->b1*x1 + iir2->b2*x2 - iir2->a1*y1 - iir2->a2*y2;
	return y;
}





#define RB_IDX(head, delay) (((head) + BUF_SIZE - (delay) - 1) % BUF_SIZE)

float iir2_filter_rb(iir2_typedef *iir2, iir2_state *s, float x)
{
    uint8_t h = s->head;

    // store current input
    s->x[h] = x;

    // get required samples
    float x0 = s->x[RB_IDX(h, 0)];
    float x1 = s->x[RB_IDX(h, 1)];
    float x2 = s->x[RB_IDX(h, 2)];

    float y1 = s->y[RB_IDX(h, 1)];
    float y2 = s->y[RB_IDX(h, 2)];

    // compute output
    float y = iir2->b0*x0 + iir2->b1*x1 + iir2->b2*x2
              - iir2->a1*y1 - iir2->a2*y2;

    // store output
    s->y[h] = y;

    // advance head
    s->head = (h + 1) % BUF_SIZE;

    return y;
}



#define RB1_IDX(head, delay) (((head) + BUF1_SIZE - (delay) - 1) % BUF1_SIZE)

float iir1_filter_rb(iir1_typedef *iir1, iir1_state *s, float x)
{
    uint8_t h = s->head;

    // store current input
    s->x[h] = x;

    // get samples
    float x0 = s->x[RB1_IDX(h, 0)];
    float x1 = s->x[RB1_IDX(h, 1)];

    float y1 = s->y[RB1_IDX(h, 1)];

    // compute output
    float y = iir1->b0*x0 + iir1->b1*x1
              - iir1->a1*y1;

    // store output
    s->y[h] = y;

    // advance head
    s->head = (h + 1) % BUF1_SIZE;

    return y;
}



void nlms_init(nlms_t *f, float mu, float eps)
{
    memset(f->w, 0, sizeof(f->w));
    memset(f->x, 0, sizeof(f->x));
    f->idx = 0;
    f->mu = mu;
    f->eps = eps;
}
float nlms_step(nlms_t *f, float x_new, float d)
{
    f->x[f->idx] = x_new;

    float y = 0.0f;
    float norm = 0.0f;

    for (int i = 0; i < NLMS_TAPS; i++)
    {
        int index = (f->idx + NLMS_TAPS - i) % NLMS_TAPS;
        float xi = f->x[index];

        y += f->w[i] * xi;
        norm += xi * xi;
    }

    float e = d - y;
    float g = f->mu / (norm + f->eps);

    for (int i = 0; i < NLMS_TAPS; i++)
    {
        int index = (f->idx + NLMS_TAPS - i) % NLMS_TAPS;
        f->w[i] += g * e * f->x[index];
    }

    f->idx = (f->idx + 1) % NLMS_TAPS;

    return y;   
}



void delay_init(delay_t *d)
{
    memset(d->buf, 0, sizeof(d->buf));
    d->idx = 0;
}

float delay_process(delay_t *d, float x)
{
    float y = d->buf[d->idx];   // delayed output

    d->buf[d->idx] = x;         // store new sample
    d->idx = (d->idx + 1) % ALE_DELAY;

    return y;
}

float ale_process(nlms_t *nlms, delay_t *delay, float x)
{
    // get delayed version
    float x_delayed = delay_process(delay, x);

    // NLMS output (estimated signal component)
    float e = nlms_step(nlms, x_delayed, x);

    // subtract to remove noise
    float clean =  e;

    return clean;
}



void biquad_init(biquad_df2t_t *f,
                 float b0, float b1, float b2,
                 float a1, float a2)
{
    f->b0 = b0;
    f->b1 = b1;
    f->b2 = b2;
    f->a1 = a1;
    f->a2 = a2;
    f->s1 = 0.0f;
    f->s2 = 0.0f;
}

float biquad_step(biquad_df2t_t *f, float x)
{
    float y = f->b0 * x + f->s1;
    f->s1 = f->b1 * x - f->a1 * y + f->s2;
    f->s2 = f->b2 * x - f->a2 * y;
    return y;
}



void iir1_init(iir1_df2t_t *f, float b0, float b1, float a1)
{
    f->b0 = b0;
    f->b1 = b1;
    f->a1 = a1;
    f->s1 = 0.0f;
}

 float iir1_step(iir1_df2t_t *f, float x)
{
    float y = f->b0 * x + f->s1;
    f->s1 = f->b1 * x - f->a1 * y;
    return y;
}


/* ================================================================== */
/*  Savitzky-Golay FIR (window=11, poly order=3, norm=429)            */
/*  Coefficients: least-squares polynomial fit, centre index 5        */
/*  Output has a 5-sample group delay at 250 Hz = 20 ms               */
/* ================================================================== */
static const float sg_coeffs[SG_WINDOW] = {
    -36.0f,  9.0f, 44.0f, 69.0f, 84.0f,
     89.0f, 84.0f, 69.0f, 44.0f,  9.0f, -36.0f
};
#define SG_NORM 429.0f

void sg_init(sg_filter_t *f)
{
    memset(f->buf, 0, sizeof(f->buf));
    f->head = 0;
}

float sg_step(sg_filter_t *f, float x)
{
    f->buf[f->head] = x;
    f->head = (f->head + 1) % SG_WINDOW;

    /* After advancing, head points to the oldest sample.
       Walk forward so c[0] multiplies oldest, c[10] multiplies newest. */
    float acc = 0.0f;
    for (uint8_t i = 0; i < SG_WINDOW; i++) {
        acc += sg_coeffs[i] * f->buf[(f->head + i) % SG_WINDOW];
    }
    return acc / SG_NORM;
}

