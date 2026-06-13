/**
 * Dashboard — GC9A01 display module for SF7 Slave (Layout V2)
 *
 * Layout V2 changes from V1:
 *   Row 1: BLE Status Bar (device name/MAC + RSSI + signal icon)
 *   Row 2: Temp (left) + SpO2 (right) — unchanged
 *   Row 3: ECG (left 55%) + HR number & sweep (right 45%)
 *   Row 4: Steps + Activity — unchanged
 */

#include "dashboard.h"
#include "gc9a01.h"
#include "nrf_delay.h"

#include <stdio.h>
#include <string.h>

/* ================================================================
 *  COLOR DEFINES
 * ================================================================ */

#define MED_GRAY    RGB565(60, 60, 60)
#define DARK_GRAY   RGB565(40, 40, 40)
#define LIGHT_GRAY  RGB565(130, 130, 130)
#define DARK_RED    RGB565(100, 0, 0)
#define SOFT_RED    RGB565(255, 60, 60)
#define SOFT_BLUE   RGB565(80, 140, 255)
#define SOFT_GREEN  RGB565(60, 220, 100)
#define ORANGE      RGB565(255, 165, 0)
#define DARK_BG     RGB565(15, 15, 20)
#define CX  120
#define CY  120

/* ================================================================
 *  ACTIVITY DETECTION (internal)
 * ================================================================ */

typedef enum {
    ACT_REST, ACT_MOVING, ACT_WALK, ACT_FAST_WALK, ACT_RUN
} activity_t;

static activity_t s_activity = ACT_REST;
static uint32_t s_prev_steps = 0;

static const char* act_name(activity_t a) {
    switch (a) {
        case ACT_REST:      return "REST";
        case ACT_MOVING:    return "MOVING";
        case ACT_WALK:      return "WALK";
        case ACT_FAST_WALK: return "F.WALK";
        case ACT_RUN:       return "RUN";
        default:            return "?";
    }
}

static activity_t detect_activity(float ac, float cad, uint32_t steps, uint32_t prev)
{
    bool stepping = (steps > prev);
    if (ac < 0.05f && !stepping) return ACT_REST;
    if (!stepping)               return ACT_MOVING;
    if (cad > 0 && cad < 120)   return ACT_WALK;
    if (cad >= 120 && cad < 150) return ACT_FAST_WALK;
    if (cad >= 150)              return ACT_RUN;
    return ACT_WALK;
}

/* ================================================================
 *  ECG SYNTHETIC
 * ================================================================ */

static int32_t ecg_synthetic(uint8_t hr, uint32_t tick_ms)
{
    uint32_t period_ms = (hr > 0) ? (60000 / hr) : 800;
    uint32_t phase_ms = tick_ms % period_ms;
    float phase = (float)phase_ms / (float)period_ms;
    int32_t val = 500;

    if (phase < 0.05f) val = 500;
    else if (phase < 0.12f) {
        float p = (phase - 0.05f) / 0.07f;
        val = 500 + (int32_t)(4.0f * p * (1.0f - p) * 80);
    }
    else if (phase < 0.18f) val = 500;
    else if (phase < 0.20f) { val = 500 - (int32_t)(((phase-0.18f)/0.02f) * 60); }
    else if (phase < 0.25f) {
        float r = (phase - 0.20f) / 0.05f;
        if (r < 0.5f) val = 440 + (int32_t)(r * 2.0f * 510);
        else val = 950 - (int32_t)((r - 0.5f) * 2.0f * 650);
    }
    else if (phase < 0.28f) { val = 300 + (int32_t)(((phase-0.25f)/0.03f) * 200); }
    else if (phase < 0.35f) val = 500;
    else if (phase < 0.50f) {
        float t = (phase - 0.35f) / 0.15f;
        val = 500 + (int32_t)(4.0f * t * (1.0f - t) * 120);
    }
    else val = 500;
    if (val < 0) val = 0; if (val > 1000) val = 1000;
    return val;
}

/* ================================================================
 *  DRAWING HELPERS
 * ================================================================ */

static void draw_heart(int16_t x, int16_t y, int16_t size, uint16_t color)
{
    int16_t r = size / 2;
    GC9A01_fill_circle(x - r + 1, y - r / 2, r, color);
    GC9A01_fill_circle(x + r - 1, y - r / 2, r, color);
    for (int16_t row = 0; row <= size + r / 2; row++) {
        int16_t hw = size - (row * size) / (size + r / 2);
        if (hw > 0) GC9A01_draw_line(color, x - hw, y + row, x + hw, y + row);
    }
}

static void draw_pulse_line(int16_t x1, int16_t x2, int16_t y, uint16_t color)
{
    int16_t s = (x2 - x1) / 8;
    GC9A01_draw_line(color, x1, y, x1+s*2, y);
    GC9A01_draw_line(color, x1+s*2, y, x1+s*3, y-12);
    GC9A01_draw_line(color, x1+s*3, y-12, x1+s*4, y+15);
    GC9A01_draw_line(color, x1+s*4, y+15, x1+s*5, y-8);
    GC9A01_draw_line(color, x1+s*5, y-8, x1+s*6, y);
    GC9A01_draw_line(color, x1+s*6, y, x2, y);
}

static void draw_droplet(int16_t x, int16_t y, int16_t size, uint16_t color)
{
    GC9A01_fill_circle(x, y + size/3, size/2, color);
    for (int16_t row = 0; row <= size; row++) {
        int16_t hw = (row * size / 2) / size;
        if (hw > 0) GC9A01_draw_line(color, x-hw, y-size/2+row, x+hw, y-size/2+row);
    }
}

static void draw_step_icon(int16_t x, int16_t y, uint16_t color)
{
    GC9A01_fill_circle(x, y+2, 5, color);
    GC9A01_fill_rect(x-4, y-4, 8, 6, color);
    GC9A01_fill_circle(x-4, y-8, 2, color);
    GC9A01_fill_circle(x-1, y-9, 2, color);
    GC9A01_fill_circle(x+2, y-9, 2, color);
    GC9A01_fill_circle(x+5, y-8, 2, color);
}

static void draw_thermometer(int16_t x, int16_t y, uint16_t color)
{
    GC9A01_fill_circle(x, y+4, 4, color);
    GC9A01_fill_rect(x-1, y-6, 3, 10, color);
    GC9A01_draw_circle(x, y+4, 5, WHITE);
    GC9A01_draw_line(WHITE, x-2, y-8, x+2, y-8);
    GC9A01_draw_line(WHITE, x-2, y-8, x-2, y+4);
    GC9A01_draw_line(WHITE, x+2, y-8, x+2, y+4);
}

/**
 * Draw signal strength bars (antenna icon)
 * bars: 0-4 (0=no signal, 4=excellent)
 */
static void draw_signal_bars(int16_t x, int16_t y, uint8_t bars, uint16_t color)
{
    uint16_t off_color = RGB565(40, 40, 40);
    /* 4 bars, increasing height: 4, 8, 12, 16 px */
    for (uint8_t i = 0; i < 4; i++) {
        int16_t bh = 5 + i * 4;     /* heights: 5, 9, 13, 17 */
        int16_t bx = x + i * 6;     /* spacing: 6px apart */
        int16_t by = y + 17 - bh;   /* align bottoms */
        uint16_t c = (i < bars) ? color : off_color;
        GC9A01_fill_rect(bx, by, 4, bh, c);
    }
}

/**
 * Draw battery icon — Row 1, right of signal bars.
 * Outline + nub always drawn; interior fill only when battery_valid.
 */
static void draw_battery_icon(const dashboard_data_t *d)
{
    GC9A01_fill_rect(151, 14, 16, 9, DARK_BG);  /* clear icon + nub area */
    GC9A01_draw_line(LIGHT_GRAY, 151, 14, 165, 14);
    GC9A01_draw_line(LIGHT_GRAY, 151, 22, 165, 22);
    GC9A01_draw_line(LIGHT_GRAY, 151, 14, 151, 22);
    GC9A01_draw_line(LIGHT_GRAY, 165, 14, 165, 22);
    GC9A01_fill_rect(165, 16, 2, 4, LIGHT_GRAY);
    if (d->battery_valid) {
        uint8_t pct = d->battery_pct > 100 ? 100 : d->battery_pct;
        uint16_t fc = (pct < 20) ? SOFT_RED : (pct < 50) ? YELLOW : SOFT_GREEN;
        int16_t fw = (int16_t)((pct * 13) / 100);
        if (fw > 0) GC9A01_fill_rect(152, 15, fw, 7, fc);
    }
    /* else: interior stays cleared → outline-only placeholder */
}

/* ================================================================
 *  COLOR LOGIC
 * ================================================================ */

static void get_hr_colors(uint8_t v, uint16_t *lc, uint16_t *fc, uint16_t *tc) {
    if (v<51||v>130) {*lc=SOFT_RED;*fc=DARK_RED;*tc=RED;}
    else if (v>=100) {*lc=YELLOW;*fc=RGB565(100,100,0);*tc=YELLOW;}
    else {*lc=SOFT_GREEN;*fc=RGB565(0,100,0);*tc=GREEN;}
}

static void get_temp_colors(uint16_t t, uint16_t *lc, uint16_t *fc, uint16_t *tc) {
    if (t<355||t>=385) {*lc=SOFT_RED;*fc=DARK_RED;*tc=RED;}
    else if (t>=376)   {*lc=YELLOW;*fc=RGB565(100,100,0);*tc=YELLOW;}
    else               {*lc=CYAN;*fc=RGB565(0,50,100);*tc=CYAN;}
}

static void get_spo2_colors(uint8_t v, uint16_t *bc, uint16_t *tc) {
    if (v<90)       {*bc=RED;*tc=RED;}
    else if (v<=94) {*bc=YELLOW;*tc=YELLOW;}
    else            {*bc=SOFT_GREEN;*tc=GREEN;}
}

/* ================================================================
 *  SWEEP CHARTS
 * ================================================================ */

static void sweep_area_chart(int16_t x0, int16_t y0, int16_t w, int16_t h,
                             int16_t *dx, int16_t *py,
                             int32_t val, int32_t vmin, int32_t vmax,
                             uint16_t lc, uint16_t fc)
{
    if (val<vmin) val=vmin; if (val>vmax) val=vmax;
    int16_t cy = y0 - (int16_t)(((float)(val-vmin)/(vmax-vmin))*h);
    int16_t cx = x0 + *dx;
    int16_t clr_x = cx+1, clr_w = 4;
    if (clr_x < x0+w) {
        int16_t cw=clr_w; if(clr_x+cw>x0+w) cw=(x0+w)-clr_x;
        GC9A01_fill_rect(clr_x, y0-h, cw, h, DARK_BG);
    }
    if (cx+1+clr_w > x0+w) {
        int16_t cw=(cx+1+clr_w)-(x0+w);
        GC9A01_fill_rect(x0, y0-h, cw, h, DARK_BG);
    }
    GC9A01_draw_line(fc, cx, cy, cx, y0);
    GC9A01_draw_pixel(cx, cy, lc);
    GC9A01_draw_pixel(cx, cy-1, lc);
    if (*dx>0) {
        if (cy<*py) GC9A01_draw_line(lc, cx-1, cy, cx-1, *py);
        else if (cy>*py) GC9A01_draw_line(lc, cx, *py, cx, cy);
    }
    *py=cy; (*dx)++; if(*dx>=w) *dx=0;
}

static void sweep_line(int16_t x0, int16_t y0, int16_t w, int16_t h,
                       int16_t *dx, int16_t *py,
                       int32_t val, int32_t vmin, int32_t vmax,
                       uint16_t color)
{
    if (val<vmin) val=vmin; if (val>vmax) val=vmax;
    int16_t cy = y0 - (int16_t)(((float)(val-vmin)/(vmax-vmin))*h);
    int16_t cx = x0 + *dx;
    int16_t clr_w = 6, clr_x = cx + 1;
    if (clr_x < x0+w) {
        int16_t cw=clr_w; if(clr_x+cw>x0+w) cw=(x0+w)-clr_x;
        GC9A01_fill_rect(clr_x, y0-h, cw, h, DARK_BG);
    }
    if (cx+1+clr_w > x0+w) {
        int16_t cw=(cx+1+clr_w)-(x0+w);
        GC9A01_fill_rect(x0, y0-h, cw, h, DARK_BG);
    }
    GC9A01_draw_pixel(cx, cy, color);
    if (*dx > 0) GC9A01_draw_line(color, cx-1, *py, cx, cy);
    *py = cy; (*dx)++; if (*dx >= w) *dx = 0;
}

static void sweep_bar_chart(int16_t x0, int16_t y0, int16_t w, int16_t h,
                            int16_t *idx, uint16_t nb,
                            uint8_t val, uint8_t vmin, uint8_t vmax)
{
    int16_t btw=w/nb, bw=btw-2; if(bw<1) bw=1;
    if(val<vmin) val=vmin; if(val>vmax) val=vmax;
    int16_t bx=x0+(*idx)*btw+1;
    int16_t bh=(int16_t)(((float)(val-vmin)/(vmax-vmin))*h); if(bh<2) bh=2;
    uint16_t bc,dc; get_spo2_colors(val,&bc,&dc);
    GC9A01_fill_rect(bx, y0-h, bw, h, DARK_BG);
    GC9A01_fill_rect(bx, y0-bh, bw, bh, bc);
    int16_t ni=(*idx+1)%nb, nbx=x0+ni*btw+1;
    GC9A01_fill_rect(nbx, y0-h, bw, h, DARK_BG);
    (*idx)++; if(*idx>=nb) *idx=0;
}

/* ================================================================
 *  CHART STATE (persistent across calls)
 * ================================================================ */

/* V2: HR sweep moved to Row 3 right (smaller) */
static int16_t hr_dx=0, hr_py=170;
static int16_t tmp_dx=0, tmp_py=110;
static int16_t spo2_idx=0;
/* V2: ECG narrower (120px) */
static int16_t ecg_dx=0, ecg_py=150;
static uint8_t  last_hr=255;
static uint8_t  last_spo2=255;
static uint16_t last_temp=0xFFFF;
static uint32_t last_steps_disp=0xFFFFFFFF;

/* V2: BLE status cache */
static int8_t   last_rssi=1;          /* Invalid initial → forces first draw */
static bool     last_ble_conn=false;
static char     last_device_name[16] = "";

/* V3: ECG ON/OFF badge + battery icon cache */
static bool     last_ecg_enabled=false;
static bool     last_battery_valid=false;
static uint8_t  last_battery_pct=0xFF;

/* ================================================================
 *  PUBLIC API
 * ================================================================ */

void dashboard_splash(void)
{
    GC9A01_fill_rect(0, 0, 239, 239, DARK_BG);
    GC9A01_draw_circle(CX, CY, 117, DARK_GRAY);
    draw_heart(CX, 52, 12, SOFT_RED);
    GC9A01_set_back_color(DARK_BG);
    GC9A01_set_font(&Font24);
    GC9A01_set_text_color(CYAN);
    GC9A01_draw_string(35, 80, "SensorLab");
    draw_pulse_line(40, 200, 110, SOFT_RED);
    GC9A01_set_font(&Font16);
    GC9A01_set_text_color(WHITE);
    GC9A01_draw_string(30, 125, "Smart Ring for");
    GC9A01_set_text_color(SOFT_GREEN);
    GC9A01_draw_string(72, 148, "Health");
    draw_heart(70, 185, 6, SOFT_RED);
    draw_droplet(120, 185, 10, SOFT_BLUE);
    draw_step_icon(170, 188, SOFT_GREEN);
    GC9A01_set_font(&Font8);
    GC9A01_set_text_color(LIGHT_GRAY);
    GC9A01_draw_string(72, 210, "SF7 BLE v2.0");
}

void dashboard_show_update_splash(const char *title, const char *val)
{
    GC9A01_fill_rect(0, 0, 239, 239, DARK_BG);
    GC9A01_draw_circle(CX, CY, 117, DARK_GRAY);
    GC9A01_set_back_color(DARK_BG);

    /* Bold = draw the centered string twice, 1px apart */
    GC9A01_set_font(&Font16);
    GC9A01_set_text_color(CYAN);
    int16_t w = (int16_t)(strlen(title) * 11);
    int16_t x = (240 - w) / 2;
    if (x < 3) x = 3;
    int16_t y = CY - 20;
    GC9A01_draw_string(x,     y, (char *)title);
    GC9A01_draw_string(x + 1, y, (char *)title);

    if (val[0] != '\0') {
        w = (int16_t)(strlen(val) * 11);
        x = (240 - w) / 2;
        if (x < 3) x = 3;
        y = CY + 4;
        GC9A01_set_text_color(SOFT_GREEN);
        GC9A01_draw_string(x,     y, (char *)val);
        GC9A01_draw_string(x + 1, y, (char *)val);
    }

    GC9A01_set_font(&Font12);
    GC9A01_set_text_color(LIGHT_GRAY);
    GC9A01_draw_string(62, CY + 30, "Updated from server");

    nrf_delay_ms(1000);
}

void dashboard_init_layout(void)
{
    GC9A01_fill_rect(0, 0, 239, 239, DARK_BG);
    GC9A01_draw_circle(CX, CY, 117, DARK_GRAY);
    GC9A01_set_back_color(DARK_BG);

    /* Separators */
    GC9A01_draw_line(MED_GRAY, 25, 62, 215, 62);
    GC9A01_draw_line(MED_GRAY, 20, 117, 220, 117);
    GC9A01_draw_line(MED_GRAY, 25, 177, 215, 177);

    /* Row 2: Vertical divider Temp | SpO2 */
    GC9A01_draw_line(MED_GRAY, 120, 63, 120, 116);

    /* Row 3: Vertical divider Steps | HR (50/50 at x=120) */
    GC9A01_draw_line(MED_GRAY, 120, 118, 120, 176);

    /* ── Row 1: Battery icon placeholder (outline only until battery_valid) ── */
    {
        dashboard_data_t tmp = {0};
        draw_battery_icon(&tmp);
    }

    /* ── Row 2 Left: Temp ── */
    draw_thermometer(32, 78, CYAN);
    GC9A01_set_font(&Font12);
    GC9A01_set_text_color(LIGHT_GRAY);
    GC9A01_draw_string(82, 70, "oC");

    /* ── Row 2 Right: SpO2 icon ── */
    draw_droplet(140, 72, 8, SOFT_GREEN);

    /* ── Row 3 Right: HR layout ── */
    draw_heart(130, 122, 6, SOFT_RED);
    GC9A01_set_font(&Font12);
    GC9A01_set_text_color(LIGHT_GRAY);
    GC9A01_draw_string(180, 120, "bpm");

    /* ── Row 4: ECG label ── */
    GC9A01_set_font(&Font12);
    GC9A01_set_text_color(SOFT_RED);
    GC9A01_draw_string(24, 184, "ECG");

    /* Reset chart sweep indices (display memory is blank after SLPIN) */
    hr_dx = 0; hr_py = 170;
    tmp_dx = 0; tmp_py = 110;
    spo2_idx = 0;
    ecg_dx = 0; ecg_py = 220;

    /* Force all value caches invalid → next update will redraw numbers */
    last_hr = 255;
    last_spo2 = 255;
    last_temp = 0xFFFF;
    last_steps_disp = 0xFFFFFFFF;
    last_rssi = 1;
    last_ble_conn = !last_ble_conn;  /* Toggle → force BLE status full redraw */
    last_ecg_enabled = !last_ecg_enabled;  /* Toggle → force ECG badge redraw */
    last_battery_valid = true;             /* Invalid vs default false → forces first draw */
    last_battery_pct = 0xFF;
}

/* ── Row 1: BLE Status ──
 * Line 1 (y=14): connection status text + battery icon (right)
 * Line 2 (y=36): full 6-byte BLE address + signal bars (only while connected)
 */
void dashboard_update_ble_status(const dashboard_data_t *d)
{
    char buf[24];

    bool name_changed = (strcmp(d->device_name, last_device_name) != 0);
    bool conn_changed = (d->ble_connected != last_ble_conn);

    /* ── Line 1: connection status ── */
    if (conn_changed || name_changed) {
        strncpy(last_device_name, d->device_name, sizeof(last_device_name) - 1);
        last_device_name[sizeof(last_device_name) - 1] = '\0';

        GC9A01_fill_rect(28, 8, 122, 16, DARK_BG);
        GC9A01_set_font(&Font12);
        if (d->ble_connected) {
            GC9A01_set_text_color(SOFT_GREEN);
            GC9A01_draw_string(72, 14, "Connected");
        } else {
            GC9A01_set_text_color(ORANGE);
            GC9A01_draw_string(72, 14, "Scanning...");
        }

        last_ble_conn = d->ble_connected;
        last_rssi = 1;  /* Force line-2 redraw */
    }

    /* ── Line 2: patient name (if connected & set) or BLE address + signal bars ──
     * Redraw only on a meaningful RSSI swing (>10 dBm) to avoid flicker from
     * minor jitter, or when the connection state / patient name changes. */
    int16_t rssi_delta = (int16_t)d->rssi - (int16_t)last_rssi;
    if (rssi_delta < 0) { rssi_delta = -rssi_delta; }
    if (rssi_delta > 10 || conn_changed || name_changed) {
        GC9A01_fill_rect(28, 30, 175, 24, DARK_BG);

        GC9A01_set_font(&Font12);
        GC9A01_set_text_color(LIGHT_GRAY);
        if (d->ble_connected && d->device_name[0] != '\0') {
            GC9A01_draw_string(40, 36, (char*)d->device_name);
        } else {
            snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                     d->mac[5], d->mac[4], d->mac[3], d->mac[2], d->mac[1], d->mac[0]);
            GC9A01_draw_string(40, 36, buf);
        }

        if (d->ble_connected) {
            uint8_t bars;
            uint16_t bar_color;
            if (d->rssi > -60)       { bars = 4; bar_color = SOFT_GREEN; }
            else if (d->rssi > -70)  { bars = 3; bar_color = SOFT_GREEN; }
            else if (d->rssi > -80)  { bars = 2; bar_color = YELLOW; }
            else                     { bars = 1; bar_color = SOFT_RED; }
            draw_signal_bars(175, 36, bars, bar_color);
        }

        last_rssi = d->rssi;
    }

    /* Line-1 clears above overlap the battery icon area (x151-167,y14-22)
     * — always (re)draw it last. */
    draw_battery_icon(d);
    last_battery_valid = d->battery_valid;
    last_battery_pct   = d->battery_pct;
}

/* ── Row 1: Battery icon (gated standalone update) ── */
void dashboard_update_battery(const dashboard_data_t *d)
{
    if (d->battery_valid == last_battery_valid &&
        (!d->battery_valid || d->battery_pct == last_battery_pct))
        return;

    draw_battery_icon(d);
    last_battery_valid = d->battery_valid;
    last_battery_pct   = d->battery_pct;
}

/* ── Row 3 Right: HR + SpO2 ── */
void dashboard_update_hr(const dashboard_data_t *d)
{
    char buf[20];
    uint8_t hr_val = d->hr_valid ? (uint8_t)d->hr : 0;
    uint8_t spo2_val = d->hr_valid ? (uint8_t)d->spo2 : 0;

    uint16_t hlc, hfc, htc;
    if (d->hr_valid) get_hr_colors(hr_val, &hlc, &hfc, &htc);
    else { hlc = DARK_GRAY; hfc = DARK_GRAY; htc = LIGHT_GRAY; }

    /* HR Number — Row 3 right (50/50: x=140, y=118) */
    if (hr_val != last_hr || (!d->hr_valid && last_hr != 0)) {
        GC9A01_fill_rect(140, 118, 38, 16, DARK_BG);
        GC9A01_set_font(&Font16);
        GC9A01_set_text_color(htc);
        if (d->hr_valid) snprintf(buf, sizeof(buf), "%d", hr_val);
        else snprintf(buf, sizeof(buf), "--");
        GC9A01_draw_string(142, 118, buf);
        last_hr = hr_val;
    }

    /* HR Sweep chart — Row 3 right (50/50: x=124, y=175, w=90, h=35) */
    if (d->hr_valid && hr_val > 0) {
        if (hr_dx < 0) { hr_dx = 0; hr_py = 175; }
        sweep_area_chart(124, 175, 90, 35, &hr_dx, &hr_py,
                         hr_val, 40, 150, hlc, hfc);
    } else if (hr_dx != -1) {
        /* Signal lost — clear stale trace so old color doesn't linger */
        GC9A01_fill_rect(124, 140, 90, 35, DARK_BG);
        hr_dx = -1;
    }

    /* SpO2 number — Row 2 right (unchanged from V1) */
    uint16_t sbc, stc;
    if (d->hr_valid && spo2_val > 0) get_spo2_colors(spo2_val, &sbc, &stc);
    else { sbc = DARK_GRAY; stc = LIGHT_GRAY; }

    if (spo2_val != last_spo2 || (!d->hr_valid && last_spo2 != 0)) {
        GC9A01_fill_rect(155, 66, 55, 18, DARK_BG);
        GC9A01_set_font(&Font16);
        GC9A01_set_text_color(stc);
        if (d->hr_valid && spo2_val > 0) {
            snprintf(buf, sizeof(buf), "%d", spo2_val);
            GC9A01_draw_string(157, 68, buf);
            int16_t px = 157 + (spo2_val >= 100 ? 33 : 22) + 2;
            GC9A01_set_font(&Font12);
            GC9A01_set_text_color(LIGHT_GRAY);
            GC9A01_draw_string(px, 70, "%");
        } else {
            GC9A01_draw_string(157, 68, "--");
        }
        last_spo2 = spo2_val;
    }

    /* SpO2 bar chart — Row 2 right (unchanged) */
    if (d->hr_valid && spo2_val > 0) {
        sweep_bar_chart(125, 115, 90, 25, &spo2_idx, 18,
                        spo2_val, 85, 100);
    }
}

/* ── Row 2 Left: Temperature ── */
void dashboard_update_temp(const dashboard_data_t *d)
{
    char buf[20];
    uint16_t temp_x10 = d->temp_valid ? (uint16_t)(d->temperature * 10) : 0;

    uint16_t tlc, tfc, ttc;
    if (d->temp_valid && temp_x10 > 0) get_temp_colors(temp_x10, &tlc, &tfc, &ttc);
    else { tlc = DARK_GRAY; tfc = DARK_GRAY; ttc = LIGHT_GRAY; }

    if (temp_x10 != last_temp) {
        GC9A01_fill_rect(44, 66, 36, 18, DARK_BG);
        GC9A01_set_font(&Font16);
        GC9A01_set_text_color(ttc);
        if (d->temp_valid && temp_x10 > 0)
            snprintf(buf, sizeof(buf), "%d.%d", temp_x10/10, temp_x10%10);
        else snprintf(buf, sizeof(buf), "--.-");
        GC9A01_draw_string(46, 68, buf);
        last_temp = temp_x10;
    }

    if (d->temp_valid && temp_x10 > 0) {
        sweep_area_chart(25, 115, 90, 25, &tmp_dx, &tmp_py,
                         temp_x10, 350, 395, tlc, tfc);
    }
}

/* ── Row 4: ECG ON/OFF badge + sweep (full width) ── */
void dashboard_update_ecg(const dashboard_data_t *d, uint16_t ecg_val)
{
    /* ON/OFF badge — redraw only when ecg_enabled changes */
    if (d->ecg_enabled != last_ecg_enabled) {
        GC9A01_fill_rect(180, 184, 30, 12, DARK_BG);
        GC9A01_set_font(&Font12);
        if (d->ecg_enabled) {
            GC9A01_set_text_color(SOFT_GREEN);
            GC9A01_draw_string(185, 184, "ON");
        } else {
            GC9A01_set_text_color(LIGHT_GRAY);
            GC9A01_draw_string(185, 184, "OFF");
        }
        last_ecg_enabled = d->ecg_enabled;
    }

    /* Sweep waveform — only while ECG streaming is enabled */
    if (d->ecg_enabled) {
        ecg_val = (uint16_t)((ecg_val * 1000) / 4095);
        if (ecg_dx < 0) { ecg_dx = 0; ecg_py = 220; }
        sweep_line(62, 220, 116, 28, &ecg_dx, &ecg_py,
                   ecg_val, 0, 1000, SOFT_RED);
    } else if (ecg_dx != -1) {
        /* ECG disabled — clear sweep rect once, idle sentinel */
        GC9A01_fill_rect(62, 192, 116, 28, DARK_BG);
        ecg_dx = -1;
    }
}

/* ── Row 3 Left: Steps (compact) ── */
void dashboard_update_steps(const dashboard_data_t *d, float ac_value)
{
    char buf[20];

    if (d->steps == last_steps_disp) return;

    /* Update activity */
    s_activity = detect_activity(ac_value, d->cadence, d->steps, s_prev_steps);
    s_prev_steps = d->steps;

    uint16_t act_color = LIGHT_GRAY;
    if (s_activity == ACT_WALK || s_activity == ACT_FAST_WALK) act_color = SOFT_GREEN;
    else if (s_activity == ACT_RUN) act_color = ORANGE;

    GC9A01_fill_rect(20, 118, 88, 42, DARK_BG);
    draw_step_icon(33, 132, act_color);

    GC9A01_set_font(&Font16);
    GC9A01_set_text_color(act_color);
    snprintf(buf, sizeof(buf), "%d", (int)d->steps);
    GC9A01_draw_string(43, 122, buf);

    GC9A01_set_font(&Font12);
    GC9A01_set_text_color(act_color);
    GC9A01_draw_string(28, 148, (char*)act_name(s_activity));

    last_steps_disp = d->steps;
}
