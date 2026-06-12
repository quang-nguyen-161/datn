# LCD Dashboard — GC9A01 Layout Reference (Layout V2)

Reference for the round 240×240 GC9A01 dashboard (`drivers/display/dashboard.c/h`,
`drivers/display/GC9A01.c/h`). Intended as a map of the current layout plus
notes for adding/rearranging widgets.

---

## 1. Display basics

- Panel: GC9A01A, **240×240**, circular viewport (visible content roughly
  inscribed in the 240px square — corners are physically hidden by the bezel).
- Center: `CX = 120, CY = 120`. Outer guide ring drawn at
  `GC9A01_draw_circle(CX, CY, 117, DARK_GRAY)`.
- Color format: RGB565 via `RGB565(r,g,b)`. Background is `DARK_BG = RGB565(15,15,20)`
  (near-black, not pure `BLACK`) — **always clear/redraw with `DARK_BG`**, not `BLACK`,
  or you'll get visible seams against the rest of the UI.
- Fonts: `Font8`, `Font12`, `Font16`, `Font24` (from `fonts.h`). Approx. width:
  Font12 ≈ 7px/char (used for centering math in `dashboard_update_ble_status`).
- Drawing primitives (`GC9A01.h`): `fill_rect`, `draw_line`, `draw_circle`,
  `fill_circle`, `draw_pixel`, `draw_string`/`printf`, plus `set_font`/
  `set_text_color`/`set_back_color` (must be set before each `draw_string`).

---

## 2. Row layout (current, V2)

```
                 ┌─────────────────────────┐
   y=8..52       │   Row 1: BLE status bar   │  name/MAC, RSSI, signal bars
                 ├─────────────┬─────────────┤
   y=63..116     │ Row 2: Temp │ Row 2: SpO2 │  divider x=120
                 ├─────────────┼─────────────┤
   y=118..176    │ Row 3: ECG  │ Row 3: HR   │  divider x=120 (50/50)
                 ├─────────────┴─────────────┤
   y=180..225    │   Row 4: Steps/Activity    │
                 └─────────────────────────┘
```

Horizontal separator lines (drawn in `dashboard_init_layout()`):

| y | x range | purpose |
|---|---|---|
| 62  | 25–215 | Row1 / Row2 |
| 117 | 20–220 | Row2 / Row3 |
| 177 | 25–215 | Row3 / Row4 |

Vertical dividers:

| x | y range | purpose |
|---|---|---|
| 120 | 63–116  | Temp \| SpO2 (Row 2) |
| 120 | 118–176 | ECG \| HR (Row 3, 50/50) |

---

## 3. Widget map (coordinates, colors, redraw triggers)

### Row 1 — BLE status (`dashboard_update_ble_status`)

| Element | Position | Notes |
|---|---|---|
| Device name / MAC (line 1) | centered, `y=14`, Font12 | Centered via `text_x = (240 - strlen*7)/2`. Shows device name if set, else MAC `XX:XX:XX:XX` (last 4 bytes). |
| "Scanning..." | `(72, 38)` Font12, ORANGE | Only when disconnected. |
| RSSI number | `(62, 38)` Font12 | Color-coded: green > -60dBm, green -70, yellow -80, red below. |
| "dBm" label | right after RSSI number | LIGHT_GRAY |
| Signal bars | `bars_x = 62 + num_w + 3 + 22 + 6`, `y=36` | 4-bar antenna icon, `draw_signal_bars()`. |

Redraw is gated:
- Full Row-1 clear (`fill_rect(28,8,185,52,DARK_BG)`) only when `ble_connected`
  or `device_name` changes.
- RSSI line (`fill_rect(40,34,172,20,DARK_BG)`) only when `rssi` changes.

### Row 2 Left — Temperature (`dashboard_update_temp`)

| Element | Position | Notes |
|---|---|---|
| Thermometer icon | `draw_thermometer(32, 78, CYAN)` | static, drawn once in `init_layout` |
| "°C" unit label | `(82, 70)` Font12, LIGHT_GRAY | static |
| Value (e.g. "36.5") | `fill_rect(44,66,36,18)` then `(46,68)` Font16 | redraws only if `temp_x10` changed. Color via `get_temp_colors()`. |
| Sweep area chart | `sweep_area_chart(25, 115, 90, 25, ...)`, range `[350,395]` (35.0–39.5°C) | persistent state `tmp_dx/tmp_py` |

`get_temp_colors(t)` (t = temp×10): `<35.5 or ≥38.5` → red, `≥37.6` → yellow,
else cyan.

### Row 2 Right — SpO2 (`dashboard_update_hr`, second half)

| Element | Position | Notes |
|---|---|---|
| Droplet icon | `draw_droplet(140, 72, 8, SOFT_GREEN)` | static |
| Value + "%" | `fill_rect(155,66,55,18)` → `(157,68)` Font16 number, `%` at `px = 157 + (≥100?33:22) + 2` Font12 | redraws on `spo2_val` change |
| Sweep bar chart | `sweep_bar_chart(125, 115, 90, 25, ..., nb=18)`, range `[85,100]` | persistent `spo2_idx` |

`get_spo2_colors(v)`: `<90` red, `≤94` yellow, else green.

### Row 3 Left — ECG (`dashboard_update_ecg`)

| Element | Position | Notes |
|---|---|---|
| "ECG" label | `(22, 119)` Font12, SOFT_RED | static |
| Sweep line | `sweep_line(20, 175, 96, 42, ...)`, range `[0,1000]`, color SOFT_RED | persistent `ecg_dx/ecg_py`. Input `ecg_val` is pre-scaled: `ecg_val * 1000 / 4095`. |

Note: `ecg_synthetic()` exists in the file (generates a fake ECG waveform from
HR) but is **not currently wired up** — `dashboard_update_ecg` is called with
the real `s_ecg_display` value from `main.c`. Keep this in mind if reviving it
for a demo/no-ECG-sensor mode.

### Row 3 Right — HR (`dashboard_update_hr`, first half)

| Element | Position | Notes |
|---|---|---|
| Heart icon | `draw_heart(130, 122, 6, SOFT_RED)` | static |
| "bpm" label | `(180, 120)` Font12, LIGHT_GRAY | static |
| HR number | `fill_rect(140,118,38,16)` → `(142,118)` Font16 | redraws on `hr_val` change. Color via `get_hr_colors()`. |
| Sweep area chart | `sweep_area_chart(124, 175, 90, 35, ...)`, range `[40,150]` bpm | persistent `hr_dx/hr_py` |

`get_hr_colors(v)`: `<51 or >130` → red, `≥100` → yellow, else green.

### Row 4 — Steps / Activity (`dashboard_update_steps`)

| Element | Position | Notes |
|---|---|---|
| Step icon | `draw_step_icon(38, 194, SOFT_GREEN)` | static |
| Step count | `(52, 183)` Font16, color = activity color | |
| Activity name | `(115, 185)` Font12 | "REST"/"MOVING"/"WALK"/"F.WALK"/"RUN" |
| Distance | `(160, 185)` Font12, WHITE | meters or `X.Ykm` |
| Progress bar track | `fill_rect(40, 208, 160, 5, MED_GRAY)` | |
| Progress bar fill | `fill_rect(40, 208, prog_w, 5, SOFT_GREEN)`, `prog_w = steps/5000 * 160` capped at 5000 | |
| "N/5000" label | `(82, 216)` Font8, LIGHT_GRAY | |

Whole row only redraws when `d->steps != last_steps_disp` (early return otherwise).
Activity classification (`detect_activity`) uses `ac_value` (accel AC component)
and `cadence`:
- `ac < 0.05 && !stepping` → REST
- `!stepping` → MOVING
- `cadence < 120` → WALK, `<150` → F.WALK, `≥150` → RUN

> **Caveat**: `ac_value` comes from `g_accel.ac`, which per
> [SENSOR_ALGORITHMS.md](SENSOR_ALGORITHMS.md) §4.2 is currently **always 0**
> (the accel filter that would populate it is never called). So REST/MOVING
> split is effectively dead — activity is driven almost entirely by `cadence`.
> Fix `g_accel.ac` first if you want REST vs. MOVING to actually work.

---

## 4. Reusable chart primitives

All three "sweep" widgets erase a small window ahead of the draw cursor and
wrap around (`*dx`/`*idx` modulo width), so they run forever without a full
redraw — useful for any new trend widget.

| Function | Style | Signature |
|---|---|---|
| `sweep_area_chart` | filled area under a line, 1px highlight on top edge | `(x0,y0,w,h,&dx,&py, val,vmin,vmax, line_color, fill_color)` |
| `sweep_line` | plain connected line | `(x0,y0,w,h,&dx,&py, val,vmin,vmax, color)` |
| `sweep_bar_chart` | discrete bars, color-coded per value via `get_spo2_colors` | `(x0,y0,w,h,&idx, nb, val,vmin,vmax)` |

All three draw against the **bottom-left origin** of their box: `(x0,y0)` is
the bottom-left corner, `h` extends upward, `w` extends right. `vmin`/`vmax`
define the value→height mapping (clamped).

`sweep_bar_chart` hardcodes `get_spo2_colors()` for bar color — if reusing it
for a non-percentage metric, either generalize the color callback or accept
the SpO2 color thresholds as a placeholder.

---

## 5. Persistent state (must reset in `dashboard_init_layout`)

```c
static int16_t hr_dx=0, hr_py=170;
static int16_t tmp_dx=0, tmp_py=110;
static int16_t spo2_idx=0;
static int16_t ecg_dx=0, ecg_py=150;
static uint8_t  last_hr=255, last_spo2=255;
static uint16_t last_temp=0xFFFF;
static uint32_t last_steps_disp=0xFFFFFFFF;
static int8_t   last_rssi=1;
static bool     last_ble_conn=false;
static char     last_device_name[16] = "";
```

These are the "previous value" caches that gate redraws (avoid flicker /
unnecessary SPI traffic) and the sweep-chart cursor positions. Any new widget
that redraws conditionally needs its own `last_*` static, and
`dashboard_init_layout()` must reset it to an "invalid" sentinel so the first
update after `SLPIN`/wake always redraws.

---

## 6. Call flow (`main.c` sensor tick, every 10 ms)

```
max30102_process()       → s_dash.hr, s_dash.spo2     → dashboard_update_hr()
MMA8452Q_read()
  + pedometer_update()    → s_dash.steps, s_dash.cadence
                                                        → dashboard_update_steps()
                                                        → dashboard_update_ecg()
TMP117 one-shot state machine
  → g_sensor.temp, temp_valid                          → dashboard_update_temp()
(BLE event handlers)       → s_dash.rssi, ble_connected,
                              device_name, mac          → dashboard_update_ble_status()
```

`dashboard_update_*` functions are cheap to call every tick — each does its
own "did this value change" check before touching the display. New widgets
should follow the same pattern (compare to a `last_*` static, only redraw the
changed sub-rectangle).

---

## 7. Adding a new widget — checklist

1. **Pick free screen real estate.** The circular bezel hides corners — keep
   content within the drawn guide circle (`r=117` from center `120,120`), i.e.
   roughly avoid `x<3` or `x>237` near `y≈120`, and more margin near `y=0/240`.
2. **Add fields to `dashboard_data_t`** (`dashboard.h`) for any new sensor
   value, and populate them in `main.c`'s sensor tick.
3. **Static layout** (icons, labels, dividers) → draw once in
   `dashboard_init_layout()`.
4. **Dynamic value** → new `dashboard_update_xxx()` function:
   - Add a `last_*` static cache, initialize to an invalid sentinel, reset it
     in `dashboard_init_layout()`.
   - `fill_rect()` only the changed sub-area with `DARK_BG` before redrawing
     text/number.
   - Pick colors via a `get_xxx_colors()` helper if the value has
     normal/warning/critical bands (follow `get_hr_colors`/`get_temp_colors`/
     `get_spo2_colors` pattern).
5. **Trend chart** → reuse `sweep_area_chart`/`sweep_line`/`sweep_bar_chart`
   with a fresh `dx`/`py` (or `idx`) static pair, reset in
   `dashboard_init_layout()`.
6. **Wire into `main.c`** — call the new `dashboard_update_xxx()` from the
   sensor tick, gated on `s_lcd_present` and (if relevant)
   `s_<sensor>_present`.
7. Update this file's row/coordinate tables so future layout changes don't
   collide with existing widgets.

---

## 8. Known issues / future-dev notes

- `g_accel.ac` dead-code issue (see §3 Row 4 caveat and
  [SENSOR_ALGORITHMS.md](SENSOR_ALGORITHMS.md) §4.2) — fix to make REST vs.
  MOVING activity detection meaningful.
- `ecg_synthetic()` is unused dead code — either remove or repurpose as a
  fallback waveform when ECG is disconnected/disabled.
- Row 1 centering math (`strlen(name) * 7`) assumes Font12 is exactly 7px/char;
  verify against `fonts.h` if switching fonts for that row.
- No layout currently uses Font24 except the splash screen — available for a
  future "focus" view (e.g. a full-screen single-metric mode).
