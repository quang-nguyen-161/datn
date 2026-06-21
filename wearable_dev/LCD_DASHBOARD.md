# LCD Dashboard — GC9A01 Layout Reference (Layout V4)

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

## 2. Row layout (current, V4)

```
                 ┌─────────────────────────┐
   y=8..52       │  Row 1: BLE status bar    │  line1: status+battery, line2: address+signal bars
                 ├─────────────┬─────────────┤
   y=63..116     │ Row 2: Temp │ Row 2: SpO2 │  divider x=120, bigger (Font20) numbers
                 ├─────────────────────────┤
   y=118..176    │ Row 3: HR sweep (left, fills row) | HR number (right, bigger) │
                 ├─────────────────────────┤
   y=180..~232   │ Row 4: ECG label+ON/OFF (left) | ECG sweep | ECG-HR number (right, bigger) │
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

Row 3 and Row 4 no longer have a vertical divider — the HR/ECG number columns
sit on the right with no hard boundary; the sweep chart simply fills whatever
space is left of them.

---

## 3. Widget map (coordinates, colors, redraw triggers)

### Row 1 — BLE status (`dashboard_update_ble_status`)

| Element | Position | Notes |
|---|---|---|
| Connection status (line 1) | `(72, 14)` Font12 | "Connected" (SOFT_GREEN) or "Scanning..." (ORANGE). |
| Battery icon | outline `(151,14)-(167,22)` incl. nub, drawn by `draw_battery_icon()` | Right end of line 1. Outline-only placeholder when `battery_valid==false` (always true today — no ADC battery reading wired up); interior `(152,15,13,7)` fills proportionally to `battery_pct` (red <20%, yellow <50%, green else) once `battery_valid` is set by `main.c`. |
| Address / patient name (line 2) | `(40, 36)` Font12, LIGHT_GRAY | If `ble_connected && device_name[0]!='\0'`: shows the patient name (from `g_patient_name`, set via `CMD_NAME_CFG`). Otherwise shows the full 6-byte BLE address `XX:XX:XX:XX:XX:XX` (MSB-first), from `ble_app_get_addr()`. |
| Signal bars | `(175, 36)` | 4-bar antenna icon, `draw_signal_bars()`. Only drawn when `ble_connected`; color-coded from `rssi` (green > -60dBm, green -70, yellow -80, red below). No numeric dBm shown. |

Redraw is gated:
- Line 1 clear (`fill_rect(28,8,122,16,DARK_BG)`) only when `ble_connected`
  or `device_name` changes.
- Line 2 clear (`fill_rect(28,30,175,24,DARK_BG)`) when `rssi` changes,
  connection state changes, or `device_name` changes.
- Neither clear overlaps the battery icon area (x151-167,y14-22), but
  `dashboard_update_ble_status()` unconditionally redraws `draw_battery_icon()`
  at the end of every call anyway. A separate gated `dashboard_update_battery()`
  exists for standalone use (compares `battery_valid`/`battery_pct` against
  `last_battery_*`).

> **Note**: `dashboard_update_ble_status()` is called from `main.c`'s periodic
> LCD-vitals refresh block (every `g_vital_interval_ms`), populating
> `ble_connected`/`rssi`/`mac` from `ble_app_is_connected()`/`ble_app_get_rssi()`/
> `ble_app_get_addr()`, and `device_name` from `g_patient_name` (cmd.c,
> `CMD_NAME_CFG` 0xC9) whenever connected and non-empty.

### Row 2 Left — Temperature (`dashboard_update_temp`)

| Element | Position | Notes |
|---|---|---|
| Thermometer icon | `draw_thermometer(32, 78, CYAN)` | static, drawn once in `init_layout` |
| "°C" unit label | `(98, 70)` Font12, LIGHT_GRAY | static — moved right to clear the wider Font20 value |
| Value (e.g. "36.5") | `fill_rect(38,62,62,22)` then `(40,63)` Font20 | redraws only if `temp_x10` changed. Color via `get_temp_colors()`. |
| Sweep area chart | `sweep_area_chart(25, 115, 90, 25, ...)`, range `[350,395]` (35.0–39.5°C) | persistent state `tmp_dx/tmp_py`, unchanged from V3 |

`get_temp_colors(t)` (t = temp×10): `<35.5 or ≥38.5` → red, `≥37.6` → yellow,
else cyan.

### Row 2 Right — SpO2 (`dashboard_update_hr`, second half)

| Element | Position | Notes |
|---|---|---|
| Droplet icon | `draw_droplet(140, 72, 8, SOFT_GREEN)` | static |
| Value + "%" | `fill_rect(150,62,65,22)` → `(152,64)` Font20 number, `%` at `px = 152 + (≥100?42:28) + 2` Font12 | redraws on `spo2_val` change |
| Sweep bar chart | `sweep_bar_chart(125, 115, 90, 25, ..., nb=18)`, range `[85,100]` | persistent `spo2_idx`, unchanged from V3 |

`get_spo2_colors(v)`: `<90` red, `≤94` yellow, else green.

### Row 3 — HR (`dashboard_update_hr`)

Steps/activity (V3's Row 3 left half) was **removed** — `dashboard_update_steps()`,
`draw_step_icon()`'s activity-tinted call site, `detect_activity()`, and the
step/cadence fields in `dashboard_data_t` are gone. The pedometer itself
(`pedometer_update()`, `g_sensor.steps`/`cadence` in `main.c`) still runs, it's
just no longer fed to the LCD. The freed space now belongs to a widened HR
sweep chart; the HR number moved into a column on the right.

| Element | Position | Notes |
|---|---|---|
| Heart icon | `draw_heart(155, 128, 6, SOFT_RED)` | static, right column |
| "bpm" label | `(164, 142)` Font12, LIGHT_GRAY | static, right column |
| HR number | `fill_rect(160,118,50,22)` → `(162,119)` **Font20** | redraws on `hr_val` change. Color via `get_hr_colors()`. |
| "SAT!" blink badge | `fill_rect(148,156,65,18)`, `dashboard_update_sat_badge()` | moved down/left to stay clear of the bigger number; unrelated to this layout pass otherwise |
| Sweep area chart | `sweep_area_chart(20, 176, 125, 50, ...)`, range `[40,150]` bpm | **widened** to `w=125,h=50` (was `90×35`) to fill the space freed by removing Steps. persistent `hr_dx/hr_py`. When `hr_valid` is false, the chart rect `(20,126,125,50)` is cleared once and `hr_dx` set to `-1` (idle sentinel) so a stale (possibly red) trace doesn't linger; resets to `0` when HR becomes valid again. |

`get_hr_colors(v)`: `<51 or >130` → red, `≥100` → yellow, else green.

### Row 4 — ECG + ON/OFF badge + ECG-derived HR number (`dashboard_update_ecg`)

| Element | Position | Notes |
|---|---|---|
| "ECG" label | `(24, 184)` Font12, SOFT_RED | static |
| ON/OFF badge | `(58, 184)` Font12 | **moved left**, next to the "ECG" label, to make room for the number on the right. "ON" (SOFT_GREEN) / "OFF" (LIGHT_GRAY), redraws only when `d->ecg_enabled` changes vs `last_ecg_enabled`. Clear rect `(55,184,32,12)`. |
| Sweep line | `sweep_line(62, 206, 88, 14, ...)`, range `[0,1000]`, color SOFT_RED | **narrowed** to `w=88` (was `116`) to leave room for the HR number. Only drawn/updated while `d->ecg_enabled` is true. Input `ecg_val` pre-scaled: `ecg_val * 1000 / 4095`. persistent `ecg_dx/ecg_py`. |
| "bpm" label | `(153, 201)` Font12, LIGHT_GRAY | static, right column |
| ECG-derived HR number | `fill_rect(150,178,50,22)` → `(152,179)` **Font20** | new in V4. Source: `d->hr_ecg`/`d->hr_ecg_valid`, wired in `main.c` from `g_sensor.hr_ecg`/`g_sensor.hr_ecg_valid` (gated on `g_ecg_stream_enabled`). Color via `get_hr_colors()` (reuses the PPG-HR bpm thresholds — no separate ECG-HR threshold exists). |

When `d->ecg_enabled` is false, the sweep rect `(62,192,88,28)` is cleared
once and `ecg_dx` set to `-1` (idle sentinel, same pattern as the Row3 HR
chart) so no stale trace lingers; resets to `0`/`206` when ECG turns back
on. `d->ecg_enabled` is copied from `g_ecg_stream_enabled` (cmd.h) by
`main.c` each tick before calling `dashboard_update_ecg()`.

> Sweep rect is now 88×28, narrower than V3's 116×28 — the right column needs
> the room, and the bezel curve at y≈220-222 already constrained the old
> width to begin with (see §7 checklist note on bezel math).

Note: `ecg_synthetic()` exists in the file (generates a fake ECG waveform from
HR) but is **not currently wired up** — `dashboard_update_ecg` is called with
the real `s_ecg_display` value from `main.c`. Keep this in mind if reviving it
for a demo/no-ECG-sensor mode.

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
static int16_t hr_dx=0, hr_py=176;
static int16_t tmp_dx=0, tmp_py=110;
static int16_t spo2_idx=0;
static int16_t ecg_dx=0, ecg_py=206;
static uint8_t  last_hr=255, last_spo2=255;
static uint16_t last_temp=0xFFFF;
static uint8_t  last_hr_ecg=255;
static int8_t   last_rssi=1;
static bool     last_ble_conn=false;
static char     last_device_name[16] = "";
static bool     last_ecg_enabled=false;
static bool     last_battery_valid=false;
static uint8_t  last_battery_pct=0xFF;
```

`hr_dx`/`ecg_dx` double as "idle sentinels": `-1` means the corresponding
sweep chart is idle (signal invalid / ECG off) and its rect has been cleared;
both reset to `0` (with `py` reset too) when the signal/feature comes back.

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
  + pedometer_update()    → g_sensor.steps, cadence    (no longer fed to the LCD)
g_ecg_stream_enabled      → s_dash.ecg_enabled,
  + R-peak HR              s_dash.hr_ecg/hr_ecg_valid  → dashboard_update_ecg()
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

- `ecg_synthetic()` is unused dead code — either remove or repurpose as a
  fallback waveform when ECG is disabled (now that Row 4 has an ON/OFF state,
  this could drive a demo waveform while `ecg_enabled` is false).
- No layout currently uses Font24 except the splash screen — available for a
  future "focus" view (e.g. a full-screen single-metric mode).
- Battery: `dashboard_data_t.battery_valid`/`battery_pct` exist and the icon
  renders as outline-only placeholder; no ADC battery-voltage reading is wired
  up yet. Populate these from `main.c` once battery sensing exists — no
  dashboard.c changes should be needed.
