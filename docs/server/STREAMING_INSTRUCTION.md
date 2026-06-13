# STREAMING INSTRUCTION: HTTPS (ECG/PPG) + MQTT Gateway (Vitals)

> **Purpose:** Complete reference for the dual-protocol streaming pipeline.
> Read this alongside `DEVELOPMENT_GUIDE.md` before touching any firmware or
> server-side streaming code.

---

## What already exists (do NOT recreate)

- `lib/thingsboard.js` — JWT auth + token cache (2hr TTL)
- `hooks/useTbWebSocket.js` — ThingsBoard WebSocket subscription hook
- `hooks/useNotifications.js` — Browser alerts + audio for criticals
- `hooks/useTrends.js` — Rolling trend arrows per vital
- `pages/api/telemetry/latest.js` — Proxies TB REST for latest values
- `pages/api/telemetry/history.js` — Proxies TB REST for historical range
- `pages/api/telemetry/ingest.js` — Receives ESP32 batches, posts to TB admin API
- `components/VitalCard.js` — Card for HR / SpO2 / Temperature
- `components/TrendChart.js` — Recharts waveform chart
- `firmware/src/main.cpp` — ESP32 firmware (PlatformIO, Arduino framework)
- `scripts/test-http-stream.js` — Node.js simulator for the full pipeline

---

## Streaming pipeline overview

| Stream | Protocol | Data | Rate | Delivery |
|---|---|---|---|---|
| Raw waveforms | **HTTPS POST** → `/api/telemetry/ingest` | `ecg_batch`, `ppg_batch` | 100 Hz (50-sample batch every 500ms) | Best-effort |
| Vital parameters | **MQTT gateway** → ThingsBoard | `ecgHeartRate`, `ppgHeartRate`, `spo2`, `temperature` | Every 15s | QoS 0 |

---

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│  ESP32 Device                                                │
│                                                              │
│  analogRead() @ 100Hz ──▶ ecgBuf / ppgBuf                   │
│                                  │                           │
│         every 50 samples         │       every 15 seconds    │
│         (500ms)                  ▼       (15000ms)           │
│                           sendRawBatch()  sendVitals()       │
│                                 │               │            │
│                          HTTPS POST     MQTT gateway         │
│                          port 443       port 1883            │
└─────────────────────────────────┼───────────────┼────────────┘
                                  │               │
                                  ▼               ▼
            ┌───────────────────────────────────────────────┐
            │             Next.js App (Vercel)               │
            │                                               │
            │  POST /api/telemetry/ingest                   │
            │    → decode batch into {ts, values} per sample│
            │    → POST to TB admin timeseries API          │
            └───────────────────────┬───────────────────────┘
                                    │ REST (admin token)
                                    ▼
            ┌───────────────────────────────────────────────┐
            │           ThingsBoard Cloud                    │
            │                                               │
            │  MQTT Broker  ──▶  device telemetry storage   │
            │  Timeseries API ◀─ ingest proxy               │
            │                                               │
            │  WebSocket API (output to dashboard)          │
            └───────────────────────┬───────────────────────┘
                                    │ wss://
            ┌───────────────────────────────────────────────┐
            │   HealthMonitor Dashboard                      │
            │                                               │
            │   useTbWebSocket ──▶ live updates             │
            │   /api/telemetry  ──▶ REST fallback            │
            └───────────────────────────────────────────────┘
```

---

## Part 1 — ThingsBoard Cloud Configuration

> You are on **ThingsBoard Cloud** (thingsboard.cloud) — a hosted PE instance.
> You do NOT manage any server infrastructure or TLS certificates.

### TB Cloud hostnames

| Service | Hostname | Port | Protocol |
|---|---|---|---|
| REST API + WebSocket | `thingsboard.cloud` | 443 | HTTPS / WSS |
| MQTT | `mqtt.thingsboard.cloud` | 1883 | MQTT (plain) |

### 1A — MQTT Gateway Device (for HR / SpO2 / Temperature)

The ESP32 connects as a **gateway device** using the gateway MQTT API.
This allows it to publish vitals on behalf of named sensor nodes.

**Step 1: Create gateway device**

`Entities → Devices → + Add Device`
- Name: `wearable-gateway` (or any name)
- Device type: `gateway`
- Enable **Is gateway**: ON

**Step 2: Get gateway access token**

`Entities → Devices → wearable-gateway → Manage Credentials → copy Access Token`

Paste into `firmware/src/main.cpp` → `GATEWAY_TOKEN`.

**MQTT gateway telemetry format** (sent by the firmware):
```json
{
  "Node1": [{ "ts": 1716854400000, "values": { "ecgHeartRate": 72.0, "ppgHeartRate": 71.0, "spo2": 98.5, "temperature": 36.6 } }]
}
```

ThingsBoard automatically creates/updates a device named `Node1` and stores the telemetry.

### 1B — Alarm Rules

Set up in: `Entities → Devices → your sensor device → Alarm Rules`

| Name | Condition | Severity |
|---|---|---|
| HIGH_ECG_HEART_RATE | ecgHeartRate > 130 | Critical |
| LOW_ECG_HEART_RATE  | ecgHeartRate < 40  | Critical |
| HIGH_PPG_HEART_RATE | ppgHeartRate > 130 | Critical |
| LOW_PPG_HEART_RATE  | ppgHeartRate < 40  | Critical |
| CRITICAL_LOW_SPO2 | spo2 < 88 | Critical |
| LOW_SPO2 | spo2 < 94 | Warning |
| HIGH_TEMP | temperature > 39.5 | Critical |
| LOW_TEMP | temperature < 35 | Critical |

These match the `critMin`/`critMax` values in `pages/index.js`.

---

## Part 2 — ESP32 Firmware

### 2A — PlatformIO project structure

```
firmware/
├── platformio.ini          # board, framework, lib_deps
└── src/
    └── main.cpp            # all firmware code
```

**`platformio.ini`:**
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    knolleary/PubSubClient
    bblanchon/ArduinoJson@^6
```

`HTTPClient` and `WiFiClientSecure` are built into the ESP32 Arduino core — no extra lib dep needed.

### 2B — Configuration block (edit before flashing)

All settings are in one block at the top of `firmware/src/main.cpp`:

```cpp
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

const char* INGEST_URL    = "https://your-app.vercel.app/api/telemetry/ingest";

const char* MQTT_HOST     = "mqtt.thingsboard.cloud";
const int   MQTT_PORT     = 1883;
const char* GATEWAY_TOKEN = "YOUR_GATEWAY_ACCESS_TOKEN";
const char* GATEWAY_TOPIC = "v1/gateway/telemetry";

const char* NODE_NAME     = "Node1";    // must match device name in ThingsBoard

#define SAMPLE_RATE_HZ      100         // ECG/PPG sampling frequency
#define SAMPLE_INTERVAL_US  (1000000 / SAMPLE_RATE_HZ)
#define BATCH_SIZE          50          // samples per HTTPS POST
#define VITAL_INTERVAL_MS   15000       // MQTT vital send interval
```

> For multi-node deployments: flash each ESP32 with a different `NODE_NAME`.

### 2C — Key design decisions

- **FreeRTOS task for HTTPS:** `sendRawBatch()` runs on `httpsTask` so TCP/TLS blocking never interrupts the sampling loop.
- **Double buffer:** `ecgBuf`/`ppgBuf` fill while `ecgSend`/`ppgSend` are being sent.
- **NTP timestamps:** `configTime()` syncs epoch time on boot. If NTP fails, `ts` is omitted from the payload and the server uses `Date.now()`.
- **`mqtt.loop()` every iteration:** required for MQTT keepalive — never skip.

---

## Part 3 — Ingest API (`pages/api/telemetry/ingest.js`)

### How it works

```
POST /api/telemetry/ingest
Body: {
  deviceName: "Node1",
  ts:         1716854400000,    // optional epoch ms; server uses Date.now() if absent
  ecg_batch:  "[512,498,501,…]",
  ppg_batch:  "[380,390,385,…]"
}
Response: { ok: true, device: "Node1", points: 50 }
```

The server:
1. Resolves `deviceName` → ThingsBoard device UUID (5-minute cache)
2. Parses `ecg_batch` / `ppg_batch` JSON strings into arrays
3. Reconstructs per-sample timestamps: `sampleTs = batchTs - (n-1-i) * (1000/Hz)`
4. POSTs `[{ts, values:{ecg, ppg}}, …]` to TB admin timeseries API

### Required environment variables

```env
TB_BASE_URL=https://thingsboard.cloud
TB_USERNAME=your-tb-email@example.com
TB_PASSWORD=your-tb-password
```

---

## Part 4 — Dashboard Integration

The existing dashboard already handles ThingsBoard WebSocket and REST.

### 4A — Environment variables (`.env.local`)

```env
TB_BASE_URL=https://thingsboard.cloud
TB_USERNAME=your-tb-email@example.com
TB_PASSWORD=your-tb-password
TB_DEVICE_ID=your-device-uuid

NEXT_PUBLIC_TB_WS_HOST=wss://thingsboard.cloud
NEXT_PUBLIC_DEVICE_ID=your-device-uuid
NEXT_PUBLIC_APP_URL=https://your-app.vercel.app
```

For Vercel: add all vars in `Vercel Dashboard → Project → Settings → Environment Variables`.

### 4B — Waveform data in the dashboard

ECG/PPG arrives in ThingsBoard as time-series `ecg` and `ppg` keys (individual samples, not batches — the ingest server decodes them). The existing `TrendChart` component and `/api/telemetry/history` endpoint serve this data unchanged.

---

## Part 5 — Production Deployment

### 5A — Vercel

```
TB_BASE_URL              = https://thingsboard.cloud
TB_USERNAME              = your-tb-email@example.com
TB_PASSWORD              = your-tb-password
TB_DEVICE_ID             = your-device-uuid
NEXT_PUBLIC_TB_WS_HOST   = wss://thingsboard.cloud
NEXT_PUBLIC_DEVICE_ID    = your-device-uuid
```

`NEXT_PUBLIC_*` vars are embedded at build time — redeploy after changing them.

### 5B — Port reference

| Port | Protocol | Endpoint | Notes |
|---|---|---|---|
| 443 | HTTPS/WSS | `thingsboard.cloud` | REST API + WebSocket + ingest proxy |
| 1883 | TCP | `mqtt.thingsboard.cloud` | MQTT (plain) |

---

## Part 6 — Data Flow Reference

### How ECG/PPG reaches the dashboard

```
ESP32
  → analogRead() at 100Hz
  → 50 samples buffered (500ms)
  → JSON: { deviceName, ts, ecg_batch:"[…50]", ppg_batch:"[…50]" }
  → HTTPS POST → /api/telemetry/ingest

Next.js ingest handler
  → parse arrays (50 values each)
  → reconstruct timestamps: batchTs - (49,48,…,0) × 10ms
  → POST [{ts, values:{ecg,ppg}}, …×50] → TB admin API

ThingsBoard
  → stores ecg / ppg as individual time-series points
  → pushes latest values to WebSocket subscribers

Dashboard
  → useTbWebSocket receives live updates
  → /api/telemetry/history for chart history
```

### How HR/SpO2/Temp reaches the dashboard

```
ESP32
  → computeHR / computeSpO2 / readTemperature every 15s
  → MQTT PUBLISH v1/gateway/telemetry
  → payload: { "Node1": [{ ts, values:{ecgHeartRate, ppgHeartRate, spo2, temperature} }] }

ThingsBoard MQTT Broker
  → stores telemetry under device "Node1"

Dashboard
  → useTbWebSocket / /api/telemetry/latest (10s poll)
  → VitalCard renders HR / SpO2 / Temp
```

---

## Part 7 — Tuning Parameters

> This section covers every knob you can turn to change sampling rate,
> batch size, send interval, and vital cadence.
> **All three locations must be kept in sync** when you change a value.

### 7A — Sample frequency

**What it controls:** How often the ESP32 reads ECG and PPG ADC values.

| Location | What to change |
|---|---|
| `firmware/src/main.cpp` | `SAMPLE_RATE_HZ` |
| `scripts/test-http-stream.js` | `SAMPLE_INTERVAL_MS = 1000 / Hz` |
| `pages/api/telemetry/ingest.js` | line `* 10` → `* (1000 / Hz)` |

**Firmware:**
```cpp
#define SAMPLE_RATE_HZ      100          // ← change here
#define SAMPLE_INTERVAL_US  (1000000 / SAMPLE_RATE_HZ)  // derived automatically
```

**Test script:**
```javascript
const SAMPLE_INTERVAL_MS = 10;   // 1000 / 100Hz — update to match firmware
```

**Server — CRITICAL:** The ingest handler reconstructs per-sample timestamps
using a hardcoded interval. You must update this to match your Hz:
```javascript
// pages/api/telemetry/ingest.js  (~line 79)
const sampleTs = batchTs - (n - 1 - i) * 10;
//                                        ↑
//                           change to: (1000 / SAMPLE_RATE_HZ)
//                           e.g. 200Hz → 5, 50Hz → 20
```

---

### 7B — Batch size (controls HTTP send interval)

**What it controls:** How many samples are bundled into one HTTPS POST.
The send interval is **derived** from batch size — there is no separate timer.

```
HTTP send interval (ms) = BATCH_SIZE × (1000 / SAMPLE_RATE_HZ)
```

| Target interval | At 100Hz | At 200Hz | At 50Hz |
|---|---|---|---|
| 200ms | `BATCH_SIZE = 20` | `BATCH_SIZE = 40` | `BATCH_SIZE = 10` |
| 500ms | `BATCH_SIZE = 50` | `BATCH_SIZE = 100` | `BATCH_SIZE = 25` |
| 1000ms | `BATCH_SIZE = 100` | `BATCH_SIZE = 200` | `BATCH_SIZE = 50` |

**Firmware** — update in one place:
```cpp
#define BATCH_SIZE  50    // ← change here; arrays are sized at compile time
```

**Test script:**
```javascript
const BATCH_SIZE       = 50;    // match firmware
const WAVE_INTERVAL_MS = 1000;  // your desired POST interval for simulation
```

**Server (`ingest.js`):** No change needed. The server reads `n = ecg.length` dynamically and handles any batch size automatically.

**Memory check:** `4 buffers × BATCH_SIZE × 2 bytes` per buffer set.
At `BATCH_SIZE = 200`: 1600 bytes — well within ESP32's 320KB heap.

---

### 7C — Vital send interval

**What it controls:** How often HR, SpO2, and Temperature are sent via MQTT.

| Location | Variable |
|---|---|
| `firmware/src/main.cpp` | `VITAL_INTERVAL_MS` |
| `scripts/test-http-stream.js` | `VITAL_INTERVAL_MS` |

```cpp
// firmware/src/main.cpp
#define VITAL_INTERVAL_MS  15000    // ← change here (milliseconds)
```

```javascript
// scripts/test-http-stream.js
const VITAL_INTERVAL_MS = 15000;   // ← keep in sync with firmware
```

No server-side changes needed.

---

### 7D — Quick reference: all parameters

| Parameter | File | Variable / Line | Notes |
|---|---|---|---|
| Sample rate (Hz) | `firmware/src/main.cpp` | `SAMPLE_RATE_HZ` | Drives `SAMPLE_INTERVAL_US` |
| Sample rate (Hz) | `scripts/test-http-stream.js` | `SAMPLE_INTERVAL_MS` | Set to `1000 / Hz` |
| Sample timestamp spacing | `pages/api/telemetry/ingest.js` | `* 10` in `sampleTs` line | Set to `1000 / Hz` |
| Batch size | `firmware/src/main.cpp` | `BATCH_SIZE` | Controls HTTP interval |
| Batch size | `scripts/test-http-stream.js` | `BATCH_SIZE` | Match firmware |
| HTTP send interval | firmware only | derived: `BATCH_SIZE / Hz × 1000` ms | Not a separate setting |
| HTTP send interval | `scripts/test-http-stream.js` | `WAVE_INTERVAL_MS` | Independent (sim only) |
| Vital interval | `firmware/src/main.cpp` | `VITAL_INTERVAL_MS` | MQTT cadence |
| Vital interval | `scripts/test-http-stream.js` | `VITAL_INTERVAL_MS` | Keep in sync |
| Ingest endpoint | `firmware/src/main.cpp` | `INGEST_URL` | Full HTTPS URL |
| MQTT gateway token | `firmware/src/main.cpp` | `GATEWAY_TOKEN` | TB gateway device token |
| Node name | `firmware/src/main.cpp` | `NODE_NAME` | TB device name per node |

---

### 7E — Example: change from 100Hz to 200Hz

1. **`firmware/src/main.cpp`**
   ```cpp
   #define SAMPLE_RATE_HZ  200      // was 100
   // SAMPLE_INTERVAL_US derived automatically — no further change
   // BATCH_SIZE: keep at 50 (now 250ms per batch) or increase to 100 (500ms)
   ```

2. **`pages/api/telemetry/ingest.js`** (~line 79)
   ```javascript
   const sampleTs = batchTs - (n - 1 - i) * 5;   // was 10, now 5 (1000/200Hz)
   ```

3. **`scripts/test-http-stream.js`**
   ```javascript
   const SAMPLE_INTERVAL_MS = 5;    // was 10, now 5 (1000/200Hz)
   ```

---

## Part 8 — Critical Rules

### Never break

| Rule | File | Reason |
|---|---|---|
| `isAnimationActive={false}` on live waveform charts | `TrendChart.js` | Animation at high Hz crashes browser tab |
| `mqtt.loop()` called every loop iteration | `firmware/src/main.cpp` | MQTT disconnects if not called |
| `micros()` for sampling timing | `firmware/src/main.cpp` | `delay()` drifts and blocks MQTT |
| Ingest timestamp spacing must match `1000/Hz` | `ingest.js` | Wrong spacing = incorrect time-series in TB |
| `BATCH_SIZE` same in firmware and test script | both | Mismatch causes confusing simulation results |
| Keep `httpsTask` stack at 8192 bytes | `firmware/src/main.cpp` | HTTPS + TLS + ArduinoJson need the headroom |
| All secrets in `.env.local` / Vercel env vars | all files | Never hardcode credentials |
| `wss://` not `ws://` in production | `.env.local` | Plain WS blocked on HTTPS pages |
| Do not recreate existing hooks/components | all files | Existing code is tested and working |

### Existing code to preserve exactly

- `hooks/useTrends.js` — no changes
- `hooks/useNotifications.js` — no changes
- `components/VitalCard.js` — no changes
- `components/OverviewGrid.js` — no changes
- `lib/exportCsv.js` — no changes
- `styles/globals.css` — no changes

---

## Checklist — definition of done

- [ ] ESP32 sends HTTPS POST batches every 500ms (50 samples × 10ms at 100Hz)
- [ ] ESP32 sends MQTT gateway vitals every 15s
- [ ] ThingsBoard creates `Node1` device automatically on first MQTT publish
- [ ] `ecg`, `ppg`, `ecgHeartRate`, `ppgHeartRate`, `spo2`, `temperature` visible in TB Latest Telemetry
- [ ] Ingest API returns `{ ok: true, points: 50 }` per batch
- [ ] Dashboard waveform charts update from TB history/WebSocket
- [ ] HR / SpO2 / Temp VitalCards update from both WS and REST
- [ ] Existing alert/notification/trend-arrow system still works
- [ ] Existing OTA flow still works
- [ ] No credentials in browser network tab or source code
- [ ] All env vars set in Vercel dashboard for production
- [ ] HTTPS ingest URL uses `https://` (TLS) in production
