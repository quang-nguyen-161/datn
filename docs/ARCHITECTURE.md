# WearableDev Health Monitor — Architecture & Data Flow

## System Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         WEARABLE NODE                               │
│  nRF52832 Peripheral  ──BLE──►  nRF52832 Central  ──UART──►  ESP32 │
│  (ECG/PPG sensor)               (aggregator)              (gateway) │
└────────────────────────────────────┬────────────────────────────────┘
                                     │ MQTT (all telemetry)
                                     ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    THINGSBOARD CE  (103.116.39.179)              │
│  Gateway device  ──auto-creates──►  Node1 / Node2 / Node3           │
│  Stores: ecg, ppg, ecgHeartRate, ppgHeartRate, spo2, temperature    │
│  Attributes: thresholds, vitalInterval, ecgSampleFreq, patient info │
└──────────────────────────┬──────────────────────────────────────────┘
                           │  WebSocket (wss://)  +  REST API (https://)
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│              NEXT.JS APP  (wearable-server.vercel.app)              │
│  Serves React UI — all TB API calls happen from the BROWSER         │
│  (bypasses Cloudflare bot protection on the TB server)              │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 1. Authentication Flow

### Why a login page?

The TB CE server is behind **Cloudflare bot protection**. Vercel's serverless
functions are blocked (returns 403 + Cloudflare challenge HTML) because they
look like automated bots. The browser is not blocked because it executes
JavaScript and stores the `cf_clearance` cookie.

### Login sequence

```
Browser                    Cloudflare               ThingsBoard CE
   │                           │                         │
   │── POST /api/auth/login ──►│                         │
   │   (first visit: Cloudflare│                         │
   │    challenge appears,     │                         │
   │    browser solves it)     │                         │
   │◄── cf_clearance cookie ───│                         │
   │                           │                         │
   │── POST /api/auth/login ──►│── forward ─────────────►│
   │                           │                         │── validate ──┐
   │                           │                         │◄─────────────┘
   │◄──────────────────────────┼──── { token: "..." } ───│
   │                           │                         │
   │  Store JWT in sessionStorage (expires in 2.5h)      │
   │  Redirect to dashboard                              │
```

### Token lifecycle

| Storage | Key | Expires |
|---|---|---|
| `sessionStorage` | `tb_token` | 2.5 hours (TB JWT expiry) |
| `sessionStorage` | `tb_token_expiry` | same |

On any 401 response from TB, the app clears the token and redirects to `/login`.
On tab close, `sessionStorage` is cleared (user must log in again).

---

## 2. Real-Time Vitals Flow (WebSocket)

```
ThingsBoard CE
      │
      │  wss://103.116.39.179/api/ws/plugins/telemetry?token=<JWT>
      │
      ▼
useTbWebSocket hook (browser)
      │
      │  Subscribes to LATEST_TELEMETRY for the selected device
      │  Receives: ppgHeartRate, ecgHeartRate, spo2, temperature
      │            ecg_batch (JSON array), ppg_batch (JSON array)
      │
      ├── pendingVitals → flush every 16ms → setVitals()
      ├── pendingEcg    → flush every 16ms → setEcgData()  (rolling buffer)
      └── pendingPpg    → flush every 16ms → setPpgData()  (rolling buffer)
```

**Buffer size** = `max(1500, ecgSampleFreq × 10)` points
(10 seconds of data at the configured sample rate)

**Flush rate** = 60fps (16ms interval) — batches incoming messages to avoid
React re-render thrashing on high-frequency ECG data.

---

## 3. ECG/PPG Waveform Flow

### Firmware → ThingsBoard

```
ESP32 (every WAVE_INTERVAL_MS — 200ms for 50-sample batches)
  │
  │  MQTT PUBLISH  mqtt://103.116.39.179:1883
  │  Topic:  v1/gateway/telemetry
  │  Auth:   TB_GATEWAY_ACCESS_TOKEN as MQTT username
  │  Body: {
  │    "Node1": [{ "ts": epochMs, "values": { "ecg_batch": "[2048,2391,...]" } }],
  │    "Node4": [{ "ts": epochMs, "values": { "ecg_batch": "[...]" } }]
  │  }
  │
  ▼
ThingsBoard gateway API auto-creates leaf devices on first publish
  │
  ▼
TB stores ecg_batch as a time-series key (ts = explicit epoch from payload)
  │
  ▼
WebSocket pushes ecg_batch to dashboard
  │
  ▼
parseWaveformBatch() reconstructs timestamps client-side
  │
  ▼
TrendChart (isLiveWaveform) renders scrolling ECG waveform
```

### Sample timestamp reconstruction

`parseWaveformBatch()` works backwards from the stored batch timestamp:

```
sample[N-1].ts = batchTs            (latest sample = batch timestamp)
sample[N-2].ts = batchTs - interval
sample[N-3].ts = batchTs - 2×interval
...
interval = 1000 / ecgSampleFreq  (ms per sample)
```

---

## 4. Vitals Flow (Heart Rate, SpO2, Temperature)

```
ESP32 (every VITAL_INTERVAL_MS — 15s)
  │
  │  MQTT PUBLISH  mqtt://103.116.39.179:1883
  │  Topic:  v1/gateway/telemetry
  │  Body: {
  │    "Node1": [{ "ts": epochMs, "values": {
  │      "ecgHeartRate": 72.5, "ppgHeartRate": 71.0,
  │      "spo2": 98.2, "temperature": 36.6
  │    }}],
  │    "Node4": [{ ... }]
  │  }
  │
  ▼
ThingsBoard stores as LATEST_TELEMETRY
  │
  ▼
WebSocket delivers to dashboard → VitalCard renders with threshold colors
```

---

## 5. Settings & Attributes Flow

### Loading settings (browser → TB directly)

```
Dashboard / Settings page
  │
  │  useTbAuth() → get JWT from sessionStorage
  │
  │  GET /api/plugins/telemetry/DEVICE/{id}/values/attributes/SERVER_SCOPE
  │  GET /api/plugins/telemetry/DEVICE/{id}/values/attributes/SHARED_SCOPE
  │  (called directly from browser — bypasses Cloudflare via browser JWT)
  │
  ▼
SettingsContext caches result per deviceId
  │
  ▼
Dashboard uses: vitalInterval, ecgSampleFreq, ecgPacketInterval, thresholds
```

### Saving settings

```
Settings page → saveDeviceAttributes(token, deviceId, scope, attrs)
  │
  ├── SERVER_SCOPE: threshold values (ppgHr_normalMin, etc.)
  └── SHARED_SCOPE: vitalInterval, ecgSampleFreq, ecgPacketInterval
                    (firmware reads SHARED_SCOPE via TB attribute subscription)
```

### Attribute key mapping

| UI Name | TB Key | Scope |
|---|---|---|
| Vital interval | `vitalInterval` | SHARED |
| ECG sample freq | `ecgSampleFreq` | SHARED |
| ECG packet interval | `ecgPacketInterval` | SHARED |
| PPG HR normal min/max | `ppgHr_normalMin/Max` | SERVER |
| ECG HR normal min/max | `ecgHr_normalMin/Max` | SERVER |
| SpO₂ thresholds | `spo2_normalMin/Max` etc. | SERVER |
| Temperature thresholds | `temp_normalMin/Max` etc. | SERVER |
| Patient name | `patientName` | SERVER |

---

## 6. No-Signal Detection

```
lastBatchTs (updated each time ECG/PPG batch arrives)
  │
  │  timeout = max(vitalInterval × 3, 10000) ms
  │  (3× the expected interval, minimum 10s)
  │
  ▼
noSignal = true → chart header shows "NO SIGNAL" in amber
noSignal = false → "LIVE" in green
```

---

## 7. Patient Name → Node Name Mapping

Patient info is stored as `SERVER_SCOPE` attributes on each node device in TB.
`/api/devices` (and `getDevices()` browser client) fetches `patientName` alongside
device status and exposes it as `device.patientName`.

Display pattern everywhere in the UI:
- **Primary**: `patientName` (if set)
- **Secondary**: `nodeName` shown below in smaller text

---

## 8. Environment Variables

### Server-side (Next.js API routes — used by ingest endpoint)

| Variable | Purpose |
|---|---|
| `TB_BASE_URL` | ThingsBoard CE base URL |
| `TB_USERNAME` | Tenant admin username |
| `TB_PASSWORD` | Tenant admin password |
| `TB_DEVICE_ID` | Gateway device UUID |
| `TB_GATEWAY_ACCESS_TOKEN` | Gateway access token (for ingest) |

### Client-side (browser — `NEXT_PUBLIC_*`, baked into build)

| Variable | Purpose |
|---|---|
| `NEXT_PUBLIC_TB_BASE_URL` | TB URL for browser-side API calls |
| `NEXT_PUBLIC_TB_DEVICE_ID` | Gateway UUID for device relation queries |
| `NEXT_PUBLIC_TB_WS_URL` | WebSocket URL for real-time telemetry |

> **Important**: `NEXT_PUBLIC_*` variables are embedded at **build time**.
> After changing them in Vercel, trigger a new deployment.

---

## 9. Key Files

```
pages/
  login.js                  ← Login page (browser authenticates to TB directly)
  index.js                  ← Main dashboard
  settings.js               ← Per-device threshold & firmware settings
  api/
    telemetry/ingest.js     ← Receives firmware data, forwards to TB
    health.js               ← Diagnostic endpoint

context/
  TbAuthContext.js          ← Browser JWT management (login / logout / token)
  SettingsContext.js        ← Per-device settings cache (thresholds, intervals)

lib/
  tbBrowserClient.js        ← Browser-side TB API calls (bypasses Cloudflare)
  thingsboard.js            ← Server-side TB API calls (used by ingest only)

hooks/
  useTbWebSocket.js         ← WebSocket connection + 60fps flush loop
  useTrends.js              ← Rolling trend arrows (up/down) for vital cards
  useNotifications.js       ← Browser push notifications on threshold breach

components/
  VitalCard.js              ← Single vital metric display with threshold colors
  TrendChart.js             ← Recharts wrapper (live waveform + history modes)
  OverviewGrid.js           ← All-nodes snapshot grid
  NodeDetailModal.js        ← Per-node history + vitals detail
  VitalHistoryModal.js      ← Full-screen vital trend chart
  PrintModal.js             ← PDF/CSV export
  OtaModal.js               ← OTA firmware update via BLE DFU

firmware/
  src/main.cpp              ← ESP32: reads ECG/PPG, posts to ingest endpoint
```

---

## 10. Deployment Checklist

```
Vercel environment variables (Settings → Environment Variables):
  ✓ TB_BASE_URL
  ✓ TB_USERNAME
  ✓ TB_PASSWORD
  ✓ TB_DEVICE_ID
  ✓ TB_GATEWAY_ACCESS_TOKEN
  ✓ NEXT_PUBLIC_TB_BASE_URL
  ✓ NEXT_PUBLIC_TB_DEVICE_ID
  ✓ NEXT_PUBLIC_TB_WS_URL

After changing any NEXT_PUBLIC_* var → must redeploy (rebuild).

Verify deployment:
  GET https://your-app.vercel.app/api/health
  → gateway.ok: true, auth.ok: true
```
