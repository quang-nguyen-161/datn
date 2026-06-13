# Credentials & Configuration Setup

This file documents every credential and endpoint that must be configured
to run the Health Monitor system across all platforms.

---

## 1. What you need from ThingsBoard

Log in to ThingsBoard as tenant admin and collect these four values before
configuring any platform.

### 1a. Server URLs

| Value | Where to find | Example |
|---|---|---|
| TB base URL (HTTPS) | Your deployment hostname | `https://c7.hust-2slab.org` |
| MQTT broker (direct IP) | Server's real IP, port 1883 | `mqtt://103.116.39.179:1883` |
| WebSocket URL | Same host, path fixed | `wss://c7.hust-2slab.org/api/ws/plugins/telemetry` |

> **Note:** If the server is behind Cloudflare, HTTPS/WSS must use the
> **hostname** (Cloudflare forwards it). MQTT must use the **direct IP**
> (Cloudflare does not forward TCP port 1883).

### 1b. Admin credentials

Used by the firmware and scripts to auto-discover node devices.

| Value | Where | Key |
|---|---|---|
| Admin email | ThingsBoard login email | `tenant@thingsboard.org` |
| Admin password | ThingsBoard login password | `tenant` |

### 1c. Gateway device access token

Used by MQTT clients to authenticate as the gateway device (auto-creates
leaf nodes on first publish).

**ThingsBoard → Devices → (your gateway device) → Manage credentials → Access token**

Example: `4o51ajerynq34mtosc26`

### 1d. Gateway device UUID

Used by the Next.js dashboard to subscribe to live telemetry via WebSocket.

**ThingsBoard → Devices → (your gateway device) → copy UUID from URL**

Example: `98656720-5b36-11f1-82ae-b1b32c7b1fa7`

---

## 2. Node.js scripts — `.env.local`

Applies to: `test-direct-stream.js`, `test-mqtt-stream.js`, `test-mqtt-tb.js`, `test-http-stream.js`

Create `health-monitor/.env.local`:

```env
# ThingsBoard server (HTTPS via Cloudflare)
TB_BASE_URL=https://<YOUR_TB_HOST>
TB_USERNAME=<ADMIN_EMAIL>
TB_PASSWORD=<ADMIN_PASSWORD>

# Gateway device
TB_GATEWAY_ACCESS_TOKEN=<GATEWAY_ACCESS_TOKEN>
TB_DEVICE_ID=<GATEWAY_DEVICE_UUID>

# MQTT broker — direct IP bypasses Cloudflare (port 1883 must be open)
TB_MQTT_BROKER=mqtt://<SERVER_DIRECT_IP>:1883
```

Run any script with:
```bash
node scripts/test-direct-stream.js
# or
node --env-file=.env.local scripts/test-direct-stream.js
```

---

## 3. Next.js web app — `.env.local` (full)

Create `health-monitor/.env.local` (extends section 2 with public vars):

```env
# Server-side (Next.js API routes)
TB_BASE_URL=https://<YOUR_TB_HOST>
TB_USERNAME=<ADMIN_EMAIL>
TB_PASSWORD=<ADMIN_PASSWORD>
TB_GATEWAY_ACCESS_TOKEN=<GATEWAY_ACCESS_TOKEN>
TB_DEVICE_ID=<GATEWAY_DEVICE_UUID>

# MQTT broker (scripts only — not used by Next.js itself)
TB_MQTT_BROKER=mqtt://<SERVER_DIRECT_IP>:1883

# Browser-side (baked into build — must redeploy after changing)
NEXT_PUBLIC_TB_BASE_URL=https://<YOUR_TB_HOST>
NEXT_PUBLIC_TB_DEVICE_ID=<GATEWAY_DEVICE_UUID>
NEXT_PUBLIC_TB_WS_URL=wss://<YOUR_TB_HOST>/api/ws/plugins/telemetry
```

For Vercel deployment, set the same keys in **Settings → Environment Variables**.
After changing any `NEXT_PUBLIC_*` variable, trigger a new deployment (rebuild).

---

## 4. ESP32 firmware — `firmware/src/main.cpp`

Two credentials are hardcoded at the top of the file; two are entered via
the captive portal on first boot.

### 4a. Hardcoded in source (edit before flashing)

```cpp
// Line ~33 — MQTT broker direct IP
const char* TB_HOST = "<SERVER_DIRECT_IP>";

// Line ~118 — Admin API hostname (HTTPS via Cloudflare)
static const char* TB_ADMIN_HOST = "<YOUR_TB      _HOST>";

// Line ~37 — Gateway access token
#define TB_GATEWAY_TOKEN "<GATEWAY_ACCESS_TOKEN>"
```

### 4b. Entered via captive portal (stored in NVS, survive reflash)

On first boot (or after clearing NVS) the ESP32 opens a Wi-Fi access point
named **HealthMonitor-Setup**. Connect to it and open `http://192.168.4.1`:

| Field | Value |
|---|---|
| WiFi SSID | Your network name |
| WiFi Password | Your network password |
| ThingsBoard Admin Email | `<ADMIN_EMAIL>` |
| ThingsBoard Admin Password | `<ADMIN_PASSWORD>` |

To force re-entry of the captive portal, erase NVS:
```bash
# PlatformIO terminal
pio run -t erase
```

---

## 5. Python BLE gateway — `scripts/gateway.py`

Reads from `.env.local` (same file as section 2) plus two BLE-specific vars:

```env
# Add to .env.local:
TB_MQTT_BROKER=mqtt://<SERVER_DIRECT_IP>:1883
TB_GATEWAY_ACCESS_TOKEN=<GATEWAY_ACCESS_TOKEN>

# Multi-node (preferred): comma-separated "NodeName:BLE_ADDR" pairs
NODE_LIST=Node1:e5:39:e6:e4:d1:e8,Node2:aa:bb:cc:dd:ee:ff

# Single-node fallback (legacy — used if NODE_LIST is not set):
TB_NODE_NAME=Node1
BLE_ADDRESS=e5:39:e6:e4:d1:e8
```

Install dependencies:
```bash
pip install paho-mqtt simplepyble
```

Run:
```bash
python scripts/gateway.py
```

---

## 6. Quick reference — all values in one place

| Key | Used by | Where to get |
|---|---|---|
| `TB_BASE_URL` | Scripts, Next.js API | Your TB deployment URL |
| `TB_USERNAME` | Scripts, Next.js API, firmware portal | TB tenant admin email |
| `TB_PASSWORD` | Scripts, Next.js API, firmware portal | TB tenant admin password |
| `TB_GATEWAY_ACCESS_TOKEN` | Scripts, firmware, gateway.py | TB → Devices → gateway → Manage credentials |
| `TB_DEVICE_ID` | Next.js dashboard | TB → Devices → gateway → UUID in URL |
| `TB_MQTT_BROKER` | Scripts, gateway.py | `mqtt://<DIRECT_IP>:1883` |
| `NEXT_PUBLIC_TB_BASE_URL` | Browser UI | Same as `TB_BASE_URL` |
| `NEXT_PUBLIC_TB_DEVICE_ID` | Browser UI | Same as `TB_DEVICE_ID` |
| `NEXT_PUBLIC_TB_WS_URL` | Browser WebSocket | `wss://<TB_HOST>/api/ws/plugins/telemetry` |
| `TB_HOST` (firmware) | ESP32 MQTT | Direct server IP |
| `TB_ADMIN_HOST` (firmware) | ESP32 admin API | TB hostname (no `https://`) |
| `TB_GATEWAY_TOKEN` (firmware) | ESP32 MQTT auth | Same as `TB_GATEWAY_ACCESS_TOKEN` |
| WiFi SSID / Password | ESP32 captive portal | Your local network |
| `BLE_ADDRESS` | gateway.py | nRF52832 MAC (from BLE scan) |
| `TB_NODE_NAME` | gateway.py | Device name in ThingsBoard |
