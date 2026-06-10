# Payload & Config Reference

Cross-platform contract for the HealthMonitor pipeline.
`gateway.py` and `main.cpp` are the two canonical gateway implementations — all payload formats must match between them.

---

## 1. System architecture

Two gateway implementations are interchangeable — both expose the same interface to the
nRF52832 nodes and to ThingsBoard. Only the internal transport differs.

```
nRF52832 node(s)
      │  BLE notify  (ECG / vitals)
      │  BLE write   (config commands)
      ▼
┌──────────────────────────────────┐  ┌──────────────────────────────────┐
│  Gateway A                       │  │  Gateway B                       │
│  gateway.py                      │  │  nRF52832 central + ESP32        │
│                                  │  │  (firmware/src/main.cpp +        │
│  Direct BLE ↔ MQTT               │  │   nrf52_gateway/main.c)          │
│                                  │  │                                  │
│  Config: MQTT subscribe          │  │  BLE → UART → ESP32 → MQTT       │
│  (TB attribute push)             │  │  Config: HTTPS poll every 3 s    │
│                                  │  │  (per-node device token)         │
└──────────────┬───────────────────┘  └───────────────┬──────────────────┘
               │  MQTT  v1/gateway/telemetry           │  MQTT  v1/gateway/telemetry
               └───────────────────┬───────────────────┘
                                   ▼
                           ThingsBoard  (103.116.39.179)
                                   │  WebSocket + REST
                                   ▼
                          Next.js dashboard  (wearable-server.vercel.app)
```

Both gateways implement the same data contract:

| Behaviour | Gateway A | Gateway B |
|---|---|---|
| ECG dispatch | `len % 2 == 0 && len >= 2` | nRF central: `hvx->len % 2 == 0 && hvx->len >= 2` |
| Vitals dispatch | `len == 5` | nRF central: `hvx->len == 5` |
| Sample count | N = len / 2, default 50, max 100 | N = UART DATA len / 2, default 50, max 100 |
| MQTT chunk rule | ≤ 100 samples → 1 msg; > 100 → split, ts +1 ms/chunk | identical |
| Config delivery | BLE write direct to RX char | UART TYPE 0x03–0x06 → nRF central → BLE write |
| Config source | MQTT attribute push (TB SHARED_SCOPE) | HTTPS poll every 3 s (TB SHARED_SCOPE) |
| Commands sent | CMD_THR, CMD_ECG_CFG, CMD_PPG_CFG, CMD_VITAL_CFG | identical set |
| BLE command bytes | raw CMD_* bytes (section 7) | identical — same bytes forwarded by central |

---

## 2. MQTT telemetry

Both gateways publish to the same broker and topic.

| Platform | Transport | Topic | Auth |
|---|---|---|---|
| `gateway.py` | MQTT TCP | `v1/gateway/telemetry` | username = `TB_GATEWAY_ACCESS_TOKEN` |
| `main.cpp` (ESP32) | MQTT TCP | `v1/gateway/telemetry` | username = `TB_GATEWAY_TOKEN` |
| `test-*.js` scripts | MQTT TCP | `v1/gateway/telemetry` | username = `TB_GATEWAY_ACCESS_TOKEN` |

---

## 3. ECG waveform payload

Each MQTT message carries at most `PUBLISH_CHUNK` (100) samples. Batches larger than 100
are split into multiple messages with `ts` incremented by 1 ms per chunk so ThingsBoard
preserves ordering.

```json
{
  "NodeName": [
    {
      "ts": 1700000000000,
      "values": {
        "ecg_batch": "[v0,v1,...,vN-1]"
      }
    }
  ]
}
```

| Field | Type | Notes |
|---|---|---|
| `NodeName` | string key | ThingsBoard device name, e.g. `"Node1"` |
| `ts` | integer | Unix epoch milliseconds; +1 ms per chunk for multi-chunk batches |
| `ecg_batch` | **JSON-encoded string** | N `int16` samples; N = 1–100 (default 50, max 100 per BLE notify) |

### `gateway.py`
```python
# on_notify — size-agnostic dispatch (vitals is 5 bytes, always odd, never collides with ECG)
if len(data) == VITALS_SIZE:          # 5 bytes: [hrEcg u8][hrPpg u8][spo2 u8][temp u16 LE x10]
    ecg_hr, ppg_hr, spo2, temp_x10 = struct.unpack_from('<3BH', data)
    temp = temp_x10 / 10.0
    publish_q.put_nowait(('vitals', node.name, (ecg_hr, ppg_hr, spo2, temp)))
elif len(data) >= 2 and len(data) % 2 == 0:   # N × int16 LE ECG
    n = len(data) // 2
    samples = list(struct.unpack_from(f'<{n}h', data))
    publish_q.put_nowait(('ecg', node.name, samples))

# publish_worker — ECG branch with chunk splitting
_, node_name, samples = item
ts = int(time.time() * 1000)
for ci, offset in enumerate(range(0, len(samples), PUBLISH_CHUNK)):
    chunk = samples[offset:offset + PUBLISH_CHUNK]
    chunk_payload = { node_name: [{'ts': ts + ci,
                                    'values': {'ecg_batch': json.dumps(chunk)}}] }
    mqtt.publish('v1/gateway/telemetry', json.dumps(chunk_payload))
```

### `main.cpp` (ESP32)
```cpp
// UART RX: N × int16_t LE (N = pktDataLen/2, max BATCH_SIZE=100) → nodeBatch[idx][]
// publishEcgBatch() — splits into PUBLISH_CHUNK (100) sample chunks
int batchSz = nodeBatchSize[idx] > 0 ? nodeBatchSize[idx] : BATCH_SIZE;
int nChunks  = (batchSz + PUBLISH_CHUNK - 1) / PUBLISH_CHUNK;
for (int ci = 0; ci < nChunks; ci++) {
    int start = ci * PUBLISH_CHUNK, end = min(start + PUBLISH_CHUNK, batchSz);
    unsigned long long chunkTs = ts + (unsigned long long)ci;
    snprintf(payload, ..., "{\"%s\":[{\"ts\":%llu,\"values\":{\"ecg_batch\":\"[",
             name.c_str(), chunkTs);
    for (int i = start; i < end; i++) { ... snprintf("%d", nodeBatch[idx][i]); }
    snprintf(payload, ..., "]\"}}]}");
    mqttClient.publish("v1/gateway/telemetry", (uint8_t*)payload, pos);
}
```

---

## 4. Vitals payload

Published by both gateways. `gateway.py` receives vitals directly via BLE notify (5-byte packet); `main.cpp` receives them via UART TYPE 0x02 from the nRF central.

**BLE/UART wire format — 5 bytes:**

```
[hrEcg u8][hrPpg u8][spo2 u8][temp u16 LE x10]
```

| Byte(s) | Field | Notes |
|---|---|---|
| 0 | `hrEcg` | uint8, bpm |
| 1 | `hrPpg` | uint8, bpm |
| 2 | `spo2` | uint8, % |
| 3–4 | `temp` | uint16 LE, °C × 10 (e.g. 368 = 36.8°C) |

```json
{
  "NodeName": [
    {
      "ts": 1700000000000,
      "values": {
        "ecgHeartRate":  88,
        "ppgHeartRate":  87,
        "spo2":          98,
        "temperature":   36.8
      }
    }
  ]
}
```

| Key | Unit | Type | Notes |
|---|---|---|---|
| `ecgHeartRate` | bpm | integer | — |
| `ppgHeartRate` | bpm | integer | — |
| `spo2` | % | integer | — |
| `temperature` | °C | float, 1 decimal | decoded from uint16 × 10 |

```python
# gateway.py — publish_worker vitals branch
# BLE notify: 5 bytes: [hrEcg u8][hrPpg u8][spo2 u8][temp u16 LE x10]
_, node_name, (ecg_hr, ppg_hr, spo2, temp) = item   # item[0] == 'vitals'
payload = { node_name: [{ 'ts': int(time.time()*1000),
                           'values': { 'ecgHeartRate': ecg_hr,
                                       'ppgHeartRate': ppg_hr,
                                       'spo2':         spo2,
                                       'temperature':  round(temp, 1) } }] }
mqtt.publish('v1/gateway/telemetry', json.dumps(payload))
```

```cpp
// main.cpp — publishVitalPacket()
// UART RX: 5 bytes: [hrEcg u8][hrPpg u8][spo2 u8][temp u16 LE x10]
snprintf(payload, ...,
  "{\"%s\":[{\"ts\":%llu,\"values\":{"
  "\"ecgHeartRate\":%.0f,\"ppgHeartRate\":%.0f,"
  "\"spo2\":%.0f,\"temperature\":%.1f}}]}",
  name, ts, nodeHr[i], nodePpgHr[i], nodeSpo2[i], nodeTemp[i]);
mqttClient.publish("v1/gateway/telemetry", payload, len);
```

---

## 5. BLE UUIDs  (nRF52832 peripheral)

| UUID | Direction | Description |
|---|---|---|
| `6e401400-b5a3-f393-e0a9-e50e24dcca9e` | — | Primary service |
| `6e401401-b5a3-f393-e0a9-e50e24dcca9e` | node → gateway (notify) | ECG / vitals data TX |
| `6e401402-b5a3-f393-e0a9-e50e24dcca9e` | gateway → node (write) | Config / command RX |

---

## 6. BLE command sequence  (gateway → node)

The SoftDevice `conn_handle` on the central uniquely identifies each connection, so no
`node_id` byte is needed in the payload. Commands are routed by which connection they are
written to, not by an embedded identifier.

All commands start with a single command byte followed directly by the payload:

```
[CMD]  [...payload bytes]
 1 B        N bytes
```

### Command table

| CMD | Name | Total bytes | Payload after `[CMD]` |
|---|---|---|---|
| `0xCF` | `CMD_ECG_CFG`   |  5 | `[freq_lo][freq_hi][interval_lo][interval_hi]` (2 × uint16 LE) |
| `0xCE` | `CMD_THR`       | 31 | see expanded layout below |
| `0xCD` | `CMD_PPG_CFG`   |  5 | `[sampleFreqLo][sampleFreqHi][redMa][irMa]` (uint16 LE + 2 × uint8) |
| `0xCC` | `CMD_VITAL_CFG` |  3 | `[intervalLo][intervalHi]` (uint16 LE, ms) |

### CMD_ECG_CFG  `0xCF`  — ECG sampling config

```
[0xCF][freq_lo][freq_hi][interval_lo][interval_hi]
```

| Byte(s) | Field | Example |
|---|---|---|
| 0 | `0xCF` | — |
| 1–2 | `freq_hz` uint16 LE | `0xFA 0x00` = 250 Hz |
| 3–4 | `interval_ms` uint16 LE | `0xC8 0x00` = 200 ms |

```python
# gateway.py
struct.pack('<B2H', CMD_ECG_CFG, freq_hz, interval_ms)
```
```cpp
// main.cpp — sendUartConfig() → wraps in UART TYPE 0x03
uint8_t data[5] = { 0xCF,
  (uint8_t)(freq & 0xFF), (uint8_t)(freq >> 8),
  (uint8_t)(interval & 0xFF), (uint8_t)(interval >> 8) };
```
```c
// nRF52832 RX handler (cmd.c)
if (data[0] == 0xCF && len == 5) {
    uint16_t freq_hz     = data[1] | ((uint16_t)data[2] << 8);
    uint16_t interval_ms = data[3] | ((uint16_t)data[4] << 8);
}
```

### CMD_THR  `0xCE`  — vital thresholds (all 3 tiers)

**31 bytes total** — 3 tiers (normal / warning / dangerous) × 4 vitals × min+max.  
Temperature is encoded as `uint16 LE × 10` to preserve 0.1 °C resolution.

```
[0]   0xCE
[1]   ppgHr_normalMin   [2]  ppgHr_normalMax    (uint8 bpm)
[3]   ppgHr_warnMin     [4]  ppgHr_warnMax
[5]   ppgHr_dangerMin   [6]  ppgHr_dangerMax
[7]   ecgHr_normalMin   [8]  ecgHr_normalMax    (uint8 bpm)
[9]   ecgHr_warnMin     [10] ecgHr_warnMax
[11]  ecgHr_dangerMin   [12] ecgHr_dangerMax
[13]  spo2_normalMin    [14] spo2_normalMax      (uint8 %)
[15]  spo2_warnMin      [16] spo2_warnMax
[17]  spo2_dangerMin    [18] spo2_dangerMax
[19-20] temp_normalMin  [21-22] temp_normalMax   (uint16 LE ×10, e.g. 361 = 36.1°C)
[23-24] temp_warnMin    [25-26] temp_warnMax
[27-28] temp_dangerMin  [29-30] temp_dangerMax
```

```python
# gateway.py — NodeState.build_threshold_payload()
struct.pack('<B18B6H',
    CMD_THR,
    ppgHr_normalMin, ppgHr_normalMax, ppgHr_warnMin, ppgHr_warnMax, ppgHr_dangerMin, ppgHr_dangerMax,
    ecgHr_normalMin, ecgHr_normalMax, ecgHr_warnMin, ecgHr_warnMax, ecgHr_dangerMin, ecgHr_dangerMax,
    spo2_normalMin,  spo2_normalMax,  spo2_warnMin,  spo2_warnMax,  spo2_dangerMin,  spo2_dangerMax,
    temp_normalMin,  temp_normalMax,  temp_warnMin,  temp_warnMax,  temp_dangerMin,  temp_dangerMax,
    # temp values stored ×10: 36.1°C → 361
)
```
```c
// nRF52832 RX handler (cmd.c)
if (data[0] == 0xCE && len == 31) {
    g_thr_ppg_norm_min = data[1];  g_thr_ppg_norm_max = data[2];
    g_thr_ppg_warn_min = data[3];  g_thr_ppg_warn_max = data[4];
    g_thr_ppg_dang_min = data[5];  g_thr_ppg_dang_max = data[6];
    g_thr_ecg_norm_min = data[7];  g_thr_ecg_norm_max = data[8];
    g_thr_ecg_warn_min = data[9];  g_thr_ecg_warn_max = data[10];
    g_thr_ecg_dang_min = data[11]; g_thr_ecg_dang_max = data[12];
    g_thr_spo2_norm_min= data[13]; g_thr_spo2_norm_max= data[14];
    g_thr_spo2_warn_min= data[15]; g_thr_spo2_warn_max= data[16];
    g_thr_spo2_dang_min= data[17]; g_thr_spo2_dang_max= data[18];
    g_thr_temp_norm_min= (uint16_t)(data[19] | (data[20]<<8));
    g_thr_temp_norm_max= (uint16_t)(data[21] | (data[22]<<8));
    g_thr_temp_warn_min= (uint16_t)(data[23] | (data[24]<<8));
    g_thr_temp_warn_max= (uint16_t)(data[25] | (data[26]<<8));
    g_thr_temp_dang_min= (uint16_t)(data[27] | (data[28]<<8));
    g_thr_temp_dang_max= (uint16_t)(data[29] | (data[30]<<8));
}
```

---

## 7. BLE write commands  (gateway → nRF52832 peripheral)  — raw bytes on RX char

Written directly to the RX characteristic (`6e401402-b5a3-f393-e0a9-e50e24dcca9e`) via
`peripheral.write_request()`. No transport framing — raw bytes only.

| CMD | Total bytes | Byte layout |
|---|---|---|
| `CMD_THR (0xCE)` | 31 | `[0xCE][18×uint8 PPG/ECG/SpO2 norm/warn/dang min+max][6×uint16LE temp×10]` |
| `CMD_ECG_CFG (0xCF)` | 5 | `[0xCF][freq_lo][freq_hi][interval_lo][interval_hi]` |
| `CMD_PPG_CFG (0xCD)` | 5 | `[0xCD][sampleFreqLo][sampleFreqHi][redMa][irMa]` |
| `CMD_VITAL_CFG (0xCC)` | 3 | `[0xCC][intervalLo][intervalHi]` |

---

## 8. UART binary framing  (ESP32 ↔ nRF52832 central)  — Path B only

```
[0xAA][0x55][TYPE][NAME_LEN][NAME...][LEN_LO][LEN_HI][DATA...][XOR_CHK]
```

| Byte(s) | Field | Notes |
|---|---|---|
| `0xAA 0x55` | Magic | Start-of-frame |
| `TYPE` | Packet type | See table |
| `NAME_LEN` | 1 byte | Length of ASCII node name |
| `NAME...` | `NAME_LEN` bytes | e.g. `Node1` |
| `LEN_LO LEN_HI` | 2 bytes LE | DATA length in bytes |
| `DATA...` | `LEN` bytes | Type-specific (same BLE command bytes) |
| `XOR_CHK` | 1 byte | XOR of all bytes from `TYPE` through last `DATA` byte |

| TYPE | Direction | DATA | Description |
|---|---|---|---|
| `0x01` | nRF→ESP32 | 2N bytes: N × `int16_t` LE (N = freq × interval / 1000, default 50, max 100) | ECG batch — size varies with config |
| `0x02` | nRF→ESP32 | 5 bytes: `[hrEcg u8][hrPpg u8][spo2 u8][temp u16 LE x10]` | Vitals |
| `0x03` | ESP32→nRF | 5 bytes: `CMD_ECG_CFG` frame | ECG config → forwarded to BLE RX char |
| `0x04` | ESP32→nRF | 31 bytes: `CMD_THR` frame | Thresholds → forwarded to BLE RX char |
| `0x05` | ESP32→nRF | 5 bytes: `CMD_PPG_CFG` frame | PPG config → forwarded to BLE RX char |
| `0x06` | ESP32→nRF | 3 bytes: `CMD_VITAL_CFG` frame | Vital interval → forwarded to BLE RX char |

The nRF52832 central forwards the DATA bytes of TYPE 0x03 / 0x04 directly to the target node's RX characteristic, matched by the NAME field.  
The DATA bytes are **identical** to the raw BLE write payloads in section 7 — Path A and Path B deliver the same bytes to the nRF52832 peripheral, ensuring full command parity.

---

## 9. ThingsBoard shared attributes  (SHARED_SCOPE per node)

Written by the dashboard (`pages/settings.js`). Read by gateways and the dashboard to configure the device.

The complete uplink payload (all keys saved on every settings save):

```json
{
  "vitalInterval": 1000,
  "ecgSampleFreq": 250, "ecgPacketInterval": 500,
  "ppgSampleFreq": 100, "ppgRedLedMa": 6, "ppgIrLedMa": 6,
  "ppgHr_normalMin": 60,  "ppgHr_normalMax": 100,
  "ppgHr_warnMin":   50,  "ppgHr_warnMax":  120,
  "ppgHr_dangerMin": 40,  "ppgHr_dangerMax": 130,
  "ecgHr_normalMin": 60,  "ecgHr_normalMax": 100,
  "ecgHr_warnMin":   50,  "ecgHr_warnMax":  120,
  "ecgHr_dangerMin": 40,  "ecgHr_dangerMax": 130,
  "spo2_normalMin":  95,  "spo2_normalMax":  100,
  "spo2_warnMin":    90,  "spo2_warnMax":    100,
  "spo2_dangerMin":  88,  "spo2_dangerMax":  100,
  "temp_normalMin":  36.1,"temp_normalMax":  37.2,
  "temp_warnMin":    35.5,"temp_warnMax":    38.5,
  "temp_dangerMin":  35.0,"temp_dangerMax":  39.5
}
```

### Key reference

| Key | Default | Description | gateway.py | main.cpp |
|---|---|---|---|---|
| `bleAddress` | from `.env.local` | nRF52832 BLE MAC | reconnect | — |
| `vitalInterval` | `1000` ms | How often the node reports vitals | — | configures timer |
| `ecgSampleFreq` | `250` Hz | ADC sampling rate | — | `CMD_ECG_CFG` |
| `ecgPacketInterval` | `500` ms | BLE notify interval | — | `CMD_ECG_CFG` |
| `ppgSampleFreq` | `100` Hz | MAX30102 sample rate | — | configures sensor |
| `ppgRedLedMa` | `6` mA | Red LED drive current | — | configures sensor |
| `ppgIrLedMa` | `6` mA | IR LED drive current | — | configures sensor |
| `ppgHr_normalMin/Max` | 60 / 100 bpm | PPG HR normal band | `CMD_THR` bytes 1–2 | `CMD_THR` |
| `ppgHr_warnMin/Max` | 50 / 120 bpm | PPG HR warning band | `CMD_THR` bytes 3–4 | `CMD_THR` |
| `ppgHr_dangerMin/Max` | 40 / 130 bpm | PPG HR danger band | `CMD_THR` bytes 5–6 | `CMD_THR` |
| `ecgHr_normalMin/Max` | 60 / 100 bpm | ECG HR normal band | `CMD_THR` bytes 7–8 | `CMD_THR` |
| `ecgHr_warnMin/Max` | 50 / 120 bpm | ECG HR warning band | `CMD_THR` bytes 9–10 | `CMD_THR` |
| `ecgHr_dangerMin/Max` | 40 / 130 bpm | ECG HR danger band | `CMD_THR` bytes 11–12 | `CMD_THR` |
| `spo2_normalMin/Max` | 95 / 100 % | SpO₂ normal band | `CMD_THR` bytes 13–14 | `CMD_THR` |
| `spo2_warnMin/Max` | 90 / 100 % | SpO₂ warning band | `CMD_THR` bytes 15–16 | `CMD_THR` |
| `spo2_dangerMin/Max` | 88 / 100 % | SpO₂ danger band | `CMD_THR` bytes 17–18 | `CMD_THR` |
| `temp_normalMin/Max` | 36.1 / 37.2 °C | Temp normal band (→ ×10: 361/372) | `CMD_THR` bytes 19–22 | `CMD_THR` |
| `temp_warnMin/Max` | 35.5 / 38.5 °C | Temp warning band (→ ×10: 355/385) | `CMD_THR` bytes 23–26 | `CMD_THR` |
| `temp_dangerMin/Max` | 35.0 / 39.5 °C | Temp danger band (→ ×10: 350/395) | `CMD_THR` bytes 27–30 | `CMD_THR` |

> All 24 threshold keys are forwarded by `gateway.py` via a single `CMD_THR` write on every push.  
> Temperature is stored in TB as float (°C) and packed as `uint16 LE × 10` in the BLE frame — `round(v * 10)` in the gateway, divide by 10 in firmware.

### How gateway.py receives attribute changes

On MQTT connect:
1. Sends `v1/gateway/connect` for each node so TB routes attribute pushes to this gateway.
2. Requests current values for `bleAddress` + all threshold/config keys.

TB pushes live changes whenever the settings page saves to `v1/gateway/attributes`:
```json
{"device": "Node1", "data": {"ppgHr_warnMin": 45, "temp_warnMax": 39, ...}}
```

`mqtt_on_message` routes the update to the correct `NodeState` by device name, updates in-memory thresholds, and enqueues a `CMD_THR` BLE write.

### How main.cpp receives attribute changes

`configSyncTask` polls every 3 s via HTTPS using the per-node device access token:
```
GET /api/v1/{node_token}/attributes?sharedKeys=ecgSampleFreq,ecgPacketInterval,...
```
On change, sends a UART TYPE 0x03 / 0x04 frame to the nRF52832 central.

---

## 9. gateway.py multi-node config

```
NODE_LIST=Node1:e5:39:e6:e4:d1:e8,Node2:aa:bb:cc:dd:ee:ff
```

- Each entry is `NodeName:BLE_ADDR`. Split on the first `:` only (MAC has its own colons).
- Falls back to `TB_NODE_NAME` + `BLE_ADDRESS` if `NODE_LIST` is not set.

One BLE worker thread per node; all share one MQTT connection and one `publish_q`.
A `_scan_lock` ensures only one thread calls `adapter.scan_for()` at a time.

---

## 10. Batch timing reference

| Parameter | Value | Source |
|---|---|---|
| Sample rate | 250 Hz | `ecgSampleFreq` (default) |
| Packet interval | 200 ms firmware default / 500 ms TB default | `ecgPacketInterval` shared attribute |
| Batch size (default) | 50 samples | 250 Hz × 200 ms / 1000 |
| Batch size (max notify) | 100 samples | `ECG_MAX_SAMPLES` — BLE ATT limit ~238 B, 100 × 2 = 200 B |
| Batch size (max config) | 500 samples | `ECG_BUF_SAMPLES` — node accumulates then splits into multiple notifies |
| BLE payload | 2N bytes per notify, N ≤ `ECG_MAX_SAMPLES` (100) | batches > 100 are split into multiple consecutive BLE notifications |
| MQTT chunk size | 100 samples (`PUBLISH_CHUNK`) | each BLE notify ≤ 100 → 1 MQTT message (already at notify boundary) |

`batch_size = floor(freq_hz × interval_ms / 1000)` — accepted up to `ECG_BUF_SAMPLES` (500) by firmware.  
Batches > `ECG_MAX_SAMPLES` are split by the node into consecutive BLE notifications of ≤ 100 samples each.  
Each notification is published independently as one MQTT message by the gateway.

**Example — 500 ms interval at 250 Hz**: 125 samples → 2 BLE notifies (100 + 25) → 2 MQTT messages.  
**Example — 200 ms interval at 250 Hz**: 50 samples → 1 BLE notify → 1 MQTT message.

Gateways dispatch ECG by `len % 2 == 0 && len != 16` (vitals check first) and derive sample count from `len / 2`, so they handle any batch size without code changes.
