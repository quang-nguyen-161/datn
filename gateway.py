"""
gateway.py — Multi-node BLE ECG gateway: nRF52832 -> ThingsBoard via MQTT

Connects to one or more nRF52832 BLE nodes concurrently.
Each node runs its own BLE worker thread; all share one MQTT connection.

Config (env vars or .env.local):
  TB_MQTT_BROKER          mqtt://103.116.39.179:1883
  TB_GATEWAY_ACCESS_TOKEN 4o51ajerynq34mtosc26

  # Multi-node (preferred): "NodeName:BLE_ADDR" pairs, comma-separated
  NODE_LIST               Node1:e5:39:e6:e4:d1:e8,Node2:aa:bb:cc:dd:ee:ff

  # Single-node fallback (legacy):
  TB_NODE_NAME            Node1
  BLE_ADDRESS             e5:39:e6:e4:d1:e8

Install deps: pip install paho-mqtt simplepyble
"""

import os
import json
import struct
import time
import queue
import threading
from urllib.parse import urlparse

import simplepyble
import paho.mqtt.client as mqtt_client


# ── Load .env.local ───────────────────────────────────────────────────
def load_env(path):
    try:
        with open(path, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                eq = line.find('=')
                if eq < 0:
                    continue
                k, v = line[:eq].strip(), line[eq + 1:].strip()
                if k and k not in os.environ:
                    os.environ[k] = v
    except FileNotFoundError:
        pass

load_env('.env.local')
load_env('../.env.local')

# ── Config ────────────────────────────────────────────────────────────
BROKER_URL   = os.environ.get('TB_MQTT_BROKER', 'mqtt://103.116.39.179:1883')
ACCESS_TOKEN = os.environ.get('TB_GATEWAY_ACCESS_TOKEN', '')
NODE_LIST_ENV = os.environ.get('NODE_LIST', '')          # "Node1:e5:..,Node2:aa:.."
TB_NODE_NAME  = os.environ.get('TB_NODE_NAME', 'Node1')  # legacy single-node
BLE_ADDRESS   = os.environ.get('BLE_ADDRESS',  'e5:39:e6:e4:d1:e8')

SERVICE_UUID        = "6e401400-b5a3-f393-e0a9-e50e24dcca9e"
CHARACTERISTIC_UUID = "6e401401-b5a3-f393-e0a9-e50e24dcca9e"
RX_CHAR_UUID        = "6e401402-b5a3-f393-e0a9-e50e24dcca9e"

PACKET_SAMPLES    = 50       # ECG: 50 × int16 LE = 100 bytes (default batch)
VITALS_SIZE       = 16       # Vitals: 4 × float32 LE = 16 bytes
PUBLISH_CHUNK     = 100      # max samples per MQTT message; mirrors BLE max payload — only batches >100 split
NO_DATA_TIMEOUT_S = 6

_parsed   = urlparse(BROKER_URL)
MQTT_HOST = _parsed.hostname or '103.116.39.179'
MQTT_PORT = _parsed.port or 1883

MQTT_TOPIC      = 'v1/gateway/telemetry'
ATTR_REQ_TOPIC  = 'v1/gateway/attributes/request'
ATTR_RESP_TOPIC = 'v1/gateway/attributes/response'
ATTR_PUSH_TOPIC = 'v1/gateway/attributes'

CMD_ECG_CFG   = 0xCF   # ECG config:      [0xCF][fLo][fHi][iLo][iHi]              5 bytes
CMD_THR       = 0xCE   # vital thresholds:[0xCE][18×uint8 PPG/ECG/SpO2][6×uint16LE temp×10]  31 bytes
CMD_PPG_CFG   = 0xCD   # PPG config:      [0xCD][fLo][fHi][redMa][irMa]           5 bytes
CMD_VITAL_CFG = 0xCC   # Vital interval:  [0xCC][intervalLo][intervalHi]           3 bytes

THRESHOLD_KEYS = [
    'ppgHr_normalMin', 'ppgHr_normalMax', 'ppgHr_warnMin', 'ppgHr_warnMax', 'ppgHr_dangerMin', 'ppgHr_dangerMax',
    'ecgHr_normalMin', 'ecgHr_normalMax', 'ecgHr_warnMin', 'ecgHr_warnMax', 'ecgHr_dangerMin', 'ecgHr_dangerMax',
    'spo2_normalMin',  'spo2_normalMax',  'spo2_warnMin',  'spo2_warnMax',  'spo2_dangerMin',  'spo2_dangerMax',
    'temp_normalMin',  'temp_normalMax',  'temp_warnMin',  'temp_warnMax',  'temp_dangerMin',  'temp_dangerMax',
]
# Temperature keys are stored ×10 (uint16) to preserve 0.1°C resolution
TEMP_KEYS = frozenset(k for k in THRESHOLD_KEYS if k.startswith('temp_'))
ECG_CFG_KEYS   = ['ecgSampleFreq', 'ecgPacketInterval']
PPG_CFG_KEYS   = ['ppgSampleFreq', 'ppgRedLedMa', 'ppgIrLedMa']
VITAL_CFG_KEYS = ['vitalInterval']
ALL_SHARED_KEYS = (['bleAddress'] + THRESHOLD_KEYS
                   + ECG_CFG_KEYS + PPG_CFG_KEYS + VITAL_CFG_KEYS)

_DEFAULT_THRESHOLDS = {
    'ppgHr_normalMin': 60,  'ppgHr_normalMax': 100,
    'ppgHr_warnMin':   50,  'ppgHr_warnMax':   120,
    'ppgHr_dangerMin': 40,  'ppgHr_dangerMax': 130,
    'ecgHr_normalMin': 60,  'ecgHr_normalMax': 100,
    'ecgHr_warnMin':   50,  'ecgHr_warnMax':   120,
    'ecgHr_dangerMin': 40,  'ecgHr_dangerMax': 130,
    'spo2_normalMin':  95,  'spo2_normalMax':  100,
    'spo2_warnMin':    90,  'spo2_warnMax':    100,
    'spo2_dangerMin':  88,  'spo2_dangerMax':  100,
    # stored ×10 to preserve 0.1°C resolution in uint16
    'temp_normalMin':  361, 'temp_normalMax':  372,
    'temp_warnMin':    355, 'temp_warnMax':    385,
    'temp_dangerMin':  350, 'temp_dangerMax':  395,
}

_DEFAULT_ECG_CFG = {
    'ecgSampleFreq':     250,
    'ecgPacketInterval': 500,
}

_DEFAULT_PPG_CFG = {
    'ppgSampleFreq': 100,
    'ppgRedLedMa':   6,
    'ppgIrLedMa':    6,
}

_DEFAULT_VITAL_CFG = {
    'vitalInterval': 1000,
}


# ── Node state ────────────────────────────────────────────────────────

class NodeState:
    """All mutable per-node state, shared between the MQTT thread and the node's BLE worker."""

    def __init__(self, name: str, ble_address: str):
        self.name         = name
        self._addr        = ble_address.lower()
        self._addr_lock   = threading.Lock()
        self.addr_changed = threading.Event()
        self.cmd_q        = queue.Queue(maxsize=10)  # outbound BLE write payloads
        self.thresholds   = dict(_DEFAULT_THRESHOLDS)
        self._thr_lock    = threading.Lock()
        self.ecg_cfg      = dict(_DEFAULT_ECG_CFG)
        self._ecg_lock    = threading.Lock()
        self.ppg_cfg      = dict(_DEFAULT_PPG_CFG)
        self._ppg_lock    = threading.Lock()
        self.vital_cfg    = dict(_DEFAULT_VITAL_CFG)
        self._vital_lock  = threading.Lock()

    def get_address(self) -> str:
        with self._addr_lock:
            return self._addr

    def set_address(self, addr: str):
        addr = addr.strip().lower()
        if not addr:
            return
        with self._addr_lock:
            if addr != self._addr:
                print(f'[TB]  {self.name}: BLE address {self._addr} -> {addr}')
                self._addr = addr
                self.addr_changed.set()

    def update_thresholds(self, updates: dict) -> bool:
        changed = False
        with self._thr_lock:
            for k, v in updates.items():
                if k in self.thresholds:
                    try:
                        new = round(float(v) * 10) if k in TEMP_KEYS else int(float(v))
                        if new != self.thresholds[k]:
                            self.thresholds[k] = new
                            changed = True
                    except (TypeError, ValueError):
                        pass
        return changed

    def build_threshold_payload(self) -> bytes | None:
        """31 bytes: [CMD_THR][18×uint8 PPG/ECG/SpO2][6×uint16LE temp×10]"""
        with self._thr_lock:
            t = self.thresholds.copy()
        try:
            return struct.pack('<B18B6H',
                CMD_THR,
                t['ppgHr_normalMin'], t['ppgHr_normalMax'],
                t['ppgHr_warnMin'],   t['ppgHr_warnMax'],
                t['ppgHr_dangerMin'], t['ppgHr_dangerMax'],
                t['ecgHr_normalMin'], t['ecgHr_normalMax'],
                t['ecgHr_warnMin'],   t['ecgHr_warnMax'],
                t['ecgHr_dangerMin'], t['ecgHr_dangerMax'],
                t['spo2_normalMin'],  t['spo2_normalMax'],
                t['spo2_warnMin'],    t['spo2_warnMax'],
                t['spo2_dangerMin'],  t['spo2_dangerMax'],
                t['temp_normalMin'],  t['temp_normalMax'],
                t['temp_warnMin'],    t['temp_warnMax'],
                t['temp_dangerMin'],  t['temp_dangerMax'],
            )
        except Exception as e:
            print(f'[THR] {self.name}: payload build error: {e}')
            return None

    def enqueue_thresholds(self):
        p = self.build_threshold_payload()
        if p:
            try:
                self.cmd_q.put_nowait(p)
            except queue.Full:
                pass

    def update_ecg_cfg(self, updates: dict) -> bool:
        changed = False
        with self._ecg_lock:
            for k, v in updates.items():
                if k in self.ecg_cfg:
                    try:
                        new = int(float(v))
                        if new != self.ecg_cfg[k]:
                            self.ecg_cfg[k] = new
                            changed = True
                    except (TypeError, ValueError):
                        pass
        return changed

    def build_ecg_cfg_payload(self) -> bytes | None:
        """[CMD_ECG_CFG][freq_lo][freq_hi][interval_lo][interval_hi] — 5 bytes."""
        with self._ecg_lock:
            cfg = self.ecg_cfg.copy()
        try:
            return struct.pack('<B2H',
                CMD_ECG_CFG,
                cfg['ecgSampleFreq'],
                cfg['ecgPacketInterval'],
            )
        except Exception as e:
            print(f'[ECG] {self.name}: payload build error: {e}')
            return None

    def enqueue_ecg_cfg(self):
        p = self.build_ecg_cfg_payload()
        if p:
            try:
                self.cmd_q.put_nowait(p)
            except queue.Full:
                pass

    def update_ppg_cfg(self, updates: dict) -> bool:
        changed = False
        with self._ppg_lock:
            for k, v in updates.items():
                if k in self.ppg_cfg:
                    try:
                        new = int(float(v))
                        if new != self.ppg_cfg[k]:
                            self.ppg_cfg[k] = new
                            changed = True
                    except (TypeError, ValueError):
                        pass
        return changed

    def build_ppg_cfg_payload(self) -> bytes | None:
        """[CMD_PPG_CFG][sampleFreqLo][sampleFreqHi][redMa][irMa] — 5 bytes."""
        with self._ppg_lock:
            cfg = self.ppg_cfg.copy()
        try:
            return struct.pack('<BH2B',
                CMD_PPG_CFG,
                cfg['ppgSampleFreq'],
                cfg['ppgRedLedMa'],
                cfg['ppgIrLedMa'],
            )
        except Exception as e:
            print(f'[PPG] {self.name}: payload build error: {e}')
            return None

    def enqueue_ppg_cfg(self):
        p = self.build_ppg_cfg_payload()
        if p:
            try:
                self.cmd_q.put_nowait(p)
            except queue.Full:
                pass

    def update_vital_cfg(self, updates: dict) -> bool:
        changed = False
        with self._vital_lock:
            for k, v in updates.items():
                if k in self.vital_cfg:
                    try:
                        new = int(float(v))
                        if new != self.vital_cfg[k]:
                            self.vital_cfg[k] = new
                            changed = True
                    except (TypeError, ValueError):
                        pass
        return changed

    def build_vital_cfg_payload(self) -> bytes | None:
        """[CMD_VITAL_CFG][intervalLo][intervalHi] — 3 bytes."""
        with self._vital_lock:
            cfg = self.vital_cfg.copy()
        try:
            return struct.pack('<BH',
                CMD_VITAL_CFG,
                cfg['vitalInterval'],
            )
        except Exception as e:
            print(f'[VIT] {self.name}: payload build error: {e}')
            return None

    def enqueue_vital_cfg(self):
        p = self.build_vital_cfg_payload()
        if p:
            try:
                self.cmd_q.put_nowait(p)
            except queue.Full:
                pass


def _parse_node_list() -> dict[str, NodeState]:
    """Build {name: NodeState} from NODE_LIST env or legacy single-node vars."""
    nodes: dict[str, NodeState] = {}
    if NODE_LIST_ENV:
        for entry in NODE_LIST_ENV.split(','):
            entry = entry.strip()
            if not entry:
                continue
            # "NodeName:BLE_ADDR" — split on first colon only; MAC has its own colons
            sep = entry.index(':')
            name = entry[:sep].strip()
            addr = entry[sep + 1:].strip()
            if name and addr:
                nodes[name] = NodeState(name, addr)
    if not nodes:
        nodes[TB_NODE_NAME] = NodeState(TB_NODE_NAME, BLE_ADDRESS)
    return nodes


# ── Shared publish queue (MQTT publish thread) ────────────────────────
# Items: ('ecg', node_name, samples_list) | ('vitals', node_name, (ecg_hr, ppg_hr, spo2, temp))
publish_q = queue.Queue(maxsize=100)

mqtt_connected        = threading.Event()
mqtt                  = None
_attr_req_id          = 0
_pending_attr_req_key  = {}   # req_id -> attribute key
_pending_attr_req_node = {}   # req_id -> node name

# Filled by _parse_node_list() before MQTT setup
nodes: dict[str, NodeState] = {}


def _connect_all_devices(client):
    for node_name in nodes:
        client.publish('v1/gateway/connect', json.dumps({"device": node_name, "type": "default"}))
    print(f'[TB]  Announced connect for {len(nodes)} node(s): {list(nodes)}')


def _request_all_attrs(client):
    global _attr_req_id
    for node_name in nodes:
        for key in ALL_SHARED_KEYS:
            _attr_req_id += 1
            _pending_attr_req_key[_attr_req_id]  = key
            _pending_attr_req_node[_attr_req_id] = node_name
            payload = json.dumps({"id": _attr_req_id, "device": node_name,
                                  "client": False, "key": key})
            client.publish(ATTR_REQ_TOPIC, payload)
    print(f'[TB]  Requested shared attrs for {len(nodes)} node(s): {list(nodes)}')


def mqtt_on_message(client, userdata, msg):
    try:
        data  = json.loads(msg.payload.decode())
        topic = msg.topic

        if topic == ATTR_RESP_TOPIC:
            req_id    = data.get('id')
            value     = data.get('value')
            key       = _pending_attr_req_key.pop(req_id, None)
            node_name = _pending_attr_req_node.pop(req_id, None)
            if key is None or value is None or node_name is None:
                return
            node = nodes.get(node_name)
            if node is None:
                return
            print(f'[TB]  {node_name}: attr response {key}={value}')
            if key == 'bleAddress':
                node.set_address(str(value))
            elif key in THRESHOLD_KEYS:
                if node.update_thresholds({key: value}):
                    node.enqueue_thresholds()
            elif key in ECG_CFG_KEYS:
                if node.update_ecg_cfg({key: value}):
                    node.enqueue_ecg_cfg()
            elif key in PPG_CFG_KEYS:
                if node.update_ppg_cfg({key: value}):
                    node.enqueue_ppg_cfg()
            elif key in VITAL_CFG_KEYS:
                if node.update_vital_cfg({key: value}):
                    node.enqueue_vital_cfg()

        elif topic == ATTR_PUSH_TOPIC:
            # {"device": "Node1", "data": {"bleAddress": "..", "ppgHr_warnMin": 50, ...}}
            node_name = data.get('device')
            print(f'[TB]  attr push for device={node_name!r}: {data}')
            node = nodes.get(node_name)
            if node is None:
                print(f'[TB]  attr push: unknown device {node_name!r} — known: {list(nodes)}')
                return
            updates = data.get('data', {})

            if 'bleAddress' in updates:
                node.set_address(str(updates['bleAddress']))

            thr_updates = {k: v for k, v in updates.items() if k in THRESHOLD_KEYS}
            if thr_updates:
                node.update_thresholds(thr_updates)
                node.enqueue_thresholds()
                print(f'[TB]  {node_name}: thresholds pushed {thr_updates} -> BLE write queued')

            ecg_updates = {k: v for k, v in updates.items() if k in ECG_CFG_KEYS}
            if ecg_updates:
                node.update_ecg_cfg(ecg_updates)
                node.enqueue_ecg_cfg()
                print(f'[TB]  {node_name}: ECG config pushed {ecg_updates} -> BLE write queued')

            ppg_updates = {k: v for k, v in updates.items() if k in PPG_CFG_KEYS}
            if ppg_updates:
                node.update_ppg_cfg(ppg_updates)
                node.enqueue_ppg_cfg()
                print(f'[TB]  {node_name}: PPG config pushed {ppg_updates} -> BLE write queued')

            vital_updates = {k: v for k, v in updates.items() if k in VITAL_CFG_KEYS}
            if vital_updates:
                node.update_vital_cfg(vital_updates)
                node.enqueue_vital_cfg()
                print(f'[TB]  {node_name}: vital config pushed {vital_updates} -> BLE write queued')

    except Exception as e:
        print(f'[MQTT] Message parse error: {e}')


def mqtt_on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f'[MQTT] Connected to {MQTT_HOST}:{MQTT_PORT}')
        client.subscribe(ATTR_RESP_TOPIC)
        client.subscribe(ATTR_PUSH_TOPIC)
        mqtt_connected.set()
        _connect_all_devices(client)
        _request_all_attrs(client)
    else:
        print(f'[MQTT] Connection failed rc={rc}')


def mqtt_on_disconnect(client, userdata, rc):
    mqtt_connected.clear()
    if rc != 0:
        print(f'[MQTT] Unexpected disconnect rc={rc} — reconnecting...')


def mqtt_setup():
    global mqtt
    mqtt = mqtt_client.Client(client_id=f'gateway-{int(time.time())}',
                               protocol=mqtt_client.MQTTv311)
    mqtt.username_pw_set(ACCESS_TOKEN, '')
    mqtt.on_connect    = mqtt_on_connect
    mqtt.on_disconnect = mqtt_on_disconnect
    mqtt.on_message    = mqtt_on_message
    mqtt.connect_async(MQTT_HOST, MQTT_PORT, keepalive=30)
    mqtt.loop_start()


def publish_worker():
    mqtt_connected.wait(timeout=15)
    batch_num = 0
    while True:
        item = publish_q.get()
        if item is None:
            break
        pkt_type = item[0]
        ts = int(time.time() * 1000)

        if pkt_type == 'ecg':
            _, node_name, samples = item
            batch_num += 1
            if not mqtt_connected.is_set():
                print(f'[MQTT] Not connected — dropping {node_name} ECG batch #{batch_num}')
            else:
                # Split into chunks of PUBLISH_CHUNK samples; offset ts by 1 ms per chunk
                # so ThingsBoard preserves ordering when multiple messages land at once
                for ci, offset in enumerate(range(0, len(samples), PUBLISH_CHUNK)):
                    chunk = samples[offset:offset + PUBLISH_CHUNK]
                    chunk_payload = {
                        node_name: [{
                            'ts':     ts + ci,
                            'values': {'ecg_batch': json.dumps(chunk)},
                        }]
                    }
                    result = mqtt.publish(MQTT_TOPIC, json.dumps(chunk_payload), qos=0)
                if batch_num % 25 == 0:
                    n_chunks = (len(samples) + PUBLISH_CHUNK - 1) // PUBLISH_CHUNK
                    print(f'[MQTT] {node_name} ECG batch #{batch_num} '
                          f'({len(samples)} samples, {n_chunks} msg(s))')

        elif pkt_type == 'vitals':
            _, node_name, (ecg_hr, ppg_hr, spo2, temp) = item
            payload = {
                node_name: [{
                    'ts':     ts,
                    'values': {
                        'ecgHeartRate': round(ecg_hr, 1),
                        'ppgHeartRate': round(ppg_hr, 1),
                        'spo2':         round(spo2, 1),
                        'temperature':  round(temp, 1),
                    },
                }]
            }
            if mqtt_connected.is_set():
                mqtt.publish(MQTT_TOPIC, json.dumps(payload), qos=0)
                print(f'[MQTT] {node_name} vitals ECG-HR:{ecg_hr:.1f} PPG-HR:{ppg_hr:.1f} SpO2:{spo2:.1f} Temp:{temp:.1f}')
            else:
                print(f'[MQTT] Not connected — dropping {node_name} vitals')

        publish_q.task_done()


# ── BLE helpers ───────────────────────────────────────────────────────

_scan_lock = threading.Lock()   # only one thread may call adapter.scan_for() at a time


def wait_for_bluetooth():
    while not simplepyble.Adapter.bluetooth_enabled():
        print('[BLE] Bluetooth not enabled — waiting...')
        time.sleep(5)


def ble_connect_node(adapter, node: NodeState):
    wait_for_bluetooth()
    while True:
        if node.addr_changed.is_set():
            node.addr_changed.clear()
        target = node.get_address()
        print(f'[BLE] {node.name}: scanning for {target}...')
        with _scan_lock:
            adapter.scan_for(3000)
            results = adapter.scan_get_results()
        for p in results:
            if p.address().lower() == target.lower():
                print(f'[BLE] {node.name}: found [{p.address()}] - connecting')
                p.connect()
                return p
        print(f'[BLE] {node.name}: not found, retrying...')
        time.sleep(1)


# ── Per-node BLE worker thread ────────────────────────────────────────

def node_worker(node: NodeState, adapter):
    local_batch_count = 0
    last_rx_ts        = 0.0

    def on_notify(data: bytes):
        nonlocal local_batch_count, last_rx_ts
        last_rx_ts = time.time()
        if len(data) == VITALS_SIZE:
            # Vitals: 4 × float32 LE = 16 bytes: [ecgHr, ppgHr, spo2, temp]
            ecg_hr, ppg_hr, spo2, temp = struct.unpack_from('<4f', data)
            try:
                publish_q.put_nowait(('vitals', node.name, (ecg_hr, ppg_hr, spo2, temp)))
            except queue.Full:
                print(f'[BLE] {node.name}: publish queue full — dropping vitals')
        elif len(data) >= 2 and len(data) % 2 == 0:
            # ECG: N × int16 LE — batch size is dynamic (ecgSampleFreq × ecgPacketInterval / 1000)
            n_samples = len(data) // 2
            samples = list(struct.unpack_from(f'<{n_samples}h', data))
            local_batch_count += 1
            try:
                publish_q.put_nowait(('ecg', node.name, samples))
            except queue.Full:
                print(f'[BLE] {node.name}: publish queue full — dropping ECG batch')
            if local_batch_count % 25 == 0:
                print(f'[BLE] {node.name}: ECG batch #{local_batch_count} ({n_samples} samples)')
        else:
            print(f'[BLE] {node.name}: unexpected notify len={len(data)}')

    while True:
        node.addr_changed.clear()
        try:
            peripheral = ble_connect_node(adapter, node)
            last_rx_ts = time.time()
            peripheral.notify(SERVICE_UUID, CHARACTERISTIC_UUID, on_notify)
            print(f'[BLE] {node.name}: streaming ECG from {node.get_address()}')

            # 1. Push current thresholds immediately after connect
            p = node.build_threshold_payload()
            if p:
                try:
                    peripheral.write_request(SERVICE_UUID, RX_CHAR_UUID, p)
                    print(f'[BLE] {node.name}: initial thresholds sent')
                except Exception as e:
                    print(f'[BLE] {node.name}: initial threshold write error: {e}')

            # 2. Push current ECG config
            p = node.build_ecg_cfg_payload()
            if p:
                try:
                    peripheral.write_request(SERVICE_UUID, RX_CHAR_UUID, p)
                    print(f'[BLE] {node.name}: initial ECG config sent')
                except Exception as e:
                    print(f'[BLE] {node.name}: initial ECG config write error: {e}')

            # 3. Push current PPG config
            p = node.build_ppg_cfg_payload()
            if p:
                try:
                    peripheral.write_request(SERVICE_UUID, RX_CHAR_UUID, p)
                    print(f'[BLE] {node.name}: initial PPG config sent')
                except Exception as e:
                    print(f'[BLE] {node.name}: initial PPG config write error: {e}')

            # 4. Push current vital interval
            p = node.build_vital_cfg_payload()
            if p:
                try:
                    peripheral.write_request(SERVICE_UUID, RX_CHAR_UUID, p)
                    print(f'[BLE] {node.name}: initial vital config sent')
                except Exception as e:
                    print(f'[BLE] {node.name}: initial vital config write error: {e}')

            while True:
                time.sleep(2)

                # Drain any pending BLE write commands queued by the MQTT thread
                while not node.cmd_q.empty():
                    try:
                        cmd_payload = node.cmd_q.get_nowait()
                        peripheral.write_request(SERVICE_UUID, RX_CHAR_UUID, cmd_payload)
                        label = {
                            CMD_THR:       'thresholds',
                            CMD_ECG_CFG:   'ECG config',
                            CMD_PPG_CFG:   'PPG config',
                            CMD_VITAL_CFG: 'vital config',
                        }.get(cmd_payload[0], 'config')
                        print(f'[BLE] {node.name}: {label} written')
                    except queue.Empty:
                        break
                    except Exception as e:
                        print(f'[BLE] {node.name}: write error: {e}')
                        break

                if node.addr_changed.is_set():
                    print(f'[BLE] {node.name}: address changed -> reconnecting')
                    break
                if time.time() - last_rx_ts > NO_DATA_TIMEOUT_S:
                    print(f'[BLE] {node.name}: no data for {NO_DATA_TIMEOUT_S}s — reconnecting')
                    break

        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f'[BLE] {node.name}: {e} — retry in 5s')
            time.sleep(5)


# ── Main ──────────────────────────────────────────────────────────────

def main():
    global nodes

    if not ACCESS_TOKEN:
        print('ERROR: TB_GATEWAY_ACCESS_TOKEN must be set in .env.local')
        return

    nodes = _parse_node_list()

    print('=' * 49)
    print(f'  BLE ECG Gateway -> ThingsBoard  ({len(nodes)} node(s))')
    print('=' * 49)
    print(f'Broker -> {MQTT_HOST}:{MQTT_PORT}')
    for n in nodes.values():
        print(f'  {n.name} -> {n.get_address()}')
    print()

    mqtt_setup()
    threading.Thread(target=publish_worker, daemon=True).start()

    adapters = simplepyble.Adapter.get_adapters()
    if not adapters:
        print('No BLE adapters found')
        return
    adapter = adapters[0]

    worker_threads = []
    for node in nodes.values():
        t = threading.Thread(target=node_worker, args=(node, adapter),
                             name=f'ble-{node.name}', daemon=True)
        t.start()
        worker_threads.append(t)

    try:
        for t in worker_threads:
            t.join()
    except KeyboardInterrupt:
        print('\nStopping...')

    publish_q.put(None)
    mqtt.loop_stop()
    mqtt.disconnect()
    print('Done.')


if __name__ == '__main__':
    main()
