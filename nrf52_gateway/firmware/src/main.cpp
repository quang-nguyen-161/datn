// ── HealthMonitor ESP32 Firmware ─────────────────────────────────────────────
// Receives framed binary packets from an nRF52832 BLE central over UART,
// then forwards ECG batch + vitals to ThingsBoard via MQTT gateway API.
//
// Architecture: (BLE nodes) ──BLE──> nRF52832 ──UART──> ESP32 ──WiFi──> ThingsBoard
//
// UART protocol (RX=16, TX=17, 115200 baud):
//   [0xAA][0x55][TYPE][NAME_LEN][NAME...][LEN_LO][LEN_HI][DATA...][XOR_CHK]
//   TYPE 0x01 = ECG:    50 × int16_t LE (100 bytes)
//   TYPE 0x02 = Vitals: [hrEcg u8][hrPpg u8][spo2 u8][temp u16 LE x10] = 5 bytes
//
// First boot — captive portal (AP "HealthMonitor-Setup"):
//   Enter WiFi SSID/pass + ThingsBoard admin credentials.
//   After saving the device restarts and connects.
//
// Node discovery (every 10s via HTTPS) + auto-registration from UART packets.
// ECG config sync (every 3s): polls ecgSampleFreq/ecgPacketInterval from TB shared
//   attributes per node; sends PKT_TYPE_CFG(0x03) via UART when values change.
//   nRF52832 central forwards the 5-byte config payload to the BLE node's RX char.
// TB gateway API auto-creates leaf devices on first publish.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <time.h>

// ── Compile-time constants ────────────────────────────────────────────────────

// Primary: old c7-2slab server. Fallback: local ThingsBoard (Docker) on the LAN —
// each broker has its own gateway device/token, so both must be set.
const char* TB_HOST_PRIMARY  = "103.116.39.179";
const char* TB_HOST_FALLBACK = "192.168.1.100";   // <-- set to your local TB server's LAN IP
#define TB_GATEWAY_TOKEN_PRIMARY  "4o51ajerynq34mtosc26"
#define TB_GATEWAY_TOKEN_FALLBACK "gFrMeC66odU1ZVlNP60X"

#define TB_MQTT_PORT          1883
#define MQTT_BUF_SIZE         2048

#define BATCH_SIZE            100     // max int16_t samples per ECG packet (100×2=200 B fits BLE ATT ~238 B)
#define PUBLISH_CHUNK         100     // max samples per MQTT message; mirrors BLE max payload — only batches >100 split
#define NODE_SYNC_INTERVAL_MS 10000
#define PAYLOAD_BUF_SIZE      4096
#define MAX_NODES             16

// ── UART framing (ESP32 ← nRF52832 central) ──────────────────────────────────
// Packet: [0xAA][0x55][TYPE][NAME_LEN][NAME...][LEN_LO][LEN_HI][DATA...][XOR_CHK]
// XOR_CHK = XOR of all bytes from TYPE through the last DATA byte
// TYPE 0x01 = ECG:    DATA = N × int16_t LE, N = 1..BATCH_SIZE (default 50, max 100)
// TYPE 0x02 = Vitals: DATA = [hrEcg u8][hrPpg u8][spo2 u8][temp u16 LE x10] = 5 bytes

#define UART_RX_PIN    16
#define UART_TX_PIN    17
#define UART_BAUD      115200
#define PKT_TYPE_ECG   0x01
#define PKT_TYPE_VIT   0x02
#define PKT_TYPE_CFG   0x03   // ESP32 → nRF central: ECG config  5 B [0xCF]...
#define PKT_TYPE_THR   0x04   // ESP32 → nRF central: thresholds 31 B [0xCE]...
#define PKT_TYPE_PPG   0x05   // ESP32 → nRF central: PPG config  4 B [0xCD]...
#define PKT_TYPE_VCF   0x06   // ESP32 → nRF central: vital cfg   3 B [0xCC]...
#define PKT_TYPE_MODE  0x07   // ESP32 → nRF central: mode cfg    6 B [0xCB]...
#define ECG_CFG_CMD    0xCF
#define THR_CMD        0xCE
#define PPG_CFG_CMD    0xCD
#define VITAL_CFG_CMD  0xCC
#define MODE_CFG_CMD   0xCB
#define PKT_MAX_NAME   15
#define PKT_MAX_DATA   256

#define DEFAULT_FREQ_HZ           250
#define DEFAULT_INTERVAL_MS       200
#define DEFAULT_PPG_FREQ_HZ       100
#define DEFAULT_PPG_HR_SOURCE     0  // 0=IR, 1=RED
#define DEFAULT_VITAL_INTERVAL_MS 1000
#define DEFAULT_DEVICE_MODE       0
#define DEFAULT_PERIODIC_INTERVAL 10
#define DEFAULT_CAPTURE_WINDOW    5
#define DEFAULT_SHOW_ECG          1
#define CONFIG_SYNC_MS            3000
#define TB_KEY_FREQ               "ecgSampleFreq"
#define TB_KEY_INTERVAL           "ecgPacketInterval"
#define TB_KEY_PPG_FREQ           "ppgSampleFreq"
#define TB_KEY_PPG_HR_SOURCE      "ppgHrSource"
#define TB_KEY_VITAL_INTERVAL     "vitalInterval"
#define TB_KEY_DEVICE_MODE        "deviceMode"
#define TB_KEY_PERIODIC_INTERVAL  "periodicInterval"
#define TB_KEY_CAPTURE_WINDOW     "captureWindow"
#define TB_KEY_SHOW_ECG           "showEcg"

// ── Threshold keys — 24 values matching gateway.py THRESHOLD_KEYS order ──────
// [0..5]  ppgHr:  normalMin/Max, warnMin/Max, dangerMin/Max
// [6..11] ecgHr:  normalMin/Max, warnMin/Max, dangerMin/Max
// [12..17] spo2:  normalMin/Max, warnMin/Max, dangerMin/Max
// [18..23] temp×10: normalMin/Max, warnMin/Max, dangerMin/Max  (uint16, stored ×10)
#define THR_COUNT 24
static const char * const THR_KEYS[THR_COUNT] = {
    "ppgHr_normalMin","ppgHr_normalMax","ppgHr_warnMin","ppgHr_warnMax","ppgHr_dangerMin","ppgHr_dangerMax",
    "ecgHr_normalMin","ecgHr_normalMax","ecgHr_warnMin","ecgHr_warnMax","ecgHr_dangerMin","ecgHr_dangerMax",
    "spo2_normalMin", "spo2_normalMax", "spo2_warnMin", "spo2_warnMax", "spo2_dangerMin", "spo2_dangerMax",
    "temp_normalMin", "temp_normalMax", "temp_warnMin", "temp_warnMax", "temp_dangerMin", "temp_dangerMax",
};
// Indices 18-23 are temp values stored ×10 in ThingsBoard (0.1°C resolution in uint16)
static const int DEFAULT_THR[THR_COUNT] = {
     60, 100,  50, 120,  40, 130,   // ppgHr
     60, 100,  50, 120,  40, 130,   // ecgHr
     95, 100,  90, 100,  88, 100,   // spo2
    361, 372, 355, 385, 350, 395,   // temp ×10 (36.1-37.2 normal, 35.5-38.5 warn, 35.0-39.5 danger)
};

// ── Runtime config ────────────────────────────────────────────────────────────

static char wifiSsid[64]    = "";
static char wifiPass[64]    = "";
static char tbAdminUser[64] = "";
static char tbAdminPass[64] = "";

// ── Node registry ─────────────────────────────────────────────────────────────

static String            nodeNames[MAX_NODES];
static int               nodeCount = 0;
static SemaphoreHandle_t nodeMutex;
static SemaphoreHandle_t adminMutex;    // serialises all adminClient HTTPS calls

// Per-node device-access tokens (resolved lazily from admin API)
static String nodeTokens[MAX_NODES];
// Last config successfully pushed to each node (detect changes)
static int    nodeLastFreq[MAX_NODES];
static int    nodeLastInterval[MAX_NODES];
static int    nodeLastPpgFreq[MAX_NODES];
static int    nodeLastPpgHrSource[MAX_NODES];
static int    nodeLastVitalInterval[MAX_NODES];
static int    nodeLastMode[MAX_NODES];
static int    nodeLastPeriodicInterval[MAX_NODES];
static int    nodeLastCaptureWindow[MAX_NODES];
static int    nodeLastShowEcg[MAX_NODES];

// Per-node received vitals (updated by UART vital packets)
static float nodeHr[MAX_NODES]    = {};
static float nodePpgHr[MAX_NODES] = {};
static float nodeSpo2[MAX_NODES]  = {};
static float nodeTemp[MAX_NODES]  = {};

// ── Runtime state ─────────────────────────────────────────────────────────────

static String      adminJwt = "";
static Preferences prefs;

// ── Admin HTTPS client (node discovery only) ──────────────────────────────────

static WiFiClientSecure adminClient;

// ── MQTT client (all telemetry) ───────────────────────────────────────────────

static WiFiClient   mqttNetClient;
static PubSubClient mqttClient(mqttNetClient);

// ── UART parser state machine ─────────────────────────────────────────────────

enum UartSm { SM_M0, SM_M1, SM_TYPE, SM_NL, SM_NAME, SM_LL, SM_LH, SM_DATA, SM_CHK };

static UartSm   smState    = SM_M0;
static uint8_t  pktType    = 0;
static char     pktName[PKT_MAX_NAME + 1] = {};
static uint8_t  pktNameLen = 0;
static uint8_t  pktNameIdx = 0;
static uint8_t  pktData[PKT_MAX_DATA] = {};
static uint16_t pktDataLen = 0;
static uint16_t pktDataIdx = 0;
static uint8_t  pktXor     = 0;

// ── Per-node incoming data buffers ────────────────────────────────────────────

static int16_t            nodeBatch[MAX_NODES][BATCH_SIZE];
static int                nodeBatchSize[MAX_NODES]  = {};  // actual sample count from last packet
static bool               nodeBatchReady[MAX_NODES] = {};
static unsigned long long nodeBatchTs[MAX_NODES]    = {};
static bool               nodeVitalReady[MAX_NODES] = {};

// Per-node threshold values (24 ints, indices 18-23 are temp×10 uint16)
static int     nodeThrVals[MAX_NODES][THR_COUNT];

static char payload[PAYLOAD_BUF_SIZE];

// ── Helpers ───────────────────────────────────────────────────────────────────

static unsigned long long epochMs() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  if (tv.tv_sec < 1000000000L) return 0;
  return (unsigned long long)tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL;
}

static const char* TB_ADMIN_HOST = "c7.hust-2slab.org";

static String tbUrl(const String& path) {
  return String("https://") + TB_ADMIN_HOST + path;
}

static String jsonStr(const String& json, const String& key) {
  String needle = "\"" + key + "\":\"";
  int s = json.indexOf(needle);
  if (s < 0) return "";
  s += needle.length();
  int e = json.indexOf("\"", s);
  return e > s ? json.substring(s, e) : "";
}

static int jsonInt(const String& json, const String& key, int def = 0) {
  String k = "\"" + key + "\":";
  int p = json.indexOf(k);
  if (p < 0) return def;
  return json.substring(p + k.length()).toInt();
}

static float jsonFloat(const String& json, const String& key, float def = 0.0f) {
  String k = "\"" + key + "\":";
  int p = json.indexOf(k);
  if (p < 0) return def;
  return json.substring(p + k.length()).toFloat();
}

// ── NVS ───────────────────────────────────────────────────────────────────────

static bool loadConfig() {
  prefs.begin("hm", true);
  String ssid = prefs.getString("wifi_ssid", "");
  String pass = prefs.getString("wifi_pass", "");
  String user = prefs.getString("tb_user",   "");
  String tpas = prefs.getString("tb_pass",   "");
  prefs.end();
  if (ssid.length() == 0 || user.length() == 0) return false;
  ssid.toCharArray(wifiSsid,    sizeof(wifiSsid));
  pass.toCharArray(wifiPass,    sizeof(wifiPass));
  user.toCharArray(tbAdminUser, sizeof(tbAdminUser));
  tpas.toCharArray(tbAdminPass, sizeof(tbAdminPass));
  return true;
}

static void loadNodesFromNVS() {
  prefs.begin("hm", true);
  int cnt = prefs.getInt("n_count", 0);
  nodeCount = 0;
  for (int i = 0; i < cnt && nodeCount < MAX_NODES; i++) {
    char nk[8];
    snprintf(nk, sizeof(nk), "n_%d", i);
    String name = prefs.getString(nk, "");
    if (name.length()) nodeNames[nodeCount++] = name;
  }
  prefs.end();
  Serial.printf("[Nodes] %d node(s) loaded from NVS\n", nodeCount);
}

static void saveNodesToNVS() {
  prefs.begin("hm", false);
  prefs.putInt("n_count", nodeCount);
  for (int i = 0; i < nodeCount; i++) {
    char nk[8];
    snprintf(nk, sizeof(nk), "n_%d", i);
    prefs.putString(nk, nodeNames[i]);
  }
  prefs.end();
}

// ── Captive portal ────────────────────────────────────────────────────────────

static const char PAGE1_HTML[] = R"html(
<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HealthMonitor Setup</title>
<style>
  *{box-sizing:border-box}
  body{font-family:sans-serif;background:#f0f2f5;margin:0;padding:24px}
  .card{background:#fff;max-width:420px;margin:0 auto;border-radius:12px;padding:28px;box-shadow:0 2px 12px rgba(0,0,0,.1)}
  h2{margin:0 0 4px;color:#1a1a2e;font-size:20px}
  .sub{color:#888;font-size:13px;margin:0 0 22px}
  label{display:block;font-size:13px;font-weight:600;color:#444;margin-bottom:5px}
  input{display:block;width:100%;padding:10px 12px;border:1px solid #ddd;border-radius:8px;font-size:15px;margin-bottom:16px;outline:none}
  input:focus{border-color:#2196F3}
  hr{border:none;border-top:1px solid #eee;margin:18px 0}
  .err{background:#fff0f0;color:#c62828;border:1px solid #ffcdd2;border-radius:8px;padding:10px 14px;font-size:13px;margin-bottom:16px}
  button{width:100%;padding:13px;background:#2196F3;color:#fff;border:none;border-radius:8px;font-size:16px;font-weight:600;cursor:pointer}
  button:hover{background:#1976D2}
</style></head><body>
<div class="card">
  <h2>HealthMonitor Setup</h2>
  <p class="sub">Nodes are discovered automatically from ThingsBoard.</p>
  %%ERROR%%
  <form method="POST" action="/save">
    <label>WiFi Network (SSID)</label>
    <input name="ssid" placeholder="Your WiFi name" required>
    <label>WiFi Password</label>
    <input name="pass" type="password" placeholder="Leave blank if open network">
    <hr>
    <label>ThingsBoard Admin Email</label>
    <input name="tbuser" placeholder="tenant@thingsboard.org" required>
    <label>ThingsBoard Admin Password</label>
    <input name="tbpass" type="password" placeholder="tenant" required>
    <button type="submit">Save &amp; Connect</button>
  </form>
</div></body></html>
)html";

static void startConfigPortal() {
  Serial.println("[Portal] Starting AP: HealthMonitor-Setup");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("HealthMonitor-Setup");
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("[Portal] Open http://%s\n", apIP.toString().c_str());

  DNSServer dns;
  dns.start(53, "*", apIP);
  WebServer server(80);

  String pError = "";
  bool   pNeedsSave = false;
  String pSsid, pPass, pUser, pPass2;

  server.on("/", HTTP_GET, [&]() {
    String html = PAGE1_HTML;
    html.replace("%%ERROR%%", pError.length()
      ? "<div class='err'>" + pError + "</div>" : "");
    pError = "";
    server.send(200, "text/html", html);
  });

  server.on("/save", HTTP_POST, [&]() {
    pSsid  = server.arg("ssid");
    pPass  = server.arg("pass");
    pUser  = server.arg("tbuser");
    pPass2 = server.arg("tbpass");
    pNeedsSave = true;
    server.send(200, "text/html",
      "<html><head><meta charset='utf-8'>"
      "<meta http-equiv='refresh' content='2;url=/'></head>"
      "<body style='font-family:sans-serif;max-width:420px;margin:40px auto;padding:24px;text-align:center'>"
      "<h3 style='color:#555'>Verifying WiFi&hellip;</h3>"
      "<p style='color:#aaa'>Please wait.</p></body></html>");
  });

  server.onNotFound([&]() {
    server.sendHeader("Location", "http://192.168.4.1/");
    server.send(302, "text/plain", "");
  });

  server.begin();

  while (true) {
    dns.processNextRequest();
    server.handleClient();

    if (!pNeedsSave) continue;
    pNeedsSave = false;

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("HealthMonitor-Setup");
    WiFi.begin(pSsid.c_str(), pPass.c_str());
    bool ok = false;
    for (int i = 0; i < 30; i++) {
      if (WiFi.status() == WL_CONNECTED) { ok = true; break; }
      delay(500);
    }
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HealthMonitor-Setup");

    if (!ok) {
      pError = "Could not connect to '" + pSsid + "'. Check the SSID and password.";
      continue;
    }

    prefs.begin("hm", false);
    prefs.putString("wifi_ssid", pSsid);
    prefs.putString("wifi_pass", pPass);
    prefs.putString("tb_user",   pUser);
    prefs.putString("tb_pass",   pPass2);
    prefs.putInt("n_count", 0);
    prefs.end();

    pError = "__ok__:" + pSsid;
    delay(2000);
    ESP.restart();
  }
}

// ── TB admin API helpers (node discovery only) ────────────────────────────────

static bool ensureJwt() {
  if (adminJwt.length()) return true;
  String resp;
  char body[256];
  snprintf(body, sizeof(body),
    "{\"username\":\"%s\",\"password\":\"%s\"}", tbAdminUser, tbAdminPass);
  adminClient.setInsecure();
  HTTPClient http;
  http.begin(adminClient, tbUrl("/api/auth/login"));
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  resp = http.getString();
  http.end();
  if (code != 200) { Serial.printf("[Auth] login failed %d\n", code); return false; }
  adminJwt = jsonStr(resp, "token");
  return adminJwt.length() > 0;
}

static int tbGet(const String& path, String& out) {
  adminClient.setInsecure();
  HTTPClient http;
  http.begin(adminClient, tbUrl(path));
  http.addHeader("X-Authorization", "Bearer " + adminJwt);
  int code = http.GET();
  out = http.getString();
  http.end();
  if (code == 401) adminJwt = "";
  return code;
}

// ── Sync node registry with ThingsBoard ───────────────────────────────────────

static void syncNodes() {
  xSemaphoreTake(adminMutex, portMAX_DELAY);
  bool jwtOk = ensureJwt();
  String resp;
  int code = jwtOk ? tbGet("/api/tenant/devices?pageSize=100&page=0", resp) : -1;
  xSemaphoreGive(adminMutex);
  if (!jwtOk || code != 200) { Serial.printf("[Sync] devices fetch %d\n", code); return; }

  String newNames[MAX_NODES];
  int newCount = 0;
  int pos = 0;
  while (newCount < MAX_NODES) {
    int np = resp.indexOf("\"name\":\"", pos);
    if (np < 0) break;
    np += 8;
    int ne = resp.indexOf("\"", np);
    String name = resp.substring(np, ne);
    String lower = name; lower.toLowerCase();
    if (lower.indexOf("node") >= 0) newNames[newCount++] = name;
    pos = ne + 1;
  }

  xSemaphoreTake(nodeMutex, portMAX_DELAY);
  bool changed = (newCount != nodeCount);
  for (int i = 0; i < newCount && !changed; i++)
    if (nodeNames[i] != newNames[i]) changed = true;
  if (changed) {
    // Invalidate cached state — indices may have shifted
    for (int j = 0; j < MAX_NODES; j++) {
      nodeTokens[j]       = "";
      nodeLastFreq[j]     = 0;
      nodeLastInterval[j] = 0;
      nodeLastMode[j]             = -1;
      nodeLastPeriodicInterval[j] = -1;
      nodeLastCaptureWindow[j]    = -1;
      nodeLastShowEcg[j]          = -1;
      memset(nodeThrVals[j], 0, sizeof(nodeThrVals[j]));
    }
    nodeCount = newCount;
    for (int i = 0; i < nodeCount; i++) nodeNames[i] = newNames[i];
  }
  xSemaphoreGive(nodeMutex);

  if (changed) saveNodesToNVS();

  Serial.printf("[Sync] %d node(s):", newCount);
  for (int i = 0; i < newCount; i++) Serial.printf(" %s", newNames[i].c_str());
  Serial.println();
}

// ── Node token resolution (caller must hold adminMutex) ───────────────────────
// Resolves the device-access token for the named node via admin API.

static bool resolveNodeToken(const String& name, String& tokenOut) {
  if (!ensureJwt()) return false;
  String resp;
  if (tbGet("/api/tenant/devices?pageSize=100&page=0", resp) != 200) return false;

  // Find the entry matching this name and extract its device UUID
  String nameTag = "\"name\":\"" + name + "\"";
  int np = resp.indexOf(nameTag);
  if (np < 0) return false;
  String idMark = "\"entityType\":\"DEVICE\",\"id\":\"";
  int mp = resp.lastIndexOf(idMark, np);
  if (mp < 0) return false;
  mp += idMark.length();
  int me = resp.indexOf("\"", mp);
  if (me <= mp) return false;
  String devId = resp.substring(mp, me);

  String credResp;
  if (tbGet("/api/device/" + devId + "/credentials", credResp) != 200) return false;
  tokenOut = jsonStr(credResp, "credentialsId");
  return tokenOut.length() > 0;
}

// ── Fetch ECG config + thresholds from TB shared attributes (caller holds adminMutex) ──
// Matches gateway.py: ALL_SHARED_KEYS = bleAddress + 24 threshold keys + ECG cfg keys

static bool fetchNodeConfig(const String& token,
    int& freq, int& interval,
    int& ppgFreq, int& ppgHrSource, int& vitalInterval,
    int& deviceMode, int& periodicInterval, int& captureWindow, int& showEcg,
    int thr[THR_COUNT]) {
  adminClient.setInsecure();
  HTTPClient http;
  String keys = String(TB_KEY_FREQ) + "," + TB_KEY_INTERVAL
              + "," + TB_KEY_PPG_FREQ + "," + TB_KEY_PPG_HR_SOURCE
              + "," + TB_KEY_VITAL_INTERVAL
              + "," + TB_KEY_DEVICE_MODE + "," + TB_KEY_PERIODIC_INTERVAL
              + "," + TB_KEY_CAPTURE_WINDOW + "," + TB_KEY_SHOW_ECG;
  for (int i = 0; i < THR_COUNT; i++) { keys += ","; keys += THR_KEYS[i]; }
  http.begin(adminClient, tbUrl("/api/v1/" + token + "/attributes?sharedKeys=" + keys));
  int code = http.GET();
  String body = http.getString();
  http.end();
  if (code != 200) return false;

  { int v = jsonInt(body, TB_KEY_FREQ);           if (v > 0) freq          = v; }
  { int v = jsonInt(body, TB_KEY_INTERVAL);        if (v > 0) interval      = v; }
  { int v = jsonInt(body, TB_KEY_PPG_FREQ);        if (v > 0) ppgFreq       = v; }
  { String s = jsonStr(body, TB_KEY_PPG_HR_SOURCE); if (s.length() > 0) ppgHrSource = (s == "red") ? 1 : 0; }
  { int v = jsonInt(body, TB_KEY_VITAL_INTERVAL);  if (v > 0) vitalInterval  = v; }
  { int v = jsonInt(body, TB_KEY_DEVICE_MODE,       -1); if (v >= 0) deviceMode       = v; }
  { int v = jsonInt(body, TB_KEY_PERIODIC_INTERVAL, -1); if (v >  0) periodicInterval = v; }
  { int v = jsonInt(body, TB_KEY_CAPTURE_WINDOW,    -1); if (v >  0) captureWindow    = v; }
  { int v = jsonInt(body, TB_KEY_SHOW_ECG,          -1); if (v >= 0) showEcg          = v; }

  for (int i = 0; i < THR_COUNT; i++) {
    bool isTemp = (i >= 18);
    int v;
    if (isTemp) {
      // Temp keys are stored ×10 in TB (0.1°C resolution fits uint16).
      // If the value is already stored as an integer (e.g. 361), jsonInt works.
      // If stored as a float (36.1), multiply by 10 and round.
      float fv = jsonFloat(body, THR_KEYS[i], -1.0f);
      if (fv < 0) { v = DEFAULT_THR[i]; }
      else         { v = (fv > 100.0f) ? (int)roundf(fv) : (int)roundf(fv * 10.0f); }
    } else {
      v = jsonInt(body, THR_KEYS[i], DEFAULT_THR[i]);
    }
    thr[i] = v;
  }
  return true;
}

// ── UART command helpers ──────────────────────────────────────────────────────
// Frame: [0xAA][0x55][TYPE][NAME_LEN][NAME...][LEN_LO][LEN_HI][DATA...][XOR]
// DATA bytes are identical to the raw BLE write payload (Path A / Path B parity).

static void _sendUartFrame(uint8_t type, const String& name,
                            const uint8_t* data, uint8_t dlen) {
  uint8_t nlen = (uint8_t)min((int)name.length(), PKT_MAX_NAME);
  uint8_t xorChk = type;
  xorChk ^= nlen;
  for (int i = 0; i < nlen; i++) xorChk ^= (uint8_t)name[i];
  xorChk ^= (dlen & 0xFF);
  xorChk ^= ((dlen >> 8) & 0xFF);
  for (int i = 0; i < dlen; i++) xorChk ^= data[i];
  Serial2.write((uint8_t)0xAA);
  Serial2.write((uint8_t)0x55);
  Serial2.write(type);
  Serial2.write(nlen);
  for (int i = 0; i < nlen; i++) Serial2.write((uint8_t)name[i]);
  Serial2.write((uint8_t)(dlen & 0xFF));
  Serial2.write((uint8_t)((dlen >> 8) & 0xFF));
  Serial2.write(data, dlen);
  Serial2.write(xorChk);
}

// Build 5-byte ECG config payload: [0xCF][freqLo][freqHi][intervalLo][intervalHi]
static void sendUartConfig(const String& name, int freq, int interval) {
  uint8_t data[5] = {
    ECG_CFG_CMD,
    (uint8_t)(freq & 0xFF),     (uint8_t)((freq >> 8) & 0xFF),
    (uint8_t)(interval & 0xFF), (uint8_t)((interval >> 8) & 0xFF),
  };
  _sendUartFrame(PKT_TYPE_CFG, name, data, 5);
  Serial.printf("[CFG] -> %s: %d Hz, %d ms\n", name.c_str(), freq, interval);
}

// Build 31-byte threshold payload: [0xCE][18×uint8 PPG/ECG/SpO2][6×uint16LE temp×10]
static void sendUartThreshold(const String& name, const int thr[THR_COUNT]) {
  uint8_t data[31];
  data[0] = THR_CMD;
  // Indices 0-17: PPG/ECG/SpO2 thresholds as uint8
  for (int i = 0; i < 18; i++)
    data[1 + i] = (uint8_t)constrain(thr[i], 0, 255);
  // Indices 18-23: temp thresholds as uint16 LE (stored ×10)
  for (int i = 0; i < 6; i++) {
    uint16_t v = (uint16_t)constrain(thr[18 + i], 0, 65535);
    data[19 + i * 2]     = (uint8_t)(v & 0xFF);
    data[19 + i * 2 + 1] = (uint8_t)(v >> 8);
  }
  _sendUartFrame(PKT_TYPE_THR, name, data, 31);
  Serial.printf("[THR] -> %s ppgWarn=[%d,%d] ecgWarn=[%d,%d] spo2Warn=[%d,%d] tempWarn=[%d,%d]x10\n",
    name.c_str(), thr[2], thr[3], thr[8], thr[9], thr[14], thr[15], thr[20], thr[21]);
}

// Build 4-byte PPG config payload: [0xCD][freqLo][freqHi][hrSrc]
static void sendUartPpgConfig(const String& name, int freq, int hrSource) {
  uint8_t data[4] = {
    PPG_CFG_CMD,
    (uint8_t)(freq & 0xFF), (uint8_t)((freq >> 8) & 0xFF),
    (uint8_t)(hrSource ? 1 : 0),
  };
  _sendUartFrame(PKT_TYPE_PPG, name, data, 4);
  Serial.printf("[PPG] -> %s: %d Hz hrSource=%s\n", name.c_str(), freq, hrSource ? "red" : "ir");
}

// Build 3-byte vital interval payload: [0xCC][intervalLo][intervalHi]
static void sendUartVitalConfig(const String& name, int intervalMs) {
  uint8_t data[3] = {
    VITAL_CFG_CMD,
    (uint8_t)(intervalMs & 0xFF), (uint8_t)((intervalMs >> 8) & 0xFF),
  };
  _sendUartFrame(PKT_TYPE_VCF, name, data, 3);
  Serial.printf("[VCF] -> %s: %d ms\n", name.c_str(), intervalMs);
}

// Build 7-byte mode config payload: [0xCB][mode][periodSecLo][periodSecHi][capSecLo][capSecHi][ecgEnabled]
static void sendUartModeConfig(const String& name, int mode, int periodSec, int captureSec, int showEcg) {
  uint8_t data[7] = {
    MODE_CFG_CMD,
    (uint8_t)constrain(mode, 0, 255),
    (uint8_t)(periodSec & 0xFF),  (uint8_t)((periodSec >> 8) & 0xFF),
    (uint8_t)(captureSec & 0xFF), (uint8_t)((captureSec >> 8) & 0xFF),
    (uint8_t)(showEcg ? 1 : 0),
  };
  _sendUartFrame(PKT_TYPE_MODE, name, data, 7);
  Serial.printf("[MOD] -> %s: mode=%d period=%ds capture=%ds ecg=%d\n",
                name.c_str(), mode, periodSec, captureSec, showEcg ? 1 : 0);
}

// ── ECG config sync task — mirrors gateway.py ecg_attr_update_worker ─────────
// Polls TB shared attributes every CONFIG_SYNC_MS; sends UART config on change.
// Runs on Core 0 alongside nodeSyncTask (adminMutex prevents HTTPS conflicts).

static void configSyncTask(void*) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(CONFIG_SYNC_MS));

    // Snapshot node list under nodeMutex
    xSemaphoreTake(nodeMutex, portMAX_DELAY);
    int    count = nodeCount;
    String names[MAX_NODES];
    for (int i = 0; i < count; i++) names[i] = nodeNames[i];
    xSemaphoreGive(nodeMutex);

    for (int i = 0; i < count; i++) {
      xSemaphoreTake(adminMutex, portMAX_DELAY);

      if (nodeTokens[i].length() == 0) {
        String tok;
        if (resolveNodeToken(names[i], tok)) {
          nodeTokens[i] = tok;
          Serial.printf("[CFG] %s token resolved\n", names[i].c_str());
        }
      }

      int freq            = DEFAULT_FREQ_HZ;
      int interval        = DEFAULT_INTERVAL_MS;
      int ppgFreq         = DEFAULT_PPG_FREQ_HZ;
      int ppgHrSource     = DEFAULT_PPG_HR_SOURCE;
      int vitalInterval   = DEFAULT_VITAL_INTERVAL_MS;
      int deviceMode      = DEFAULT_DEVICE_MODE;
      int periodicInterval = DEFAULT_PERIODIC_INTERVAL;
      int captureWindow   = DEFAULT_CAPTURE_WINDOW;
      int showEcg         = DEFAULT_SHOW_ECG;
      int thr[THR_COUNT];
      memcpy(thr, nodeThrVals[i], sizeof(thr));

      bool ok = nodeTokens[i].length() > 0
             && fetchNodeConfig(nodeTokens[i],
                                freq, interval,
                                ppgFreq, ppgHrSource, vitalInterval,
                                deviceMode, periodicInterval, captureWindow, showEcg,
                                thr);

      xSemaphoreGive(adminMutex);
      if (!ok) continue;

      if (freq != nodeLastFreq[i] || interval != nodeLastInterval[i]) {
        nodeLastFreq[i]     = freq;
        nodeLastInterval[i] = interval;
        Serial.printf("[TB] %s ECG: %d Hz, %d ms\n", names[i].c_str(), freq, interval);
        sendUartConfig(names[i], freq, interval);
      }

      if (ppgFreq != nodeLastPpgFreq[i] || ppgHrSource != nodeLastPpgHrSource[i]) {
        nodeLastPpgFreq[i]      = ppgFreq;
        nodeLastPpgHrSource[i]  = ppgHrSource;
        sendUartPpgConfig(names[i], ppgFreq, ppgHrSource);
      }

      if (vitalInterval != nodeLastVitalInterval[i]) {
        nodeLastVitalInterval[i] = vitalInterval;
        sendUartVitalConfig(names[i], vitalInterval);
      }

      if (deviceMode != nodeLastMode[i] || periodicInterval != nodeLastPeriodicInterval[i]
          || captureWindow != nodeLastCaptureWindow[i] || showEcg != nodeLastShowEcg[i]) {
        nodeLastMode[i]             = deviceMode;
        nodeLastPeriodicInterval[i] = periodicInterval;
        nodeLastCaptureWindow[i]    = captureWindow;
        nodeLastShowEcg[i]          = showEcg;
        sendUartModeConfig(names[i], deviceMode, periodicInterval, captureWindow, showEcg);
      }

      if (memcmp(thr, nodeThrVals[i], sizeof(thr)) != 0) {
        memcpy(nodeThrVals[i], thr, sizeof(thr));
        sendUartThreshold(names[i], thr);
      }
    }
  }
}

// Runs on Core 0 — HTTPS never blocks the ECG publish loop on Core 1.
static void nodeSyncTask(void*) {
  for (;;) {
    syncNodes();
    vTaskDelay(pdMS_TO_TICKS(NODE_SYNC_INTERVAL_MS));
  }
}

// ── WiFi + NTP ────────────────────────────────────────────────────────────────

static void setupWiFi() {
  WiFi.begin(wifiSsid, wifiPass);
  Serial.printf("WiFi connecting to %s", wifiSsid);
  for (int i = 0; i < 40; i++) {
    if (WiFi.status() == WL_CONNECTED) break;
    delay(500); Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[WiFi] Failed — clearing credentials, restarting to portal");
    prefs.begin("hm", false);
    prefs.remove("wifi_ssid");
    prefs.remove("wifi_pass");
    prefs.end();
    delay(500);
    ESP.restart();
  }
  Serial.printf("\nWiFi OK — %s\n", WiFi.localIP().toString().c_str());

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("NTP sync");
  for (int i = 0; i < 20; i++) {
    if (epochMs() > 0) { Serial.println(" OK"); return; }
    delay(500); Serial.print(".");
  }
  Serial.println(" (skipped)");
}

// ── MQTT ──────────────────────────────────────────────────────────────────────

static const char* g_tbHost  = TB_HOST_PRIMARY;
static const char* g_tbToken = TB_GATEWAY_TOKEN_PRIMARY;

// Probe the primary (old c7-2slab) broker; fall back to local TB if unreachable.
// Call once after WiFi is up, before mqttClient.setServer().
static void selectBroker() {
  WiFiClient probe;
  if (probe.connect(TB_HOST_PRIMARY, TB_MQTT_PORT, 2000)) {
    probe.stop();
    g_tbHost  = TB_HOST_PRIMARY;
    g_tbToken = TB_GATEWAY_TOKEN_PRIMARY;
    Serial.printf("[MQTT] Using primary broker %s:%d\n", g_tbHost, TB_MQTT_PORT);
  } else {
    g_tbHost  = TB_HOST_FALLBACK;
    g_tbToken = TB_GATEWAY_TOKEN_FALLBACK;
    Serial.printf("[MQTT] Primary broker %s:%d unreachable — falling back to %s:%d\n",
                  TB_HOST_PRIMARY, TB_MQTT_PORT, g_tbHost, TB_MQTT_PORT);
  }
}

static bool mqttConnect() {
  if (mqttClient.connected()) return true;
  char clientId[32];
  snprintf(clientId, sizeof(clientId), "esp32_%08X", (uint32_t)ESP.getEfuseMac());
  bool ok = mqttClient.connect(clientId, g_tbToken, NULL);
  if (ok) Serial.println("[MQTT] Connected");
  else    Serial.printf("[MQTT] Failed rc=%d — retry in 5s\n", mqttClient.state());
  return ok;
}

// ── Node auto-registration from UART ─────────────────────────────────────────

static int ensureNode(const char* name) {
  xSemaphoreTake(nodeMutex, portMAX_DELAY);
  for (int i = 0; i < nodeCount; i++) {
    if (nodeNames[i] == name) { xSemaphoreGive(nodeMutex); return i; }
  }
  if (nodeCount >= MAX_NODES) { xSemaphoreGive(nodeMutex); return -1; }
  int idx       = nodeCount++;
  nodeNames[idx] = String(name);
  xSemaphoreGive(nodeMutex);
  saveNodesToNVS();
  Serial.printf("[UART] New node registered: %s (idx %d)\n", name, idx);
  return idx;
}

// ── UART packet handler ───────────────────────────────────────────────────────

static void handlePacket() {
  int idx = ensureNode(pktName);
  if (idx < 0) { Serial.println("[UART] Node registry full"); return; }

  if (pktType == PKT_TYPE_ECG && pktDataLen >= 2 && (pktDataLen & 1) == 0) {
    // Dynamic batch: sample count derived from actual packet length
    int n = min((int)(pktDataLen / 2), BATCH_SIZE);
    for (int i = 0; i < n; i++)
      nodeBatch[idx][i] = (int16_t)(pktData[i * 2] | ((uint16_t)pktData[i * 2 + 1] << 8));
    nodeBatchSize[idx]  = n;
    nodeBatchTs[idx]    = epochMs();
    nodeBatchReady[idx] = true;
  } else if (pktType == PKT_TYPE_VIT && pktDataLen == 5) {
    // [hrEcg u8][hrPpg u8][spo2 u8][temp u16 LE x10]
    nodeHr[idx]         = pktData[0];
    nodePpgHr[idx]      = pktData[1];
    nodeSpo2[idx]       = pktData[2];
    nodeTemp[idx]       = (float)(pktData[3] | ((uint16_t)pktData[4] << 8)) / 10.0f;
    nodeVitalReady[idx] = true;
  } else {
    Serial.printf("[UART] Unknown pkt type=0x%02X len=%u\n", pktType, pktDataLen);
  }
}

// ── UART state-machine reader ─────────────────────────────────────────────────

static void readUart() {
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    switch (smState) {
      case SM_M0: if (b == 0xAA) smState = SM_M1; break;
      case SM_M1: smState = (b == 0x55) ? SM_TYPE : SM_M0; break;
      case SM_TYPE:
        pktType = b; pktXor = b; smState = SM_NL; break;
      case SM_NL:
        pktNameLen = (b < PKT_MAX_NAME) ? b : PKT_MAX_NAME;
        pktNameIdx = 0; pktXor ^= b;
        smState = (pktNameLen > 0) ? SM_NAME : SM_LL; break;
      case SM_NAME:
        pktName[pktNameIdx++] = (char)b; pktXor ^= b;
        if (pktNameIdx >= pktNameLen) { pktName[pktNameIdx] = '\0'; smState = SM_LL; }
        break;
      case SM_LL:
        pktDataLen = b; pktXor ^= b; smState = SM_LH; break;
      case SM_LH:
        pktDataLen |= ((uint16_t)b << 8); pktXor ^= b;
        pktDataIdx  = 0;
        smState = (pktDataLen > 0) ? SM_DATA : SM_CHK; break;
      case SM_DATA:
        if (pktDataIdx < PKT_MAX_DATA) pktData[pktDataIdx] = b;
        pktXor ^= b;
        if (++pktDataIdx >= pktDataLen) smState = SM_CHK;
        break;
      case SM_CHK:
        if (b == pktXor) handlePacket();
        else Serial.printf("[UART] Bad XOR: got 0x%02X expected 0x%02X\n", b, pktXor);
        smState = SM_M0; break;
    }
  }
}

// ── Publish one ECG batch, split into PUBLISH_CHUNK-sample MQTT messages ─────
// Format per chunk: {"NodeName":[{"ts":epochMs+ci,"values":{"ecg_batch":"[v0,v1,...]"}}]}
// ts is offset by 1 ms per chunk so ThingsBoard preserves ordering.

static void publishEcgBatch(int idx, const String& name, unsigned long long ts) {
  if (!mqttConnect()) return;
  int batchSz = nodeBatchSize[idx] > 0 ? nodeBatchSize[idx] : BATCH_SIZE;
  int nChunks  = (batchSz + PUBLISH_CHUNK - 1) / PUBLISH_CHUNK;

  for (int ci = 0; ci < nChunks; ci++) {
    int start = ci * PUBLISH_CHUNK;
    int end   = min(start + PUBLISH_CHUNK, batchSz);
    unsigned long long chunkTs = ts + (unsigned long long)ci;

    int pos = 0;
    pos += snprintf(payload + pos, PAYLOAD_BUF_SIZE - pos,
                    "{\"%s\":[{\"ts\":%llu,\"values\":{\"ecg_batch\":\"[",
                    name.c_str(), chunkTs);
    for (int i = start; i < end && pos < PAYLOAD_BUF_SIZE - 20; i++) {
      if (i > start) payload[pos++] = ',';
      pos += snprintf(payload + pos, PAYLOAD_BUF_SIZE - pos, "%d", nodeBatch[idx][i]);
    }
    pos += snprintf(payload + pos, PAYLOAD_BUF_SIZE - pos, "]\"}}]}");

    bool ok = mqttClient.publish("v1/gateway/telemetry", (uint8_t*)payload, pos);
    Serial.printf("[%s] ECG chunk %d/%d samples[%d..%d] %s\n",
                  name.c_str(), ci + 1, nChunks, start, end - 1, ok ? "OK" : "FAIL");
  }
}

// ── Publish vitals for one node ───────────────────────────────────────────────

static void publishVitalPacket(int idx, const String& name) {
  if (!mqttConnect()) return;
  unsigned long long ts  = epochMs();
  int                pos = 0;
  pos += snprintf(payload + pos, PAYLOAD_BUF_SIZE - pos, "{\"%s\":[{", name.c_str());
  if (ts > 0)
    pos += snprintf(payload + pos, PAYLOAD_BUF_SIZE - pos, "\"ts\":%llu,", ts);
  pos += snprintf(payload + pos, PAYLOAD_BUF_SIZE - pos,
    "\"values\":{\"ecgHeartRate\":%.0f,\"ppgHeartRate\":%.0f,"
    "\"spo2\":%.0f,\"temperature\":%.1f}}]}",
    nodeHr[idx], nodePpgHr[idx], nodeSpo2[idx], nodeTemp[idx]);
  bool ok = mqttClient.publish("v1/gateway/telemetry", (uint8_t*)payload, pos);
  Serial.printf("[%s] vitals ECG-HR:%.0f PPG-HR:%.0f SpO2:%.0f %s\n",
    name.c_str(), nodeHr[idx], nodePpgHr[idx], nodeSpo2[idx], ok ? "OK" : "FAIL");
}

// ── setup / loop ──────────────────────────────────────────────────────────────

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // suppress brownout on WiFi startup
  Serial.begin(115200);
  esp_log_level_set("ssl_client",       ESP_LOG_NONE);

  if (!loadConfig()) {
    Serial.println("[Setup] No config — starting captive portal");
    startConfigPortal();
  }
  Serial.printf("[Setup] WiFi: %s  TB: %s\n", wifiSsid, tbAdminUser);

  setupWiFi();
  loadNodesFromNVS();

  selectBroker();
  mqttClient.setServer(g_tbHost, TB_MQTT_PORT);
  mqttClient.setBufferSize(MQTT_BUF_SIZE);

  nodeMutex  = xSemaphoreCreateMutex();
  adminMutex = xSemaphoreCreateMutex();
  memset(nodeLastFreq,     0, sizeof(nodeLastFreq));
  memset(nodeLastInterval, 0, sizeof(nodeLastInterval));
  // -1 (0xFF bytes) forces a first push even when the value equals 0 (e.g. mode 0)
  memset(nodeLastMode,             0xFF, sizeof(nodeLastMode));
  memset(nodeLastPeriodicInterval, 0xFF, sizeof(nodeLastPeriodicInterval));
  memset(nodeLastCaptureWindow,    0xFF, sizeof(nodeLastCaptureWindow));
  memset(nodeLastShowEcg,          0xFF, sizeof(nodeLastShowEcg));
  for (int i = 0; i < MAX_NODES; i++)
    memcpy(nodeThrVals[i], DEFAULT_THR, sizeof(DEFAULT_THR));
  xTaskCreatePinnedToCore(nodeSyncTask,   "nodeSync", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(configSyncTask, "cfgSync",  8192, NULL, 1, NULL, 0);

  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  Serial.printf("[UART] Listening on RX=%d TX=%d @ %d baud\n",
    UART_RX_PIN, UART_TX_PIN, UART_BAUD);

  mqttConnect();

  Serial.printf("Ready — %d node(s) loaded, waiting for UART data\n", nodeCount);
}

void loop() {
  mqttClient.loop();
  readUart();

  for (int i = 0; i < MAX_NODES; i++) {
    if (nodeBatchReady[i]) {
      nodeBatchReady[i] = false;
      xSemaphoreTake(nodeMutex, portMAX_DELAY);
      String name = (i < nodeCount) ? nodeNames[i] : "";
      xSemaphoreGive(nodeMutex);
      if (name.length()) publishEcgBatch(i, name, nodeBatchTs[i]);
    }
    if (nodeVitalReady[i]) {
      nodeVitalReady[i] = false;
      xSemaphoreTake(nodeMutex, portMAX_DELAY);
      String name = (i < nodeCount) ? nodeNames[i] : "";
      xSemaphoreGive(nodeMutex);
      if (name.length()) publishVitalPacket(i, name);
    }
  }
}
