#include <Arduino.h>
#include <ArduinoOTA.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_system.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled for this target"
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP is required; select an original ESP32 target"
#endif

namespace {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t INQUIRY_DURATION_MS = 12000;
constexpr uint32_t INQUIRY_WINDOW_MS = 1280;
constexpr uint32_t INQUIRY_WINDOW_GRACE_MS = 150;
constexpr uint32_t WIFI_RECOVERY_WINDOW_MS = 1000;
constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 18000;
constexpr uint32_t STA_RETRY_INTERVAL_MS = 60000;
constexpr size_t MAX_BT_RESULTS = 16;
constexpr size_t MAX_BT_NAME = 48;
constexpr size_t LOG_LINES = 24;
constexpr size_t LOG_LINE_LENGTH = 128;

enum class ScanState : uint8_t { IDLE, SCANNING, COMPLETE, ERROR };
struct KnownSma { const char* mac; const char* label; uint32_t serial; };
struct BtResult {
  char mac[18]{};
  char name[MAX_BT_NAME + 1]{};
  int8_t rssi = 0;
  uint32_t cod = 0;
  bool haveName = false;
  bool haveRssi = false;
  bool haveCod = false;
  int8_t knownIndex = -1;
};

const KnownSma KNOWN_SMAS[] = {
  {"02:00:00:00:00:01", "SMA #1", 1000000001UL},
  {"02:00:00:00:00:02", "SMA #2", 1000000002UL},
  {"02:00:00:00:00:03", "SMA #3", 1000000003UL},
};

BluetoothSerial serialBt;
Preferences preferences;
WebServer server(80);
portMUX_TYPE resultsMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;
BtResult btResults[MAX_BT_RESULTS];
volatile size_t btResultCount = 0;
volatile bool btOverflow = false;
char logLines[LOG_LINES][LOG_LINE_LENGTH]{};
size_t logNext = 0;
size_t logCount = 0;
ScanState scanState = ScanState::IDLE;
uint32_t scanStartedAt = 0;
uint32_t scanCompletedAt = 0;
uint32_t inquiryWindowStartedAt = 0;
uint32_t nextInquiryWindowAt = 0;
uint32_t inquiryWindowCount = 0;
uint32_t scanNumber = 0;
uint32_t nextStaAttemptAt = 0;
uint32_t bootAt = 0;
uint32_t lastLoopAt = 0;
uint32_t maxLoopGapMs = 0;
uint32_t httpDuringScanCount = 0;
uint32_t callbackCount = 0;
wl_status_t lastWifiStatus = WL_NO_SHIELD;
bool bluetoothReady = false;
bool inquiryWindowActive = false;
bool otaBusy = false;
bool apActive = false;
String hostname;
String apSsid;
String apPassword;
String otaPassword;

void addLog(const char* format, ...);

const char* scanStateName(ScanState value) {
  switch (value) {
    case ScanState::IDLE: return "IDLE";
    case ScanState::SCANNING: return "SCANNING";
    case ScanState::COMPLETE: return "COMPLETE";
    case ScanState::ERROR: return "ERROR";
  }
  return "ERROR";
}

void logMemory(const char* stage) {
  addLog("[MEM] %s free=%u min=%u largest=%u", stage, ESP.getFreeHeap(), ESP.getMinFreeHeap(),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

void addLog(const char* format, ...) {
  char line[LOG_LINE_LENGTH];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  Serial.println(line);
  portENTER_CRITICAL(&logMux);
  strlcpy(logLines[logNext], line, LOG_LINE_LENGTH);
  logNext = (logNext + 1) % LOG_LINES;
  if (logCount < LOG_LINES) ++logCount;
  portEXIT_CRITICAL(&logMux);
}

String jsonEscape(const char* value) {
  String result;
  if (value == nullptr) return result;
  result.reserve(strlen(value) + 8);
  for (const char* cursor = value; *cursor; ++cursor) {
    const uint8_t c = static_cast<uint8_t>(*cursor);
    if (c == '"' || c == '\\') { result += '\\'; result += static_cast<char>(c); }
    else if (c == '\n') result += F("\\n");
    else if (c == '\r') result += F("\\r");
    else if (c == '\t') result += F("\\t");
    else if (c >= 0x20) result += static_cast<char>(c);
  }
  return result;
}

int8_t knownSmaIndex(const char* mac) {
  for (size_t index = 0; index < std::size(KNOWN_SMAS); ++index) {
    if (strcasecmp(mac, KNOWN_SMAS[index].mac) == 0) return static_cast<int8_t>(index);
  }
  return -1;
}

void onBtDevice(BTAdvertisedDevice* device) {
  if (device == nullptr) return;
  ++callbackCount;
  BtResult incoming;
  const String address = device->getAddress().toString();
  strlcpy(incoming.mac, address.c_str(), sizeof(incoming.mac));
  for (char* cursor = incoming.mac; *cursor; ++cursor) *cursor = toupper(*cursor);
  incoming.haveRssi = device->haveRSSI();
  incoming.rssi = incoming.haveRssi ? device->getRSSI() : 0;
  incoming.haveName = device->haveName();
  if (incoming.haveName) strlcpy(incoming.name, device->getName().c_str(), sizeof(incoming.name));
  incoming.haveCod = device->haveCOD();
  incoming.cod = incoming.haveCod ? device->getCOD() : 0;
  incoming.knownIndex = knownSmaIndex(incoming.mac);

  portENTER_CRITICAL(&resultsMux);
  size_t position = btResultCount;
  for (size_t index = 0; index < btResultCount; ++index) {
    if (strcmp(btResults[index].mac, incoming.mac) == 0) { position = index; break; }
  }
  if (position < MAX_BT_RESULTS) {
    btResults[position] = incoming;
    if (position == btResultCount) ++btResultCount;
  } else btOverflow = true;
  portEXIT_CRITICAL(&resultsMux);

  addLog("[BT] MAC=%s RSSI=%s%d NAME=%s%s%s COD=%s0x%06lX%s",
         incoming.mac, incoming.haveRssi ? "" : "n/a ", incoming.haveRssi ? incoming.rssi : 0,
         incoming.haveName ? "\"" : "", incoming.haveName ? incoming.name : "n/a", incoming.haveName ? "\"" : "",
         incoming.haveCod ? "" : "n/a ", static_cast<unsigned long>(incoming.cod),
         incoming.knownIndex >= 0 ? " MATCH" : "");
  if (incoming.knownIndex >= 0) {
    const KnownSma& sma = KNOWN_SMAS[incoming.knownIndex];
    addLog("[BT] MATCH %s SN=%lu", sma.label, static_cast<unsigned long>(sma.serial));
  }
}

bool startInquiryWindow() {
  serialBt.discoverClear();
  inquiryWindowStartedAt = millis();
  ++inquiryWindowCount;
  inquiryWindowActive = serialBt.discoverAsync(onBtDevice, INQUIRY_WINDOW_MS);
  addLog("[SCAN] WINDOW_START cycle=%lu window=%lu ok=%s free=%u largest=%u",
         static_cast<unsigned long>(scanNumber), static_cast<unsigned long>(inquiryWindowCount),
         inquiryWindowActive ? "true" : "false", ESP.getFreeHeap(),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  return inquiryWindowActive;
}

void stopInquiryWindow(const char* stage) {
  if (!inquiryWindowActive) return;
  serialBt.discoverAsyncStop();
  inquiryWindowActive = false;
  logMemory(stage);
  // BluetoothSerial retains every result in a dynamically allocated std::map.
  // The fixed-size snapshot above is authoritative; free the library copy.
  serialBt.discoverClear();
  logMemory("window_after_library_clear");
  nextInquiryWindowAt = millis() + WIFI_RECOVERY_WINDOW_MS;
}

bool startBtScan() {
  if (!bluetoothReady || otaBusy || scanState == ScanState::SCANNING) return false;
  portENTER_CRITICAL(&resultsMux);
  memset(btResults, 0, sizeof(btResults));
  btResultCount = 0;
  btOverflow = false;
  portEXIT_CRITICAL(&resultsMux);
  ++scanNumber;
  callbackCount = 0;
  httpDuringScanCount = 0;
  maxLoopGapMs = 0;
  inquiryWindowCount = 0;
  inquiryWindowActive = false;
  scanStartedAt = millis();
  scanCompletedAt = 0;
  scanState = ScanState::SCANNING;
  logMemory("scan_before_start");
  addLog("[SCAN] START cycle=%lu elapsed_ms=%lu window_ms=%lu wifi_gap_ms=%lu",
         static_cast<unsigned long>(scanNumber), static_cast<unsigned long>(INQUIRY_DURATION_MS),
         static_cast<unsigned long>(INQUIRY_WINDOW_MS), static_cast<unsigned long>(WIFI_RECOVERY_WINDOW_MS));
  if (!startInquiryWindow()) {
    scanState = ScanState::ERROR;
    addLog("[SCAN] ERROR inquiry_start_failed");
    return false;
  }
  return true;
}

void serviceBtScan() {
  if (scanState != ScanState::SCANNING) return;
  const uint32_t now = millis();
  if (inquiryWindowActive && now - inquiryWindowStartedAt >= INQUIRY_WINDOW_MS + INQUIRY_WINDOW_GRACE_MS) {
    stopInquiryWindow("window_after_stop");
  }
  if (now - scanStartedAt < INQUIRY_DURATION_MS) {
    if (!inquiryWindowActive && static_cast<int32_t>(now - nextInquiryWindowAt) >= 0 && !startInquiryWindow()) {
      scanState = ScanState::ERROR;
      addLog("[SCAN] ERROR window_start_failed");
    }
    return;
  }
  stopInquiryWindow("scan_final_stop");
  scanState = ScanState::COMPLETE;
  scanCompletedAt = millis();
  addLog("[SCAN] COMPLETE cycle=%lu windows=%lu devices=%u callbacks=%lu overflow=%s",
         static_cast<unsigned long>(scanNumber), static_cast<unsigned long>(inquiryWindowCount),
         static_cast<unsigned>(btResultCount),
         static_cast<unsigned long>(callbackCount), btOverflow ? "true" : "false");
  addLog("[SCAN] HEALTH http_during=%lu wifi=%d loop_gap_max=%lu",
         static_cast<unsigned long>(httpDuringScanCount), static_cast<int>(WiFi.status()),
         static_cast<unsigned long>(maxLoopGapMs));
  logMemory("scan_complete");
}

String resultsJson() {
  BtResult snapshot[MAX_BT_RESULTS];
  size_t count;
  bool overflow;
  portENTER_CRITICAL(&resultsMux);
  count = btResultCount;
  overflow = btOverflow;
  memcpy(snapshot, btResults, count * sizeof(BtResult));
  portEXIT_CRITICAL(&resultsMux);
  String json;
  json.reserve(512 + count * 180);
  json += F("{\"state\":\""); json += scanStateName(scanState);
  json += F("\",\"cycle\":"); json += scanNumber;
  json += F(",\"startedAtMs\":"); json += scanStartedAt;
  json += F(",\"completedAtMs\":"); json += scanCompletedAt;
  json += F(",\"overflow\":"); json += overflow ? F("true") : F("false");
  json += F(",\"devices\":[");
  for (size_t index = 0; index < count; ++index) {
    if (index) json += ',';
    const BtResult& item = snapshot[index];
    json += F("{\"mac\":\""); json += item.mac; json += '"';
    json += F(",\"rssi\":"); if (item.haveRssi) json += item.rssi; else json += F("null");
    json += F(",\"name\":"); if (item.haveName) { json += '"'; json += jsonEscape(item.name); json += '"'; } else json += F("null");
    json += F(",\"cod\":"); if (item.haveCod) json += item.cod; else json += F("null");
    json += F(",\"sma\":");
    if (item.knownIndex >= 0) {
      const KnownSma& sma = KNOWN_SMAS[item.knownIndex];
      json += F("{\"label\":\""); json += sma.label; json += F("\",\"serial\":"); json += sma.serial; json += '}';
    } else json += F("null");
    json += '}';
  }
  json += F("]}");
  return json;
}

String statusJson() {
  String json;
  json.reserve(640);
  json += F("{\"uptimeMs\":"); json += millis() - bootAt;
  json += F(",\"heap\":{\"free\":"); json += ESP.getFreeHeap();
  json += F(",\"minimum\":"); json += ESP.getMinFreeHeap();
  json += F(",\"largestInternal\":"); json += heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  json += F("},\"wifi\":{\"mode\":\""); json += apActive ? (WiFi.status() == WL_CONNECTED ? "AP+STA" : "AP") : "STA";
  json += F("\",\"connected\":"); json += WiFi.status() == WL_CONNECTED ? F("true") : F("false");
  json += F(",\"ip\":\""); json += WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString() : String();
  json += F("\",\"rssi\":"); if (WiFi.status() == WL_CONNECTED) json += WiFi.RSSI(); else json += F("null");
  json += F(",\"apSsid\":"); if (apActive) { json += '"'; json += apSsid; json += '"'; } else json += F("null");
  json += F("},\"bluetooth\":{\"ready\":"); json += bluetoothReady ? F("true") : F("false");
  json += F(",\"scanState\":\""); json += scanStateName(scanState);
  json += F("\",\"scanCycle\":"); json += scanNumber;
  json += F(",\"deviceCount\":"); json += static_cast<unsigned>(btResultCount);
  json += F("},\"ota\":{\"busy\":"); json += otaBusy ? F("true") : F("false"); json += F("}}");
  return json;
}

String logJson() {
  String json(F("{\"lines\":["));
  size_t count;
  size_t oldest;
  portENTER_CRITICAL(&logMux);
  count = logCount;
  oldest = (logNext + LOG_LINES - count) % LOG_LINES;
  portEXIT_CRITICAL(&logMux);
  json.reserve(256 + count * 96);
  for (size_t index = 0; index < count; ++index) {
    char line[LOG_LINE_LENGTH];
    portENTER_CRITICAL(&logMux);
    strlcpy(line, logLines[(oldest + index) % LOG_LINES], sizeof(line));
    portEXIT_CRITICAL(&logMux);
    if (index) json += ',';
    json += '"'; json += jsonEscape(line); json += '"';
  }
  json += F("]}");
  return json;
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>SMA SunnyBoy Monitor</title><style>body{font:15px system-ui;max-width:1000px;margin:auto;padding:18px;background:#f5f6f8;color:#18212b}section{background:white;padding:16px;margin:12px 0;border-radius:8px}button,input,select{padding:8px;margin:4px}table{width:100%;border-collapse:collapse}td,th{padding:6px;border-bottom:1px solid #ddd;text-align:left}pre{white-space:pre-wrap;max-height:340px;overflow:auto}.match{font-weight:bold;color:#087830}</style></head><body><h1>SMA SunnyBoy Monitor</h1><section><h2>État</h2><pre id="status">Chargement…</pre><button onclick="scan()">Démarrer un scan Classic</button></section><section><h2>Dernier scan</h2><table><thead><tr><th>MAC</th><th>RSSI</th><th>Nom</th><th>Classe</th><th>SMA</th></tr></thead><tbody id="devices"></tbody></table></section><section><h2>Wi-Fi</h2><button onclick="networks()">Scanner les réseaux</button><form method="post" action="/api/wifi/config"><select name="ssid" id="ssid"></select><input name="password" type="password" placeholder="Mot de passe"><button>Enregistrer et connecter</button></form><button onclick="eraseWifi()">Effacer la configuration Wi-Fi</button></section><section><h2>Journal</h2><pre id="log"></pre></section><script>
async function get(u,o){const r=await fetch(u,o);const t=await r.text();if(!r.ok)throw Error(t);return t?JSON.parse(t):{}}
async function refresh(){try{const [s,b,l]=await Promise.all([get('/api/status'),get('/api/bt/results'),get('/api/log')]);status.textContent=JSON.stringify(s,null,2);devices.innerHTML=b.devices.map(d=>`<tr class="${d.sma?'match':''}"><td>${d.mac}</td><td>${d.rssi??'--'}</td><td>${d.name??'--'}</td><td>${d.cod??'--'}</td><td>${d.sma?.label??''}</td></tr>`).join('');log.textContent=l.lines.join('\n');}catch(e){status.textContent=e}}
async function scan(){await get('/api/bt/scan',{method:'POST'});refresh()}
async function networks(){const n=await get('/api/wifi/networks');ssid.innerHTML=n.networks.map(x=>`<option value="${x.ssid}">${x.ssid} (${x.rssi} dBm)</option>`).join('')}
async function eraseWifi(){if(confirm('Effacer les identifiants Wi-Fi et redémarrer ?'))await get('/api/wifi/reset',{method:'POST',headers:{'Content-Type':'application/json'},body:'{"confirm":"ERASE_WIFI"}'})}
setInterval(refresh,2000);refresh();
</script></body></html>)HTML";

void sendJson(const String& value, int status = 200) { server.send(status, "application/json", value); }

void noteHttpRequest(const char* route) {
  (void)route;
  if (scanState == ScanState::SCANNING) ++httpDuringScanCount;
}

void registerRoutes() {
  server.on("/", HTTP_GET, [] { noteHttpRequest("/"); server.send_P(200, "text/html; charset=utf-8", PAGE); });
  server.on("/api/status", HTTP_GET, [] { noteHttpRequest("status"); sendJson(statusJson()); });
  server.on("/api/bt/results", HTTP_GET, [] { noteHttpRequest("results"); sendJson(resultsJson()); });
  server.on("/api/log", HTTP_GET, [] { noteHttpRequest("log"); sendJson(logJson()); });
  server.on("/api/bt/scan", HTTP_POST, [] {
    noteHttpRequest("scan");
    if (scanState == ScanState::SCANNING) { sendJson("{\"error\":\"scan_already_running\"}", 409); return; }
    if (otaBusy) { sendJson("{\"error\":\"ota_in_progress\"}", 409); return; }
    if (!startBtScan()) { sendJson("{\"error\":\"scan_start_failed\"}", 500); return; }
    sendJson("{\"accepted\":true,\"state\":\"SCANNING\"}", 202);
  });
  server.on("/api/wifi/networks", HTTP_GET, [] {
    const int count = WiFi.scanNetworks(false, true);
    String json(F("{\"networks\":["));
    for (int index = 0; index < count; ++index) {
      if (index) json += ',';
      json += F("{\"ssid\":\""); json += jsonEscape(WiFi.SSID(index).c_str());
      json += F("\",\"rssi\":"); json += WiFi.RSSI(index);
      json += F(",\"encrypted\":"); json += WiFi.encryptionType(index) == WIFI_AUTH_OPEN ? F("false") : F("true"); json += '}';
    }
    json += F("]}"); WiFi.scanDelete(); sendJson(json);
  });
  server.on("/api/wifi/config", HTTP_POST, [] {
    const String ssid = server.arg("ssid");
    const String password = server.arg("password");
    if (ssid.isEmpty() || ssid.length() > 32 || password.length() > 64) {
      sendJson("{\"error\":\"invalid_wifi_configuration\"}", 400); return;
    }
    preferences.begin("sma-monitor", false);
    preferences.putString("ssid", ssid); preferences.putString("wifiPass", password); preferences.end();
    addLog("[WIFI] configuration saved for SSID=%s", ssid.c_str());
    sendJson("{\"saved\":true,\"restarting\":true}"); delay(250); ESP.restart();
  });
  server.on("/api/wifi/reset", HTTP_POST, [] {
    if (server.arg("plain") != "{\"confirm\":\"ERASE_WIFI\"}") {
      sendJson("{\"error\":\"confirmation_required\"}", 400); return;
    }
    preferences.begin("sma-monitor", false);
    preferences.remove("ssid"); preferences.remove("wifiPass"); preferences.end();
    addLog("[WIFI] credentials erased");
    sendJson("{\"erased\":true,\"restarting\":true}"); delay(250); ESP.restart();
  });
  server.on("/api/reboot", HTTP_POST, [] {
    if (server.arg("plain") != "{\"confirm\":\"REBOOT\"}") {
      sendJson("{\"error\":\"confirmation_required\"}", 400); return;
    }
    sendJson("{\"restarting\":true}"); delay(250); ESP.restart();
  });
  server.onNotFound([] { sendJson("{\"error\":\"not_found\"}", 404); });
}

void startProvisioningAp() {
  if (apActive) return;
  WiFi.mode(WIFI_AP_STA);
  apActive = WiFi.softAP(apSsid.c_str(), apPassword.c_str());
  addLog("[WIFI] provisioning AP %s ssid=%s ip=%s", apActive ? "ready" : "failed",
         apSsid.c_str(), WiFi.softAPIP().toString().c_str());
  if (apActive) Serial.printf("[SECRET] provisioning password=%s\n", apPassword.c_str());
}

bool loadWifi(String& ssid, String& password) {
  preferences.begin("sma-monitor", true);
  ssid = preferences.getString("ssid", ""); password = preferences.getString("wifiPass", "");
  preferences.end(); return !ssid.isEmpty();
}

void connectSta() {
  String ssid, password;
  if (!loadWifi(ssid, password)) {
    startProvisioningAp();
    nextStaAttemptAt = millis() + STA_RETRY_INTERVAL_MS;
    return;
  }
  WiFi.mode(apActive ? WIFI_AP_STA : WIFI_STA);
  WiFi.setHostname(hostname.c_str());
  addLog("[WIFI] connecting SSID=%s", ssid.c_str());
  WiFi.begin(ssid.c_str(), password.c_str());
  const uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < STA_CONNECT_TIMEOUT_MS) delay(50);
  if (WiFi.status() == WL_CONNECTED) {
    addLog("[WIFI] connected ip=%s rssi=%d", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    if (apActive) { WiFi.softAPdisconnect(true); apActive = false; WiFi.mode(WIFI_STA); }
  } else {
    addLog("[WIFI] connection timeout; provisioning enabled"); startProvisioningAp();
  }
  nextStaAttemptAt = millis() + STA_RETRY_INTERVAL_MS;
}

void configureOta() {
  preferences.begin("sma-monitor", false);
  otaPassword = preferences.getString("otaPass", "");
  if (otaPassword.length() < 12) {
    char generated[17];
    snprintf(generated, sizeof(generated), "%08lX%08lX",
             static_cast<unsigned long>(esp_random()), static_cast<unsigned long>(esp_random()));
    otaPassword = generated; preferences.putString("otaPass", otaPassword);
  }
  preferences.end();
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.setPassword(otaPassword.c_str());
  ArduinoOTA.onStart([] { otaBusy = true; addLog("[OTA] start"); });
  ArduinoOTA.onEnd([] { addLog("[OTA] complete; rebooting"); });
  ArduinoOTA.onError([](ota_error_t error) { otaBusy = false; addLog("[OTA] error=%u", static_cast<unsigned>(error)); });
  ArduinoOTA.begin();
  addLog("[OTA] ready hostname=%s password_configured=true", hostname.c_str());
  Serial.printf("[SECRET] OTA password=%s\n", otaPassword.c_str());
}
}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  bootAt = millis();
  const uint32_t identity = static_cast<uint32_t>(ESP.getEfuseMac());
  char suffix[5]; snprintf(suffix, sizeof(suffix), "%04X", identity & 0xFFFF);
  hostname = "sma-sunnyboy-monitor-" + String(suffix); hostname.toLowerCase();
  apSsid = "SMA-Monitor-" + String(suffix);
  apPassword = "SMAsetup-" + String(suffix);

  addLog("=== SMA SunnyBoy Monitor - Remote Dev Base ===");
  addLog("[SYSTEM] chip=%s revision=%u cores=%u cpu_mhz=%u flash=%u sdk=%s",
         ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz(),
         ESP.getFlashChipSize(), ESP.getSdkVersion());
  addLog("[HEAP] boot free=%u min=%u largest=%u", ESP.getFreeHeap(), ESP.getMinFreeHeap(),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  connectSta();
  registerRoutes(); server.begin(); addLog("[WEB] ready port=80");
  configureOta();
  bluetoothReady = serialBt.begin("SMA-SunnyBoy-Monitor", true);
  addLog("[BT] %s mode=master inquiry=classic", bluetoothReady ? "ready" : "initialization_failed");
  addLog("[HEAP] services_ready free=%u min=%u largest=%u", ESP.getFreeHeap(), ESP.getMinFreeHeap(),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

void loop() {
  const uint32_t now = millis();
  const uint32_t loopGap = now - lastLoopAt;
  if (lastLoopAt != 0 && loopGap > maxLoopGapMs) maxLoopGapMs = loopGap;
  lastLoopAt = now;
  server.handleClient();
  serviceBtScan();
  if (scanState != ScanState::SCANNING) ArduinoOTA.handle();
  const wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != lastWifiStatus) {
    addLog("[WIFI] status_change old=%d new=%d scan=%s", static_cast<int>(lastWifiStatus),
           static_cast<int>(wifiStatus), scanStateName(scanState));
    lastWifiStatus = wifiStatus;
  }
  if (WiFi.status() != WL_CONNECTED && static_cast<int32_t>(millis() - nextStaAttemptAt) >= 0) connectSta();
  delay(2);
}
