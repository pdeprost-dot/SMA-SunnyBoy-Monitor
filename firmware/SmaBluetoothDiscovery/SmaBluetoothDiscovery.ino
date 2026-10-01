#include <Arduino.h>
#include <ArduinoOTA.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <time.h>

#include "SmaBluetoothClient.h"
#include "SmaLocalConfig.h"
#include "MqttOutputService.h"
#include "ProductConfiguration.h"

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
constexpr time_t MIN_VALID_UNIX_TIME = 1704067200;  // 2024-01-01 UTC
constexpr uint32_t APP_SERIAL_MIN = 900000000UL;
constexpr uint32_t APP_SERIAL_RANGE = 100000000UL;
constexpr uint32_t RTC_BREADCRUMB_MAGIC = 0x534D4133UL;

enum class CrashCheckpoint : uint32_t {
  NONE, BOOT, BT_START_REQUESTED, BEFORE_BT_BEGIN, BT_BEGIN_ENTER,
  BT_BEGIN_RETURNED, AFTER_BT_BEGIN, BT_BEGIN_OK,
  RFCOMM_CONNECT_ENTER, RFCOMM_OPEN, SMA_SESSION_START, LOGIN_START,
  CLEANUP_START, BT_END_ENTER, BT_END_OK
};

RTC_NOINIT_ATTR uint32_t rtcBreadcrumbMagic;
RTC_NOINIT_ATTR uint32_t rtcCheckpoint;
RTC_NOINIT_ATTR uint32_t rtcCheckpointInverse;
RTC_NOINIT_ATTR uint32_t rtcAttempt;
uint32_t previousCheckpoint = 0;
uint32_t previousAttempt = 0;
esp_reset_reason_t bootResetReason = ESP_RST_UNKNOWN;

enum class ScanState : uint8_t { IDLE, SCANNING, COMPLETE, ERROR };
enum class SmaLifecycleState : uint8_t {
  BT_OFF, BT_STARTING, BT_READY, CONNECTING, SESSION, TRANSACTION,
  DISCONNECTING, BT_STOPPING, FAILED, BACKOFF
};
enum class BtOnlyTestState : uint8_t {
  IDLE, MQTT_SETTLE, BEGIN, WAIT_READY, READY_DELAY, RFCOMM_CONNECTING,
  RFCOMM_HOLD, RFCOMM_DISCONNECT, HOLD, END, COMPLETE, FAILED
};
enum class NetworkAuditState : uint8_t {
  IDLE, MQTT_SETTLE, OTA_MDNS_SETTLE, WEB_SETTLE, WIFI_SETTLE, WAIT_BT,
  WAIT_SMA, WAIT_MQTT, RESTART, COMPLETE, FAILED
};
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
volatile bool sppInitSeen = false;
BtOnlyTestState btOnlyState = BtOnlyTestState::IDLE;
bool btOnlyDisconnectMqtt = false;
bool btOnlyRfcomm = false;
bool btOnlyMqttWasConnected = false;
bool btOnlyHeapIntegrity = false;
volatile bool btOnlyConnectDone = false;
volatile bool btOnlyConnectResult = false;
volatile UBaseType_t btOnlyConnectStackHwm = 0;
TaskHandle_t btOnlyConnectTaskHandle = nullptr;
uint32_t btOnlyStateAt = 0;
uint8_t btOnlyTimelineStep = 0;
char btOnlyLastError[40]{};
NetworkAuditState networkAuditState = NetworkAuditState::IDLE;
uint32_t networkAuditStateAt = 0;
bool networkAuditWithBt = false;
bool networkAuditWifiOff = false;
bool networkAuditRfcomm = false;
bool networkAuditSma = false;
uint32_t networkAuditRestoreStartedAt = 0;
SmaLifecycleState smaLifecycleState = SmaLifecycleState::BT_OFF;
constexpr uint32_t SMA_FAILURE_BACKOFF_MS = 60000;
constexpr uint32_t SMA_DISCONNECT_GUARD_MS = 3000;
constexpr uint32_t SMA_DIAGNOSTIC_READY_DELAY_MS = 2000;
constexpr size_t PREVIOUS_BT_READY_INTERNAL_FREE = 9812;
constexpr size_t PREVIOUS_BT_READY_INTERNAL_LARGEST = 7156;
constexpr size_t RFCOMM_DIAGNOSTIC_MIN_GAIN = 2048;
uint32_t smaLastAttemptAt = 0;
uint32_t smaLastSuccessAt = 0;
uint32_t smaNextAttemptAt = 0;
uint32_t smaStopAt = 0;
uint32_t smaBtReadyAt = 0;
uint32_t smaSuccessfulTransactions = 0;
uint32_t smaFailedConnections = 0;
char smaLifecycleLastError[64]{};
String hostname;
String apSsid;
String apPassword;
String otaPassword;

void addLog(const char* format, ...);
void smaLogAdapter(const char* line);
void onSppEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t* parameter);
void onBtAuthComplete(bool success);
bool startSmaLifecycle();
void serviceSmaLifecycle();
void serviceBtOnlyTest();
void serviceNetworkAudit();
bool startBtOnlyTest(bool disconnectMqtt, bool rfcomm);
void connectSta();

SmaBluetoothClient smaClient(serialBt, smaLogAdapter);
MqttOutputService mqttOutput;
ProductConfig::Settings productSettings;
SbfspotCompat::InverterSnapshot inverterSnapshots[ProductConfig::kInverterCount];

time_t currentUnixTime() { return time(nullptr); }
bool timeSynchronized() { return currentUnixTime() >= MIN_VALID_UNIX_TIME; }

void refreshInverterSnapshot() {
  const time_t now = currentUnixTime();
  time_t sunrise = 0, sunset = 0;
  const bool haveSun = ProductConfig::calculateSunTimes(now, productSettings.latitude,
                                                         productSettings.longitude, sunrise, sunset);
  for (size_t slot = 0; slot < ProductConfig::kInverterCount; ++slot) {
    SbfspotCompat::InverterSnapshot& snapshot = inverterSnapshots[slot];
    ProductConfig::formatLocalTime(now, snapshot.timestamp, sizeof(snapshot.timestamp));
    if (haveSun) {
      ProductConfig::formatLocalTime(sunrise, snapshot.sunrise, sizeof(snapshot.sunrise));
      ProductConfig::formatLocalTime(sunset, snapshot.sunset, sizeof(snapshot.sunset));
    }
    snapshot.inverterSerial = productSettings.inverters[slot].serial;
    strlcpy(snapshot.inverterName, productSettings.inverters[slot].name, sizeof(snapshot.inverterName));
  }
  if (smaClient.acPowerValid()) {
    SbfspotCompat::InverterSnapshot& snapshot = inverterSnapshots[0];
    snapshot.acTotalPower.state = SbfspotCompat::ValueState::Valid;
    snapshot.acTotalPower.value = smaClient.acPowerW();
    snapshot.acquisitionValid = true;
    snapshot.lastSuccessfulAcquisition = smaClient.measurementTimestamp();
    // SBFspot semantics: InvSleepTm is the timestamp carried by LRI 0x263F.
    ProductConfig::formatLocalTime(smaClient.measurementTimestamp(), snapshot.inverterSleepTime,
                                   sizeof(snapshot.inverterSleepTime));
  }
}

void loadProductSettings() {
  preferences.begin("sma-monitor", false);
  String value = preferences.getString("apPass", "");
  if (ProductConfig::validWpaPassword(value.c_str())) strlcpy(productSettings.apPassword, value.c_str(), sizeof(productSettings.apPassword));
  productSettings.latitude = preferences.getDouble("latitude", 0);
  productSettings.longitude = preferences.getDouble("longitude", 0);
  value = preferences.getString("timezone", "CET-1CEST,M3.5.0,M10.5.0/3");
  if (!value.isEmpty() && value.length() < sizeof(productSettings.timezone))
    strlcpy(productSettings.timezone, value.c_str(), sizeof(productSettings.timezone));
  for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
    char key[12];
    snprintf(key, sizeof(key), "inv%uEn", static_cast<unsigned>(i + 1));
    productSettings.inverters[i].enabled = preferences.getBool(key, i == 0);
    snprintf(key, sizeof(key), "inv%uMac", static_cast<unsigned>(i + 1));
    value = preferences.getString(key, SmaLocalConfig::KNOWN_SMAS[i].mac);
    strlcpy(productSettings.inverters[i].mac, value.c_str(), sizeof(productSettings.inverters[i].mac));
    snprintf(key, sizeof(key), "inv%uSerial", static_cast<unsigned>(i + 1));
    productSettings.inverters[i].serial = preferences.getUInt(key, SmaLocalConfig::KNOWN_SMAS[i].serial);
    snprintf(productSettings.inverters[i].name, sizeof(productSettings.inverters[i].name), "INV%u",
             static_cast<unsigned>(i + 1));
  }
  preferences.end();
  setenv("TZ", productSettings.timezone, 1); tzset();
}

const char* checkpointName(uint32_t value) {
  switch (static_cast<CrashCheckpoint>(value)) {
    case CrashCheckpoint::NONE: return "NONE";
    case CrashCheckpoint::BOOT: return "BOOT";
    case CrashCheckpoint::BT_START_REQUESTED: return "BT_START_REQUESTED";
    case CrashCheckpoint::BEFORE_BT_BEGIN: return "BEFORE_BT_BEGIN";
    case CrashCheckpoint::BT_BEGIN_ENTER: return "BT_BEGIN_ENTER";
    case CrashCheckpoint::BT_BEGIN_RETURNED: return "BT_BEGIN_RETURNED";
    case CrashCheckpoint::AFTER_BT_BEGIN: return "AFTER_BT_BEGIN";
    case CrashCheckpoint::BT_BEGIN_OK: return "BT_BEGIN_OK";
    case CrashCheckpoint::RFCOMM_CONNECT_ENTER: return "RFCOMM_CONNECT_ENTER";
    case CrashCheckpoint::RFCOMM_OPEN: return "RFCOMM_OPEN";
    case CrashCheckpoint::SMA_SESSION_START: return "SMA_SESSION_START";
    case CrashCheckpoint::LOGIN_START: return "LOGIN_START";
    case CrashCheckpoint::CLEANUP_START: return "CLEANUP_START";
    case CrashCheckpoint::BT_END_ENTER: return "BT_END_ENTER";
    case CrashCheckpoint::BT_END_OK: return "BT_END_OK";
  }
  return "UNKNOWN";
}

const char* resetReasonName(esp_reset_reason_t value) {
  switch (value) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT_RESET";
    case ESP_RST_SW: return "SW_RESET";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    case ESP_RST_EFUSE: return "EFUSE";
    case ESP_RST_PWR_GLITCH: return "POWER_GLITCH";
    case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
    case ESP_RST_UNKNOWN: default: return "UNKNOWN";
  }
}

void recordCheckpoint(CrashCheckpoint value) {
  rtcBreadcrumbMagic = RTC_BREADCRUMB_MAGIC;
  rtcCheckpoint = static_cast<uint32_t>(value);
  rtcCheckpointInverse = ~rtcCheckpoint;
}

void beginDiagnosticAttempt() {
  rtcBreadcrumbMagic = RTC_BREADCRUMB_MAGIC;
  ++rtcAttempt;
  rtcCheckpoint = static_cast<uint32_t>(CrashCheckpoint::BT_START_REQUESTED);
  rtcCheckpointInverse = ~rtcCheckpoint;
}

const char* scanStateName(ScanState value) {
  switch (value) {
    case ScanState::IDLE: return "IDLE";
    case ScanState::SCANNING: return "SCANNING";
    case ScanState::COMPLETE: return "COMPLETE";
    case ScanState::ERROR: return "ERROR";
  }
  return "ERROR";
}

const char* smaLifecycleStateName(SmaLifecycleState value) {
  switch (value) {
    case SmaLifecycleState::BT_OFF: return "BT_OFF";
    case SmaLifecycleState::BT_STARTING: return "BT_STARTING";
    case SmaLifecycleState::BT_READY: return "BT_READY";
    case SmaLifecycleState::CONNECTING: return "CONNECTING";
    case SmaLifecycleState::SESSION: return "SESSION";
    case SmaLifecycleState::TRANSACTION: return "TRANSACTION";
    case SmaLifecycleState::DISCONNECTING: return "DISCONNECTING";
    case SmaLifecycleState::BT_STOPPING: return "BT_STOPPING";
    case SmaLifecycleState::FAILED: return "FAILED";
    case SmaLifecycleState::BACKOFF: return "BACKOFF";
  }
  return "BT_OFF";
}

const char* btOnlyStateName(BtOnlyTestState value) {
  switch (value) {
    case BtOnlyTestState::IDLE: return "IDLE";
    case BtOnlyTestState::MQTT_SETTLE: return "MQTT_SETTLE";
    case BtOnlyTestState::BEGIN: return "BEGIN";
    case BtOnlyTestState::WAIT_READY: return "WAIT_READY";
    case BtOnlyTestState::READY_DELAY: return "READY_DELAY";
    case BtOnlyTestState::RFCOMM_CONNECTING: return "RFCOMM_CONNECTING";
    case BtOnlyTestState::RFCOMM_HOLD: return "RFCOMM_HOLD";
    case BtOnlyTestState::RFCOMM_DISCONNECT: return "RFCOMM_DISCONNECT";
    case BtOnlyTestState::HOLD: return "HOLD";
    case BtOnlyTestState::END: return "END";
    case BtOnlyTestState::COMPLETE: return "COMPLETE";
    case BtOnlyTestState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

void setSmaLifecycleState(SmaLifecycleState next) {
  if (smaLifecycleState == next) return;
  addLog("[SMA-LIFECYCLE] %s -> %s", smaLifecycleStateName(smaLifecycleState),
         smaLifecycleStateName(next));
  smaLifecycleState = next;
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

void smaLogAdapter(const char* line) {
  addLog("%s", line);
}

void onBtAuthComplete(bool success) {
  Serial.printf("[GAP] auth_complete t=%lu success=%s\n",
                static_cast<unsigned long>(millis()), success ? "true" : "false");
}

void onSppEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t* parameter) {
  if (parameter == nullptr) return;
  switch (event) {
    case ESP_SPP_INIT_EVT:
      sppInitSeen = parameter->init.status == ESP_SPP_SUCCESS;
      addLog("[SPP] init t=%lu status=%d", static_cast<unsigned long>(millis()),
             parameter->init.status);
      break;
    case ESP_SPP_UNINIT_EVT:
      addLog("[SPP] uninit t=%lu status=%d", static_cast<unsigned long>(millis()),
             parameter->uninit.status);
      break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
      addLog("[SPP] discovery t=%lu status=%d channels=%u first=%u", static_cast<unsigned long>(millis()),
             parameter->disc_comp.status,
             parameter->disc_comp.scn_num,
             parameter->disc_comp.scn_num ? parameter->disc_comp.scn[0] : 0);
      break;
    case ESP_SPP_OPEN_EVT:
      recordCheckpoint(CrashCheckpoint::RFCOMM_OPEN);
      addLog("[SPP] open t=%lu status=%d handle=%lu bda=%02X:%02X:%02X:%02X:%02X:%02X",
             static_cast<unsigned long>(millis()), parameter->open.status,
             static_cast<unsigned long>(parameter->open.handle), parameter->open.rem_bda[0],
             parameter->open.rem_bda[1], parameter->open.rem_bda[2], parameter->open.rem_bda[3],
             parameter->open.rem_bda[4], parameter->open.rem_bda[5]);
      break;
    case ESP_SPP_CLOSE_EVT:
      addLog("[SPP] close t=%lu status=%d port_status=%lu handle=%lu async=%s",
             static_cast<unsigned long>(millis()), parameter->close.status,
             static_cast<unsigned long>(parameter->close.port_status),
             static_cast<unsigned long>(parameter->close.handle), parameter->close.async ? "true" : "false");
      break;
    case ESP_SPP_CL_INIT_EVT:
      addLog("[SPP] client_init t=%lu status=%d handle=%lu sec_id=%u use_co=%s",
             static_cast<unsigned long>(millis()), parameter->cl_init.status,
             static_cast<unsigned long>(parameter->cl_init.handle), parameter->cl_init.sec_id,
             parameter->cl_init.use_co ? "true" : "false");
      break;
    case ESP_SPP_CONG_EVT:
      addLog("[SPP] congestion t=%lu status=%d handle=%lu congested=%s",
             static_cast<unsigned long>(millis()), parameter->cong.status,
             static_cast<unsigned long>(parameter->cong.handle), parameter->cong.cong ? "true" : "false");
      break;
    case ESP_SPP_WRITE_EVT:
      if (parameter->write.status != ESP_SPP_SUCCESS || parameter->write.cong) {
        addLog("[SPP] write t=%lu status=%d handle=%lu len=%d congested=%s",
               static_cast<unsigned long>(millis()), parameter->write.status,
               static_cast<unsigned long>(parameter->write.handle), parameter->write.len,
               parameter->write.cong ? "true" : "false");
      }
      break;
    default:
      break;
  }
}

void stopBluetoothService() {
  if (bluetoothReady) {
    addLog("[BT] stopping");
    recordCheckpoint(CrashCheckpoint::BT_END_ENTER);
    serialBt.end();
    bluetoothReady = false;
    recordCheckpoint(CrashCheckpoint::BT_END_OK);
  }
  logMemory("bt_off");
}

void logBtStartResources(const char* stage) {
  btOnlyHeapIntegrity = heap_caps_check_integrity_all(true);
  addLog("[BT-ONLY] %s free=%u min=%u largest=%u internal=%u internal_largest=%u dma=%u mqtt=%s wifi=%d loop_hwm=%u integrity=%s",
         stage, ESP.getFreeHeap(), ESP.getMinFreeHeap(),
         heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
         heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
         heap_caps_get_free_size(MALLOC_CAP_DMA), mqttOutput.connected() ? "connected" : "disconnected",
         static_cast<int>(WiFi.status()), static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
         btOnlyHeapIntegrity ? "ok" : "failed");
}

void logTaskAudit(const char* stage) {
  TaskStatus_t tasks[32];
  uint32_t totalRunTime = 0;
  const UBaseType_t count = uxTaskGetSystemState(tasks, 32, &totalRunTime);
  addLog("[TASKS] stage=%s count=%u", stage, static_cast<unsigned>(count));
  for (UBaseType_t i = 0; i < count; ++i) {
    addLog("[TASK] name=%s prio=%u base=%u core=%d hwm=%u state=%u",
           tasks[i].pcTaskName, static_cast<unsigned>(tasks[i].uxCurrentPriority),
           static_cast<unsigned>(tasks[i].uxBasePriority), static_cast<int>(tasks[i].xCoreID),
           static_cast<unsigned>(tasks[i].usStackHighWaterMark),
           static_cast<unsigned>(tasks[i].eCurrentState));
  }
}

bool startNetworkAudit(bool withBt, bool wifiOff = false, bool rfcomm = false) {
  if (networkAuditState != NetworkAuditState::IDLE && networkAuditState != NetworkAuditState::COMPLETE &&
      networkAuditState != NetworkAuditState::FAILED) return false;
  if (bluetoothReady || smaLifecycleState != SmaLifecycleState::BT_OFF) return false;
  networkAuditWithBt = withBt;
  networkAuditWifiOff = wifiOff;
  networkAuditRfcomm = rfcomm;
  networkAuditSma = false;
  logBtStartResources("network_normal");
  mqttOutput.disconnect();
  networkAuditStateAt = millis();
  networkAuditState = NetworkAuditState::MQTT_SETTLE;
  addLog("[NET-AUDIT] started with_bt=%s wifi_off=%s", withBt ? "true" : "false",
         wifiOff ? "true" : "false");
  return true;
}

bool startSafeSmaAcquisition() {
  if (!startNetworkAudit(true, true, false)) return false;
  networkAuditSma = true;
  addLog("[ACQUIRE] safe SMA PACTot cycle requested");
  return true;
}

void restartNetworkAuditServices() {
  server.begin();
  ArduinoOTA.begin();
  addLog("[NET-AUDIT] services_restarted web=true ota=true mdns=true");
  networkAuditState = NetworkAuditState::WAIT_MQTT;
}

void serviceNetworkAudit() {
  const uint32_t now = millis();
  switch (networkAuditState) {
    case NetworkAuditState::MQTT_SETTLE:
      if (now - networkAuditStateAt >= 2000) {
        logBtStartResources("network_mqtt_off");
        ArduinoOTA.end();  // ArduinoOTA 3.3.11 also calls MDNS.end().
        networkAuditStateAt = now;
        networkAuditState = NetworkAuditState::OTA_MDNS_SETTLE;
      }
      break;
    case NetworkAuditState::OTA_MDNS_SETTLE:
      if (now - networkAuditStateAt >= 2000) {
        logBtStartResources("network_ota_mdns_off");
        logTaskAudit("ota_mdns_off");
        server.stop();
        networkAuditStateAt = now;
        networkAuditState = NetworkAuditState::WEB_SETTLE;
      }
      break;
    case NetworkAuditState::WEB_SETTLE:
      if (now - networkAuditStateAt >= 2000) {
        logBtStartResources("network_web_off");
        if (networkAuditWifiOff) {
          const bool stopped = WiFi.disconnect(true, false);
          addLog("[NET-AUDIT] wifi_stop result=%s", stopped ? "true" : "false");
          networkAuditStateAt = now;
          networkAuditState = NetworkAuditState::WIFI_SETTLE;
        } else if (networkAuditWithBt) {
          if (!startBtOnlyTest(false, false)) {
            networkAuditState = NetworkAuditState::FAILED;
            restartNetworkAuditServices();
          } else {
            networkAuditState = NetworkAuditState::WAIT_BT;
          }
        } else {
          restartNetworkAuditServices();
        }
      }
      break;
    case NetworkAuditState::WIFI_SETTLE:
      if (now - networkAuditStateAt >= 3000) {
        logBtStartResources("network_wifi_off");
        logTaskAudit("wifi_off");
        if (networkAuditSma) {
          if (startSmaLifecycle()) {
            networkAuditState = NetworkAuditState::WAIT_SMA;
          } else {
            networkAuditState = NetworkAuditState::FAILED;
            networkAuditRestoreStartedAt = millis();
            connectSta();
            restartNetworkAuditServices();
          }
        } else if (!startBtOnlyTest(false, networkAuditRfcomm)) {
          networkAuditState = NetworkAuditState::FAILED;
          networkAuditRestoreStartedAt = millis();
          connectSta();
          addLog("[NET-AUDIT] wifi_ip_restore_ms=%lu connected=%s",
                 static_cast<unsigned long>(millis() - networkAuditRestoreStartedAt),
                 WiFi.status() == WL_CONNECTED ? "true" : "false");
          restartNetworkAuditServices();
        } else {
          networkAuditState = NetworkAuditState::WAIT_BT;
        }
      }
      break;
    case NetworkAuditState::WAIT_SMA:
      if (smaLifecycleState == SmaLifecycleState::BT_OFF ||
          smaLifecycleState == SmaLifecycleState::BACKOFF) {
        networkAuditRestoreStartedAt = millis();
        connectSta();
        addLog("[NET-AUDIT] wifi_ip_restore_ms=%lu connected=%s",
               static_cast<unsigned long>(millis() - networkAuditRestoreStartedAt),
               WiFi.status() == WL_CONNECTED ? "true" : "false");
        restartNetworkAuditServices();
      }
      break;
    case NetworkAuditState::WAIT_BT:
      if (btOnlyState == BtOnlyTestState::COMPLETE || btOnlyState == BtOnlyTestState::FAILED) {
        if (networkAuditWifiOff) {
          networkAuditRestoreStartedAt = millis();
          connectSta();
          addLog("[NET-AUDIT] wifi_ip_restore_ms=%lu connected=%s",
                 static_cast<unsigned long>(millis() - networkAuditRestoreStartedAt),
                 WiFi.status() == WL_CONNECTED ? "true" : "false");
        }
        restartNetworkAuditServices();
      }
      break;
    case NetworkAuditState::WAIT_MQTT:
      if (mqttOutput.connected()) {
        addLog("[NET-AUDIT] mqtt_restore_ms=%lu",
               static_cast<unsigned long>(millis() - networkAuditRestoreStartedAt));
        networkAuditState = NetworkAuditState::COMPLETE;
      }
      break;
    default: break;
  }
}

void btOnlyConnectTask(void*) {
  recordCheckpoint(CrashCheckpoint::RFCOMM_CONNECT_ENTER);
  addLog("[BT-ONLY] rfcomm_connect_enter t=%lu channel=1",
         static_cast<unsigned long>(millis()));
  uint8_t target[6];
  memcpy(target, SmaLocalConfig::TARGET_CONNECT_ADDRESS, sizeof(target));
  btOnlyConnectResult = serialBt.connect(target, 1);
  btOnlyConnectStackHwm = uxTaskGetStackHighWaterMark(nullptr);
  btOnlyConnectDone = true;
  btOnlyConnectTaskHandle = nullptr;
  vTaskDelete(nullptr);
}

bool startBtOnlyTest(bool disconnectMqtt, bool rfcomm = false) {
  if (btOnlyState != BtOnlyTestState::IDLE && btOnlyState != BtOnlyTestState::COMPLETE &&
      btOnlyState != BtOnlyTestState::FAILED) return false;
  if (smaLifecycleState != SmaLifecycleState::BT_OFF || bluetoothReady) return false;
  beginDiagnosticAttempt();
  recordCheckpoint(CrashCheckpoint::BEFORE_BT_BEGIN);
  btOnlyDisconnectMqtt = disconnectMqtt;
  btOnlyRfcomm = rfcomm;
  btOnlyMqttWasConnected = mqttOutput.connected();
  btOnlyConnectDone = false;
  btOnlyConnectResult = false;
  btOnlyConnectStackHwm = 0;
  btOnlyTimelineStep = 0;
  btOnlyLastError[0] = 0;
  sppInitSeen = false;
  if (disconnectMqtt) {
    logBtStartResources("normal_mqtt_connected_bt_off");
    if (!rfcomm) logTaskAudit("bt_off");
    mqttOutput.disconnect();
    btOnlyState = BtOnlyTestState::MQTT_SETTLE;
    btOnlyStateAt = millis();
    addLog("[BT-ONLY] requested mode=%c mqtt_disconnected=true", rfcomm ? 'C' : 'B');
  } else {
    btOnlyState = BtOnlyTestState::BEGIN;
    addLog("[BT-ONLY] requested mode=A mqtt_connected=%s", mqttOutput.connected() ? "true" : "false");
  }
  return true;
}

void failBtOnlyTest(const char* error) {
  strlcpy(btOnlyLastError, error, sizeof(btOnlyLastError));
  if (bluetoothReady) stopBluetoothService();
  btOnlyState = BtOnlyTestState::FAILED;
  addLog("[BT-ONLY] failed error=%s", btOnlyLastError);
}

void serviceBtOnlyTest() {
  const uint32_t now = millis();
  switch (btOnlyState) {
    case BtOnlyTestState::MQTT_SETTLE:
      if (now - btOnlyStateAt >= 2000) btOnlyState = BtOnlyTestState::BEGIN;
      break;
    case BtOnlyTestState::BEGIN:
      recordCheckpoint(CrashCheckpoint::BEFORE_BT_BEGIN);
      logBtStartResources("before_begin");
      recordCheckpoint(CrashCheckpoint::BT_BEGIN_ENTER);
      bluetoothReady = serialBt.begin("SMA-SunnyBoy-Monitor", true, true);
      recordCheckpoint(CrashCheckpoint::BT_BEGIN_RETURNED);
      addLog("[BT-ONLY] begin_returned result=%s", bluetoothReady ? "true" : "false");
      if (!bluetoothReady) { failBtOnlyTest("bt_begin_false"); break; }
      logBtStartResources("begin_return");
      serialBt.setPin("0000", 4);
      serialBt.onAuthComplete(onBtAuthComplete);
      serialBt.register_callback(onSppEvent);
      recordCheckpoint(CrashCheckpoint::AFTER_BT_BEGIN);
      btOnlyStateAt = now;
      btOnlyState = BtOnlyTestState::WAIT_READY;
      break;
    case BtOnlyTestState::WAIT_READY:
      if (sppInitSeen || serialBt.isReady(false, 0)) {
        addLog("[BT-ONLY] spp_ready t=%lu", static_cast<unsigned long>(now));
        logBtStartResources("spp_init");
        if (btOnlyRfcomm) {
          const size_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          const size_t internalLargest =
              heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          const bool materiallyImproved =
              internalFree >= PREVIOUS_BT_READY_INTERNAL_FREE + RFCOMM_DIAGNOSTIC_MIN_GAIN &&
              internalLargest >= PREVIOUS_BT_READY_INTERNAL_LARGEST + RFCOMM_DIAGNOSTIC_MIN_GAIN;
          addLog("[BT-ONLY] rfcomm_gate gain_free=%d gain_largest=%d result=%s",
                 static_cast<int>(internalFree) - static_cast<int>(PREVIOUS_BT_READY_INTERNAL_FREE),
                 static_cast<int>(internalLargest) - static_cast<int>(PREVIOUS_BT_READY_INTERNAL_LARGEST),
                 materiallyImproved ? "GO" : "NO_GO");
          if (!materiallyImproved || !btOnlyHeapIntegrity) {
            failBtOnlyTest(btOnlyHeapIntegrity ? "rfcomm_memory_no_go" : "heap_integrity_failed");
            break;
          }
        }
        btOnlyStateAt = now;
        btOnlyTimelineStep = 0;
        btOnlyState = btOnlyRfcomm ? BtOnlyTestState::READY_DELAY : BtOnlyTestState::HOLD;
      } else if (now - btOnlyStateAt >= 10000) failBtOnlyTest("spp_ready_timeout");
      break;
    case BtOnlyTestState::READY_DELAY:
      if (now - btOnlyStateAt >= SMA_DIAGNOSTIC_READY_DELAY_MS) {
        logBtStartResources("before_rfcomm");
        if (xTaskCreatePinnedToCore(btOnlyConnectTask, "bt-only-connect", 4096, nullptr, 1,
                                    &btOnlyConnectTaskHandle, 0) != pdPASS) {
          failBtOnlyTest("connect_task_create_failed");
          break;
        }
        btOnlyStateAt = now;
        btOnlyState = BtOnlyTestState::RFCOMM_CONNECTING;
      }
      break;
    case BtOnlyTestState::RFCOMM_CONNECTING:
      if (btOnlyConnectDone) {
        addLog("[BT-ONLY] rfcomm_connect_return t=%lu result=%s stack_hwm=%u",
               static_cast<unsigned long>(now), btOnlyConnectResult ? "true" : "false",
               static_cast<unsigned>(btOnlyConnectStackHwm));
        logBtStartResources(btOnlyConnectResult ? "rfcomm_open" : "rfcomm_failed");
        btOnlyStateAt = now;
        btOnlyState = btOnlyConnectResult ? BtOnlyTestState::RFCOMM_HOLD : BtOnlyTestState::END;
      } else if (now - btOnlyStateAt >= 15000) {
        failBtOnlyTest("rfcomm_task_timeout");
      }
      break;
    case BtOnlyTestState::RFCOMM_HOLD:
      if (now - btOnlyStateAt >= 250) btOnlyState = BtOnlyTestState::RFCOMM_DISCONNECT;
      break;
    case BtOnlyTestState::RFCOMM_DISCONNECT:
      addLog("[BT-ONLY] rfcomm_disconnect result=%s",
             serialBt.disconnect() ? "true" : "false");
      logBtStartResources("after_rfcomm_disconnect");
      btOnlyStateAt = now;
      btOnlyState = BtOnlyTestState::HOLD;
      break;
    case BtOnlyTestState::HOLD:
      if (!btOnlyRfcomm) {
        static constexpr uint32_t kTimelineMs[] = {250, 500, 1000, 2000, 5000};
        static constexpr const char* kTimelineNames[] = {"spp_250ms", "spp_500ms", "spp_1000ms",
                                                         "spp_2000ms", "spp_5000ms"};
        if (btOnlyTimelineStep < 5 && now - btOnlyStateAt >= kTimelineMs[btOnlyTimelineStep]) {
          logBtStartResources(kTimelineNames[btOnlyTimelineStep]);
          if (btOnlyTimelineStep == 3) logTaskAudit("spp_2000ms");
          ++btOnlyTimelineStep;
        }
        if (btOnlyTimelineStep >= 5) btOnlyState = BtOnlyTestState::END;
      } else if (now - btOnlyStateAt >= 5000) {
        btOnlyState = BtOnlyTestState::END;
      }
      break;
    case BtOnlyTestState::END:
      logBtStartResources("before_end");
      stopBluetoothService();
      logBtStartResources("after_end");
      btOnlyState = BtOnlyTestState::COMPLETE;
      addLog("[BT-ONLY] complete mode=%c mqtt_was_connected=%s",
             btOnlyRfcomm ? 'C' : (btOnlyDisconnectMqtt ? 'B' : 'A'),
             btOnlyMqttWasConnected ? "true" : "false");
      break;
    default: break;
  }
}

void failSmaLifecycle(const char* error) {
  recordCheckpoint(CrashCheckpoint::CLEANUP_START);
  strlcpy(smaLifecycleLastError, error && error[0] ? error : "unknown", sizeof(smaLifecycleLastError));
  ++smaFailedConnections;
  setSmaLifecycleState(SmaLifecycleState::FAILED);
  if (smaClient.state() != SmaBluetoothClient::State::DISCONNECTED) smaClient.requestDisconnect();
  setSmaLifecycleState(SmaLifecycleState::BT_STOPPING);
  stopBluetoothService();
  smaNextAttemptAt = millis() + SMA_FAILURE_BACKOFF_MS;
  setSmaLifecycleState(SmaLifecycleState::BACKOFF);
  addLog("[SMA-LIFECYCLE] failure=%s backoff_ms=%lu", smaLifecycleLastError,
         static_cast<unsigned long>(SMA_FAILURE_BACKOFF_MS));
}

bool startSmaLifecycle() {
  const uint32_t now = millis();
  if (btOnlyState != BtOnlyTestState::IDLE && btOnlyState != BtOnlyTestState::COMPLETE &&
      btOnlyState != BtOnlyTestState::FAILED) return false;
  if (!timeSynchronized()) {
    addLog("[TIME] SMA start rejected synchronized=false unix=%lld",
           static_cast<long long>(currentUnixTime()));
    return false;
  }
  if (smaLifecycleState == SmaLifecycleState::BACKOFF &&
      static_cast<int32_t>(now - smaNextAttemptAt) < 0) return false;
  if (smaLifecycleState != SmaLifecycleState::BT_OFF && smaLifecycleState != SmaLifecycleState::BACKOFF) return false;

  smaLastAttemptAt = now;
  beginDiagnosticAttempt();
  smaLifecycleLastError[0] = 0;
  smaNextAttemptAt = 0;
  setSmaLifecycleState(SmaLifecycleState::BT_STARTING);
  recordCheckpoint(CrashCheckpoint::BT_BEGIN_ENTER);
  bluetoothReady = serialBt.begin("SMA-SunnyBoy-Monitor", true, true);
  if (!bluetoothReady) {
    failSmaLifecycle("bt_begin_failed");
    return false;
  }
  serialBt.setPin("0000", 4);
  serialBt.onAuthComplete(onBtAuthComplete);
  serialBt.register_callback(onSppEvent);
  recordCheckpoint(CrashCheckpoint::BT_BEGIN_OK);
  addLog("[BT] begin accepted");
  return true;
}

void serviceSmaLifecycle() {
  const uint32_t now = millis();
  switch (smaLifecycleState) {
    case SmaLifecycleState::BT_STARTING:
      if (serialBt.isReady(false, 0)) {
        smaBtReadyAt = now;
        addLog("[BT-TIMELINE] bt_ready t=%lu diagnostic_delay_ms=%lu",
               static_cast<unsigned long>(now),
               static_cast<unsigned long>(SMA_DIAGNOSTIC_READY_DELAY_MS));
        setSmaLifecycleState(SmaLifecycleState::BT_READY);
      } else if (now - smaLastAttemptAt >= 10000) {
        failSmaLifecycle("bt_ready_timeout");
      }
      break;
    case SmaLifecycleState::BT_READY:
      if (now - smaBtReadyAt < SMA_DIAGNOSTIC_READY_DELAY_MS) break;
      addLog("[BT-TIMELINE] readiness_delay_complete t=%lu elapsed_ms=%lu",
             static_cast<unsigned long>(now),
             static_cast<unsigned long>(now - smaBtReadyAt));
      recordCheckpoint(CrashCheckpoint::RFCOMM_CONNECT_ENTER);
      if (smaClient.requestConnect()) setSmaLifecycleState(SmaLifecycleState::CONNECTING);
      else failSmaLifecycle("connect_request_rejected");
      break;
    case SmaLifecycleState::CONNECTING:
      if (smaClient.sessionReady()) {
        recordCheckpoint(CrashCheckpoint::SMA_SESSION_START);
        setSmaLifecycleState(SmaLifecycleState::SESSION);
        recordCheckpoint(CrashCheckpoint::LOGIN_START);
        if (smaClient.startPhase3Transaction()) setSmaLifecycleState(SmaLifecycleState::TRANSACTION);
        else failSmaLifecycle("phase3_login_tx_failed");
      } else if (smaClient.state() == SmaBluetoothClient::State::ERROR) {
        failSmaLifecycle(smaClient.lastError());
      }
      break;
    case SmaLifecycleState::TRANSACTION:
      if (smaClient.phase3Complete()) {
        ++smaSuccessfulTransactions;
        smaLastSuccessAt = now;
        recordCheckpoint(CrashCheckpoint::CLEANUP_START);
        if (!smaClient.requestDisconnect()) {
          failSmaLifecycle("disconnect_failed");
          break;
        }
        smaStopAt = millis() + SMA_DISCONNECT_GUARD_MS;
        setSmaLifecycleState(SmaLifecycleState::DISCONNECTING);
      } else if (smaClient.state() == SmaBluetoothClient::State::ERROR) {
        failSmaLifecycle(smaClient.lastError());
      }
      break;
    case SmaLifecycleState::DISCONNECTING:
      if (static_cast<int32_t>(now - smaStopAt) >= 0) {
        setSmaLifecycleState(SmaLifecycleState::BT_STOPPING);
        stopBluetoothService();
        setSmaLifecycleState(SmaLifecycleState::BT_OFF);
      }
      break;
    case SmaLifecycleState::BACKOFF:
      if (static_cast<int32_t>(now - smaNextAttemptAt) >= 0) {
        smaNextAttemptAt = 0;
        setSmaLifecycleState(SmaLifecycleState::BT_OFF);
      }
      break;
    default:
      break;
  }
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
  for (size_t index = 0; index < std::size(SmaLocalConfig::KNOWN_SMAS); ++index) {
    if (strcasecmp(mac, SmaLocalConfig::KNOWN_SMAS[index].mac) == 0) return static_cast<int8_t>(index);
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
    const SmaLocalConfig::Device& sma = SmaLocalConfig::KNOWN_SMAS[incoming.knownIndex];
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
  if (!bluetoothReady || otaBusy || scanState == ScanState::SCANNING ||
      smaClient.state() != SmaBluetoothClient::State::DISCONNECTED) return false;
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
      const SmaLocalConfig::Device& sma = SmaLocalConfig::KNOWN_SMAS[item.knownIndex];
      json += F("{\"label\":\""); json += sma.label; json += F("\",\"serial\":"); json += sma.serial; json += '}';
    } else json += F("null");
    json += '}';
  }
  json += F("]}");
  return json;
}

String statusJson() {
  String json;
  json.reserve(1200);
  json += F("{\"uptimeMs\":"); json += millis() - bootAt;
  json += F(",\"system\":{\"lastResetReason\":\""); json += resetReasonName(bootResetReason);
  json += F("\",\"lastResetCode\":"); json += static_cast<unsigned>(bootResetReason);
  json += F(",\"previousLifecycleCheckpoint\":\""); json += checkpointName(previousCheckpoint);
  json += F("\",\"previousAttempt\":"); json += previousAttempt;
  json += F(",\"currentLifecycleCheckpoint\":\""); json += checkpointName(rtcCheckpoint);
  json += F("\",\"currentAttempt\":"); json += rtcAttempt; json += F("}");
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
  json += F("},\"mqtt\":{\"mqttEnabled\":"); json += mqttOutput.config().enabled ? F("true") : F("false");
  json += F(",\"mqttConfigured\":"); json += mqttOutput.configured() ? F("true") : F("false");
  json += F(",\"mqttConnected\":"); json += mqttOutput.connected() ? F("true") : F("false");
  json += F(",\"broker\":\""); json += jsonEscape(mqttOutput.config().broker); json += '"';
  json += F(",\"port\":"); json += mqttOutput.config().port;
  json += F(",\"topicPrefix\":\""); json += jsonEscape(mqttOutput.config().topicPrefix); json += '"';
  json += F(",\"publishIntervalSeconds\":"); json += mqttOutput.config().publishIntervalSeconds;
  json += F(",\"lastConnectAttempt\":"); json += mqttOutput.lastConnectAttempt();
  json += F(",\"lastConnectResult\":"); json += mqttOutput.lastConnectResult();
  json += F(",\"reconnectCount\":"); json += mqttOutput.reconnectCount();
  json += F(",\"publishCount\":"); json += mqttOutput.publishCount();
  json += F(",\"publishFailures\":"); json += mqttOutput.publishFailures();
  json += F(",\"lastPublishMs\":"); json += mqttOutput.lastPublishMs();
  json += F(",\"lastPayloadBytes\":"); json += static_cast<unsigned>(mqttOutput.lastPayloadBytes());
  json += F("},\"btOnlyTest\":{\"state\":\""); json += btOnlyStateName(btOnlyState);
  json += F("\",\"mode\":\""); json += btOnlyRfcomm ? "C" : (btOnlyDisconnectMqtt ? "B" : "A");
  json += F("\",\"sppInitSeen\":"); json += sppInitSeen ? F("true") : F("false");
  json += F(",\"heapIntegrity\":"); json += btOnlyHeapIntegrity ? F("true") : F("false");
  json += F(",\"lastError\":\""); json += jsonEscape(btOnlyLastError); json += '"';
  json += F("},\"ota\":{\"busy\":"); json += otaBusy ? F("true") : F("false"); json += F("}}");
  return json;
}

String dashboardJson() {
  String json(F("{\"inverters\":["));
  json.reserve(1700);
  for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
    if (i) json += ',';
    const auto& cfg = productSettings.inverters[i];
    const auto& s = inverterSnapshots[i];
    json += F("{\"slot\":"); json += static_cast<unsigned>(i + 1);
    json += F(",\"name\":\""); json += cfg.name; json += '"';
    json += F(",\"enabled\":"); json += cfg.enabled ? F("true") : F("false");
    json += F(",\"serial\":"); json += cfg.serial;
    json += F(",\"dataValid\":"); json += s.acquisitionValid ? F("true") : F("false");
    json += F(",\"lastSuccess\":");
    if (s.lastSuccessfulAcquisition) { char formatted[24]{}; ProductConfig::formatLocalTime(s.lastSuccessfulAcquisition, formatted, sizeof(formatted)); json += '"'; json += formatted; json += '"'; }
    else json += F("null");
#define DASH_VALUE(key, member, divisor) do { json += F(",\"" key "\":"); if (s.member.state == SbfspotCompat::ValueState::Valid) json += static_cast<double>(s.member.value) / divisor; else json += F("null"); } while (0)
    DASH_VALUE("uac", acVoltage1, 100.0);
    DASH_VALUE("iac", acCurrent1, 1000.0);
    DASH_VALUE("pac", acTotalPower, 1.0);
    DASH_VALUE("eToday", todayEnergy, 1000.0);
    DASH_VALUE("eTotal", totalEnergy, 1000.0);
    DASH_VALUE("udc", dcVoltage1, 100.0);
    DASH_VALUE("idc", dcCurrent1, 1000.0);
    DASH_VALUE("pdc", dcPower1, 1.0);
    DASH_VALUE("frequency", gridFrequency, 100.0);
    DASH_VALUE("temperature", inverterTemperature, 100.0);
#undef DASH_VALUE
    json += F(",\"status\":"); if (s.inverterStatus[0] && strcmp(s.inverterStatus, "Information not available")) { json += '"'; json += jsonEscape(s.inverterStatus); json += '"'; } else json += F("null");
    json += F(",\"gridRelay\":"); if (s.inverterGridRelay[0] && strcmp(s.inverterGridRelay, "Information not available")) { json += '"'; json += jsonEscape(s.inverterGridRelay); json += '"'; } else json += F("null");
    json += '}';
  }
  json += F("],\"location\":{\"latitude\":"); json += String(productSettings.latitude, 6);
  json += F(",\"longitude\":"); json += String(productSettings.longitude, 6);
  json += F(",\"timezone\":\""); json += jsonEscape(productSettings.timezone); json += F("\"}}");
  return json;
}

String publicConfigJson() {
  String json(F("{\"latitude\":")); json.reserve(900);
  json += String(productSettings.latitude, 6); json += F(",\"longitude\":"); json += String(productSettings.longitude, 6);
  json += F(",\"timezone\":\""); json += jsonEscape(productSettings.timezone);
  json += F("\",\"mqtt\":{\"enabled\":"); json += mqttOutput.config().enabled ? F("true") : F("false");
  json += F(",\"broker\":\""); json += jsonEscape(mqttOutput.config().broker); json += F("\",\"port\":"); json += mqttOutput.config().port;
  json += F(",\"username\":\""); json += jsonEscape(mqttOutput.config().username); json += F("\",\"prefix\":\""); json += jsonEscape(mqttOutput.config().topicPrefix);
  json += F("\",\"interval\":"); json += mqttOutput.config().publishIntervalSeconds; json += F("},\"inverters\":[");
  for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
    if (i) json += ','; const auto& inv = productSettings.inverters[i];
    json += F("{\"enabled\":"); json += inv.enabled ? F("true") : F("false");
    json += F(",\"mac\":\""); json += inv.mac; json += F("\",\"serial\":"); json += inv.serial; json += '}';
  }
  json += F("]}"); return json;
}

String smaStatusJson() {
  // BluetoothSerial::end() deletes its internal SPP EventGroup.  In
  // Arduino-ESP32 3.3.11 connected() does not guard that null handle, so the
  // status endpoint must use our lifecycle state while Bluetooth is off.
  const bool bluetoothConnected = bluetoothReady && smaClient.bluetoothConnected();
  String json;
  json.reserve(768);
  json += F("{\"target\":\""); json += SmaLocalConfig::TARGET_MAC;
  json += F("\",\"expectedSerial\":"); json += SmaLocalConfig::TARGET_SERIAL;
  json += F(",\"lifecycleState\":\""); json += smaLifecycleStateName(smaLifecycleState); json += '"';
  json += F(",\"btActive\":"); json += bluetoothReady ? F("true") : F("false");
  json += F(",\"bluetoothConnected\":"); json += bluetoothConnected ? F("true") : F("false");
  json += F(",\"state\":\""); json += smaClient.stateName(); json += '"';
  json += F(",\"sessionReady\":"); json += smaClient.sessionReady() ? F("true") : F("false");
  json += F(",\"decodedSerial\":"); if (smaClient.decodedSerial()) json += smaClient.decodedSerial(); else json += F("null");
  json += F(",\"netId\":"); json += smaClient.netId();
  json += F(",\"lastAttemptMs\":"); if (smaLastAttemptAt) json += smaLastAttemptAt; else json += F("null");
  json += F(",\"lastSuccessMs\":"); if (smaLastSuccessAt) json += smaLastSuccessAt; else json += F("null");
  json += F(",\"lastError\":\""); json += jsonEscape(smaLifecycleLastError); json += '"';
  json += F(",\"nextAttemptAllowedMs\":"); if (smaNextAttemptAt) json += smaNextAttemptAt; else json += F("null");
  json += F(",\"backoffRemainingMs\":");
  if (smaLifecycleState == SmaLifecycleState::BACKOFF && static_cast<int32_t>(smaNextAttemptAt - millis()) > 0)
    json += smaNextAttemptAt - millis();
  else json += 0;
  json += F(",\"successfulTransactions\":"); json += smaSuccessfulTransactions;
  json += F(",\"failedConnections\":"); json += smaFailedConnections;
  json += F(",\"txBytes\":"); json += smaClient.txBytes();
  json += F(",\"rxBytes\":"); json += smaClient.rxBytes();
  json += F(",\"validResponses\":"); json += smaClient.validResponses();
  json += F(",\"phase3\":{\"loginValid\":"); json += smaClient.loginValid() ? F("true") : F("false");
  json += F(",\"timeSynchronized\":"); json += timeSynchronized() ? F("true") : F("false");
  json += F(",\"unixTime\":"); json += static_cast<unsigned long>(currentUnixTime());
  json += F(",\"appSerial\":"); json += smaClient.appSerial();
  json += F(",\"loginStatus\":"); json += smaClient.loginStatus();
  json += F(",\"loginPacketId\":"); json += smaClient.loginPacketId();
  json += F(",\"loginTimestamp\":"); json += smaClient.loginTimestamp();
  json += F(",\"acPowerValid\":"); json += smaClient.acPowerValid() ? F("true") : F("false");
  json += F(",\"acPowerW\":"); if (smaClient.acPowerValid()) json += smaClient.acPowerW(); else json += F("null");
  json += F(",\"acPowerPacketId\":"); json += smaClient.acPowerPacketId();
  json += F(",\"returnedLri\":"); json += smaClient.returnedLri();
  json += F(",\"recordType\":"); json += smaClient.returnedRecordType();
  json += F(",\"recordSize\":"); json += smaClient.returnedRecordSize();
  json += F(",\"measurementTimestamp\":"); json += smaClient.measurementTimestamp(); json += '}';
  json += F(",\"connectTaskStackHighWater\":"); json += static_cast<unsigned>(smaClient.connectTaskStackHighWater());
  json += F(",\"loopTaskStackHighWater\":"); json += static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr));
  json += F(",\"lastValidResponseAgeMs\":");
  if (smaClient.lastValidResponseAt()) json += millis() - smaClient.lastValidResponseAt(); else json += F("null");
  json += F(",\"heap\":{\"free\":"); json += ESP.getFreeHeap();
  json += F(",\"minimum\":"); json += ESP.getMinFreeHeap();
  json += F(",\"largestInternal\":");
  json += heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  json += F("}}");
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

const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>SMA SunnyBoy Monitor</title><style>body{font:15px system-ui;max-width:1100px;margin:auto;padding:18px;background:#f4f6f8;color:#18212b}section,.card{background:#fff;padding:16px;margin:12px 0;border-radius:9px;box-shadow:0 1px 4px #0001}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}.card{margin:0}.metrics{display:grid;grid-template-columns:1fr 1fr;gap:6px}.value{font-weight:650}label{display:block;margin:7px 0}input,select,button{padding:8px;max-width:100%;box-sizing:border-box}input:not([type=checkbox]),select{width:100%}fieldset{border:1px solid #ddd;margin:10px 0}pre{white-space:pre-wrap;max-height:260px;overflow:auto}.muted{color:#68727d;font-size:13px}</style></head><body><h1>SMA SunnyBoy Monitor</h1><section><h2>Onduleurs</h2><button id="readP" type="button">Lire la puissance P (SMA #1)</button> <span id="readPState" class="muted"></span><div id="cards" class="grid"></div></section><section><h2>Configuration</h2><form method="post" action="/api/product/config"><fieldset><legend>NETWORK</legend><label>Mot de passe AP <input name="apPassword" type="password" maxlength="63" placeholder="vide = conserver"></label><div class="muted">Prend effet au prochain redémarrage.</div></fieldset><fieldset><legend>OTA</legend><label>Mot de passe OTA <input name="otaPassword" type="password" maxlength="63" placeholder="vide = conserver"></label><div class="muted">Prend effet au prochain redémarrage.</div></fieldset><fieldset><legend>LOCATION</legend><label>Latitude <input name="latitude" type="number" step="0.000001" min="-90" max="90"></label><label>Longitude <input name="longitude" type="number" step="0.000001" min="-180" max="180"></label><label>Fuseau POSIX <input name="timezone" maxlength="63" placeholder="UTC0"></label></fieldset><fieldset><legend>MQTT</legend><label><input name="mqttEnabled" type="checkbox" value="1"> Activé</label><label>Broker <input name="broker" value="192.168.50.200" maxlength="63"></label><label>Port <input name="port" type="number" value="1883" min="1" max="65535"></label><label>Utilisateur <input name="username" maxlength="39"></label><label>Mot de passe <input name="mqttPassword" type="password" maxlength="63" placeholder="vide = conserver"></label><label>Préfixe <input name="prefix" value="smaesp" maxlength="47"></label><label>Intervalle (s) <input name="interval" type="number" value="300" min="10" max="86400"></label></fieldset><div id="invConfig"></div><button>Enregistrer la configuration</button></form></section><section><h2>Diagnostic</h2><pre id="status">Chargement…</pre><pre id="log"></pre></section><script>
const unavailable=v=>v===null||v===undefined?'--':v;async function get(u,o){const r=await fetch(u,o),t=await r.text();if(!r.ok)throw Error(t);return t?JSON.parse(t):{}}function card(x){return `<div class="card"><h3>${x.name}</h3><div>Série : ${x.serial||'--'}</div><div>État : ${x.enabled?(x.dataValid?'données valides':'non acquis'):'désactivé'}</div><div>Dernière acquisition : ${x.lastSuccess??'--'}</div><div class="metrics"><span>UAC</span><span class="value">${unavailable(x.uac)} V</span><span>IAC</span><span class="value">${unavailable(x.iac)} A</span><span>PACTot</span><span class="value">${unavailable(x.pac)} W</span><span>EToday / ETotal</span><span>${unavailable(x.eToday)} / ${unavailable(x.eTotal)} kWh</span><span>UDC / IDC / PDC</span><span>${unavailable(x.udc)} / ${unavailable(x.idc)} / ${unavailable(x.pdc)}</span><span>Fréquence</span><span>${unavailable(x.frequency)} Hz</span><span>Température</span><span>${unavailable(x.temperature)} °C</span><span>Statut / relais</span><span>${unavailable(x.status)} / ${unavailable(x.gridRelay)}</span></div></div>`}async function refresh(){try{const [s,d,l]=await Promise.all([get('/api/status'),get('/api/dashboard'),get('/api/log')]);status.textContent=JSON.stringify(s,null,2);cards.innerHTML=d.inverters.map(card).join('');log.textContent=l.lines.join('\n')}catch(e){status.textContent=e}}readP.onclick=async()=>{readP.disabled=true;readPState.textContent='Acquisition en cours — retour réseau automatique…';try{await get('/api/sma/connect',{method:'POST'});setTimeout(()=>{readP.disabled=false;readPState.textContent=''},30000)}catch(e){readP.disabled=false;readPState.textContent=e}};invConfig.innerHTML=[1,2,3].map(i=>`<fieldset><legend>INV${i}</legend><label><input name="inv${i}Enabled" type="checkbox" value="1"> Activé</label><label>Bluetooth MAC <input name="inv${i}Mac" maxlength="17"></label><label>Numéro de série <input name="inv${i}Serial" type="number" min="1" max="4294967295"></label></fieldset>`).join('');async function loadConfig(){const c=await get('/api/config'),f=document.forms[0];f.latitude.value=c.latitude;f.longitude.value=c.longitude;f.timezone.value=c.timezone;f.mqttEnabled.checked=c.mqtt.enabled;f.broker.value=c.mqtt.broker;f.port.value=c.mqtt.port;f.username.value=c.mqtt.username;f.prefix.value=c.mqtt.prefix;f.interval.value=c.mqtt.interval;c.inverters.forEach((x,n)=>{const i=n+1;f[`inv${i}Enabled`].checked=x.enabled;f[`inv${i}Mac`].value=x.mac;f[`inv${i}Serial`].value=x.serial})}setInterval(refresh,3000);loadConfig();refresh();
</script></body></html>)HTML";

const char WIFI_CONFIGURATION_SECTION[] PROGMEM = R"HTML(<section><h2>Wi-Fi</h2><form method="post" action="/api/wifi/config"><label>SSID <input name="ssid" maxlength="32"></label><label>Mot de passe Wi-Fi <input name="password" type="password" maxlength="64"></label><button>Enregistrer et connecter</button></form><p class="muted">La modification Wi-Fi redémarre l'appareil. Le mot de passe n'est jamais relu dans cette page.</p></section>)HTML";

String renderedPage() {
  String page(FPSTR(PAGE));
  page.reserve(page.length() + strlen_P(WIFI_CONFIGURATION_SECTION) + 32);
  page.replace("</form></section><section><h2>Diagnostic",
               String("</form></section>") + FPSTR(WIFI_CONFIGURATION_SECTION) + "<section><h2>Diagnostic");
  return page;
}

void sendJson(const String& value, int status = 200) { server.send(status, "application/json", value); }

void noteHttpRequest(const char* route) {
  (void)route;
  if (scanState == ScanState::SCANNING) ++httpDuringScanCount;
}

void registerRoutes() {
  server.on("/", HTTP_GET, [] { noteHttpRequest("/"); server.send(200, "text/html; charset=utf-8", renderedPage()); });
  server.on("/api/status", HTTP_GET, [] { noteHttpRequest("status"); sendJson(statusJson()); });
  server.on("/api/dashboard", HTTP_GET, [] { noteHttpRequest("dashboard"); sendJson(dashboardJson()); });
  server.on("/api/config", HTTP_GET, [] { noteHttpRequest("config"); sendJson(publicConfigJson()); });
  server.on("/api/bt/results", HTTP_GET, [] { noteHttpRequest("results"); sendJson(resultsJson()); });
  server.on("/api/log", HTTP_GET, [] { noteHttpRequest("log"); sendJson(logJson()); });
  server.on("/api/sma/status", HTTP_GET, [] { noteHttpRequest("sma_status"); sendJson(smaStatusJson()); });
  server.on("/api/mqtt/config", HTTP_POST, [] {
    SbfspotCompat::MqttConfig config = mqttOutput.config();
    config.enabled = server.hasArg("enabled") && server.arg("enabled") == "1";
    strlcpy(config.broker, server.arg("broker").c_str(), sizeof(config.broker));
    strlcpy(config.username, server.arg("username").c_str(), sizeof(config.username));
    strlcpy(config.topicPrefix, server.arg("prefix").c_str(), sizeof(config.topicPrefix));
    const long port = server.arg("port").toInt();
    const long interval = server.arg("interval").toInt();
    if (port < 1 || port > 65535 || interval < 10 || interval > 86400) {
      sendJson("{\"error\":\"invalid_mqtt_configuration\"}", 400); return;
    }
    config.port = static_cast<uint16_t>(port);
    config.publishIntervalSeconds = static_cast<uint32_t>(interval);
    const String password = server.arg("password");
    const bool replacePassword = !password.isEmpty();
    if (replacePassword) strlcpy(config.password, password.c_str(), sizeof(config.password));
    if (!mqttOutput.save(preferences, config, replacePassword)) {
      sendJson("{\"error\":\"invalid_mqtt_configuration\"}", 400); return;
    }
    addLog("[MQTT] configuration saved enabled=%s broker=%s port=%u prefix=%s interval=%lu",
           config.enabled ? "true" : "false", config.broker, config.port, config.topicPrefix,
           static_cast<unsigned long>(config.publishIntervalSeconds));
    sendJson("{\"saved\":true}");
  });
  server.on("/api/product/config", HTTP_POST, [] {
    const double latitude = server.arg("latitude").toDouble();
    const double longitude = server.arg("longitude").toDouble();
    const String timezone = server.arg("timezone");
    const String apCandidate = server.arg("apPassword");
    const String otaCandidate = server.arg("otaPassword");
    if (!ProductConfig::validLatitude(latitude) || !ProductConfig::validLongitude(longitude) ||
        timezone.isEmpty() || timezone.length() >= sizeof(productSettings.timezone) ||
        (!apCandidate.isEmpty() && !ProductConfig::validWpaPassword(apCandidate.c_str())) ||
        (!otaCandidate.isEmpty() && !ProductConfig::validOtaPassword(otaCandidate.c_str()))) {
      sendJson("{\"error\":\"invalid_product_configuration\"}", 400); return;
    }
    ProductConfig::InverterSlot slots[ProductConfig::kInverterCount];
    for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
      const String base = "inv" + String(i + 1);
      slots[i].enabled = server.hasArg(base + "Enabled");
      const String mac = server.arg(base + "Mac");
      const uint32_t serial = strtoul(server.arg(base + "Serial").c_str(), nullptr, 10);
      if (!ProductConfig::validMac(mac.c_str()) || !ProductConfig::validSerial(serial)) {
        sendJson("{\"error\":\"invalid_inverter_configuration\"}", 400); return;
      }
      strlcpy(slots[i].mac, mac.c_str(), sizeof(slots[i].mac)); slots[i].serial = serial;
      snprintf(slots[i].name, sizeof(slots[i].name), "INV%u", static_cast<unsigned>(i + 1));
    }
    SbfspotCompat::MqttConfig mqtt = mqttOutput.config();
    mqtt.enabled = server.hasArg("mqttEnabled");
    strlcpy(mqtt.broker, server.arg("broker").c_str(), sizeof(mqtt.broker));
    strlcpy(mqtt.username, server.arg("username").c_str(), sizeof(mqtt.username));
    strlcpy(mqtt.topicPrefix, server.arg("prefix").c_str(), sizeof(mqtt.topicPrefix));
    const long port = server.arg("port").toInt(), interval = server.arg("interval").toInt();
    if (port < 1 || port > 65535 || interval < 10 || interval > 86400) { sendJson("{\"error\":\"invalid_mqtt_configuration\"}", 400); return; }
    mqtt.port = static_cast<uint16_t>(port); mqtt.publishIntervalSeconds = static_cast<uint32_t>(interval);
    const String mqttPassword = server.arg("mqttPassword");
    if (!mqttPassword.isEmpty()) strlcpy(mqtt.password, mqttPassword.c_str(), sizeof(mqtt.password));
    if (!SbfspotCompat::validateMqttConfig(mqtt)) { sendJson("{\"error\":\"invalid_mqtt_configuration\"}", 400); return; }
    preferences.begin("sma-monitor", false);
    preferences.putDouble("latitude", latitude); preferences.putDouble("longitude", longitude);
    preferences.putString("timezone", timezone);
    if (!apCandidate.isEmpty()) preferences.putString("apPass", apCandidate);
    if (!otaCandidate.isEmpty()) preferences.putString("otaPass", otaCandidate);
    for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
      char key[12]; snprintf(key, sizeof(key), "inv%uEn", static_cast<unsigned>(i + 1)); preferences.putBool(key, slots[i].enabled);
      snprintf(key, sizeof(key), "inv%uMac", static_cast<unsigned>(i + 1)); preferences.putString(key, slots[i].mac);
      snprintf(key, sizeof(key), "inv%uSerial", static_cast<unsigned>(i + 1)); preferences.putUInt(key, slots[i].serial);
    }
    preferences.end();
    productSettings.latitude = latitude; productSettings.longitude = longitude;
    strlcpy(productSettings.timezone, timezone.c_str(), sizeof(productSettings.timezone));
    memcpy(productSettings.inverters, slots, sizeof(slots)); setenv("TZ", productSettings.timezone, 1); tzset();
    if (!mqttOutput.save(preferences, mqtt, !mqttPassword.isEmpty())) { sendJson("{\"error\":\"mqtt_save_failed\"}", 500); return; }
    sendJson("{\"saved\":true,\"restartRequired\":true}");
  });
  server.on("/api/sma/connect", HTTP_POST, [] {
    noteHttpRequest("sma_connect");
    if (scanState == ScanState::SCANNING) { sendJson("{\"error\":\"bt_scan_in_progress\"}", 409); return; }
    if (!timeSynchronized()) { sendJson("{\"error\":\"time_not_synchronized\"}", 503); return; }
    if (!startSafeSmaAcquisition()) {
      if (smaLifecycleState == SmaLifecycleState::BACKOFF) {
        sendJson("{\"error\":\"sma_backoff_active\"}", 429);
      } else {
        sendJson("{\"error\":\"sma_lifecycle_busy\"}", 409);
      }
      return;
    }
    sendJson("{\"accepted\":true,\"state\":\"NETWORK_QUIESCE\"}", 202);
  });
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
  if (!ProductConfig::validOtaPassword(otaPassword.c_str())) {
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
}
}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  bootResetReason = esp_reset_reason();
  if (rtcBreadcrumbMagic == RTC_BREADCRUMB_MAGIC && rtcCheckpointInverse == ~rtcCheckpoint &&
      rtcCheckpoint <= static_cast<uint32_t>(CrashCheckpoint::BT_END_OK)) {
    previousCheckpoint = rtcCheckpoint;
    previousAttempt = rtcAttempt;
  } else {
    rtcBreadcrumbMagic = RTC_BREADCRUMB_MAGIC;
    rtcAttempt = 0;
    rtcCheckpoint = static_cast<uint32_t>(CrashCheckpoint::NONE);
    rtcCheckpointInverse = ~rtcCheckpoint;
  }
  recordCheckpoint(CrashCheckpoint::BOOT);
  bootAt = millis();
  const uint32_t identity = static_cast<uint32_t>(ESP.getEfuseMac());
  char suffix[5]; snprintf(suffix, sizeof(suffix), "%04X", identity & 0xFFFF);
  hostname = "sma-sunnyboy-monitor-" + String(suffix); hostname.toLowerCase();
  apSsid = "SMA-Monitor-" + String(suffix);
  apPassword = "SMAsetup-" + String(suffix);
  loadProductSettings();
  if (productSettings.apPassword[0]) apPassword = productSettings.apPassword;

  addLog("=== SMA SunnyBoy Monitor - Remote Dev Base ===");
  addLog("[SYSTEM] reset=%s code=%u previous_checkpoint=%s previous_attempt=%lu",
         resetReasonName(bootResetReason), static_cast<unsigned>(bootResetReason),
         checkpointName(previousCheckpoint), static_cast<unsigned long>(previousAttempt));
  addLog("[SYSTEM] chip=%s revision=%u cores=%u cpu_mhz=%u flash=%u sdk=%s",
         ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz(),
         ESP.getFlashChipSize(), ESP.getSdkVersion());
  addLog("[HEAP] boot free=%u min=%u largest=%u", ESP.getFreeHeap(), ESP.getMinFreeHeap(),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  connectSta();
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  smaClient.setAppSerial(APP_SERIAL_MIN + (esp_random() % APP_SERIAL_RANGE));
  addLog("[TIME] SNTP configured synchronized=%s unix=%lld app_serial=%lu",
         timeSynchronized() ? "true" : "false", static_cast<long long>(currentUnixTime()),
         static_cast<unsigned long>(smaClient.appSerial()));
  registerRoutes(); server.begin(); addLog("[WEB] ready port=80");
  configureOta();
  mqttOutput.load(preferences);
  mqttOutput.begin();
  refreshInverterSnapshot();
  addLog("[MQTT] ready enabled=%s configured=%s broker=%s port=%u prefix=%s",
         mqttOutput.config().enabled ? "true" : "false", mqttOutput.configured() ? "true" : "false",
         mqttOutput.config().broker, mqttOutput.config().port, mqttOutput.config().topicPrefix);
  bluetoothReady = false;
  smaLifecycleState = SmaLifecycleState::BT_OFF;
  addLog("[BT] off on_demand=true");
  addLog("[HEAP] services_ready free=%u min=%u largest=%u", ESP.getFreeHeap(), ESP.getMinFreeHeap(),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

void loop() {
  const uint32_t now = millis();
  const uint32_t loopGap = now - lastLoopAt;
  if (lastLoopAt != 0 && loopGap > maxLoopGapMs) maxLoopGapMs = loopGap;
  lastLoopAt = now;
  server.handleClient();
  smaClient.tick();
  serviceBtOnlyTest();
  serviceNetworkAudit();
  serviceSmaLifecycle();
  serviceBtScan();
  if (scanState != ScanState::SCANNING) ArduinoOTA.handle();
  refreshInverterSnapshot();
  const bool btOnlySuppressMqtt = btOnlyDisconnectMqtt && btOnlyState != BtOnlyTestState::IDLE &&
                                  btOnlyState != BtOnlyTestState::COMPLETE && btOnlyState != BtOnlyTestState::FAILED;
  const bool networkAuditSuppressMqtt = networkAuditState != NetworkAuditState::IDLE &&
                                        networkAuditState != NetworkAuditState::COMPLETE &&
                                        networkAuditState != NetworkAuditState::FAILED &&
                                        networkAuditState != NetworkAuditState::WAIT_MQTT;
  const bool suppressMqtt = btOnlySuppressMqtt || networkAuditSuppressMqtt;
  mqttOutput.tick(WiFi.status() == WL_CONNECTED && !suppressMqtt, otaBusy, inverterSnapshots);
  const wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != lastWifiStatus) {
    addLog("[WIFI] status_change old=%d new=%d scan=%s", static_cast<int>(lastWifiStatus),
           static_cast<int>(wifiStatus), scanStateName(scanState));
    lastWifiStatus = wifiStatus;
  }
  const bool networkAuditOwnsWifi = networkAuditWifiOff && networkAuditState != NetworkAuditState::IDLE &&
                                    networkAuditState != NetworkAuditState::COMPLETE &&
                                    networkAuditState != NetworkAuditState::FAILED &&
                                    networkAuditState != NetworkAuditState::WAIT_MQTT;
  if (!networkAuditOwnsWifi && WiFi.status() != WL_CONNECTED &&
      static_cast<int32_t>(millis() - nextStaAttemptAt) >= 0) connectSta();
  delay(2);
}
