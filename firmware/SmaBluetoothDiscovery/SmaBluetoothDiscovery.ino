#include <Arduino.h>
#include <ArduinoOTA.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Update.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <time.h>

#include "SmaBluetoothClient.h"
#include "MqttOutputService.h"
#include "ProductConfiguration.h"
#include "Version.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth is not enabled for this target"
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP is required; select an original ESP32 target"
#endif

namespace {
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t INQUIRY_DURATION_MS = 12000;
// Network services and Wi-Fi are quiesced for product discovery, so one
// continuous inquiry provides better coverage than the former short windows.
constexpr uint32_t INQUIRY_WINDOW_MS = INQUIRY_DURATION_MS;
constexpr uint32_t INQUIRY_WINDOW_GRACE_MS = 250;
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

enum class ScanState : uint8_t { IDLE, PREPARING, SCANNING, COMPLETE, ERROR };
enum class SmaLifecycleState : uint8_t {
  BT_OFF, BT_STARTING, BT_READY, CONNECTING, SESSION, TRANSACTION,
  DISCONNECTING, BT_STOPPING, FAILED, BACKOFF
};
enum class NetworkAuditState : uint8_t {
  IDLE, MQTT_SETTLE, OTA_MDNS_SETTLE, WEB_SETTLE, WIFI_SETTLE, WAIT_BT,
  WAIT_SMA, WAIT_MQTT, RESTART, COMPLETE, FAILED
};
enum class SchedulerState : uint8_t { GRACE, RUNNING, ACQUIRING, MAINTENANCE };
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
struct AcquisitionMemory {
  uint32_t beforeInternal = 0;
  uint32_t beforeLargest = 0;
  uint32_t btReadyInternal = 0;
  uint32_t btReadyLargest = 0;
  uint32_t rfcommInternal = 0;
  uint32_t rfcommLargest = 0;
  uint32_t minimumHeap = 0;
  uint32_t btEndInternal = 0;
  uint32_t btEndLargest = 0;
  uint32_t restoredInternal = 0;
  uint32_t restoredLargest = 0;
  bool guardPassed = false;
  bool heapIntegrity = false;
};
struct SchedulerSlotRecord {
  uint32_t scheduledAt = 0;
  uint32_t startedAt = 0;
  uint32_t completedAt = 0;
  uint32_t beforeInternal = 0;
  uint32_t beforeLargest = 0;
  uint32_t btReadyInternal = 0;
  uint32_t btReadyLargest = 0;
  uint32_t rfcommInternal = 0;
  uint32_t rfcommLargest = 0;
  uint32_t btEndInternal = 0;
  uint32_t btEndLargest = 0;
  uint32_t restoredInternal = 0;
  uint32_t restoredLargest = 0;
  uint32_t minimumHeap = 0;
  uint32_t pactotW = 0;
  uint8_t slot = 0;
  uint8_t attempts = 0;
  bool success = false;
  bool heapIntegrity = false;
  char result[24] = "NONE";
};
struct SoakInverterStats {
  uint32_t slots = 0;
  uint32_t firstAttemptSuccess = 0;
  uint32_t secondAttemptSuccess = 0;
  uint32_t completeFailures = 0;
  uint32_t skipped = 0;
  uint32_t consecutiveFailures = 0;
  uint32_t maxConsecutiveFailures = 0;
  time_t lastSuccessTimestamp = 0;
  uint32_t lastPactotW = 0;
  char lastResult[24] = "NONE";
};
struct SoakEvent {
  uint32_t atMs = 0;
  time_t atUnix = 0;
  uint8_t slot = 0;
  char event[24]{};
};
struct SoakStats {
  uint32_t startMs = 0;
  time_t startUnix = 0;
  uint32_t scheduledSlots = 0;
  uint32_t successfulSlots = 0;
  uint32_t failedSlots = 0;
  uint32_t skippedSlots = 0;
  uint32_t secondAttemptsUsed = 0;
  uint32_t btCleanupFailures = 0;
  uint32_t networkRestoreFailures = 0;
  uint32_t heapIntegrityFailures = 0;
  uint32_t mqttPublishFailuresAtStart = 0;
  uint32_t initialInternalFree = 0;
  uint32_t minimumInternalFree = UINT32_MAX;
  uint32_t currentInternalFree = 0;
  uint32_t initialLargestBlock = 0;
  uint32_t minimumLargestBlock = UINT32_MAX;
  uint32_t currentLargestBlock = 0;
  SoakInverterStats inverter[ProductConfig::kInverterCount];
  SoakEvent events[16];
  uint8_t eventNext = 0;
  uint8_t eventCount = 0;
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
bool managedScanActive = false;
bool networkAuditScan = false;
bool inverterTestActive = false;
size_t inverterTestSlot = 0;
uint32_t inverterTestStartedAt = 0;
uint32_t inverterTestCompletedAt = 0;
char inverterTestResult[16] = "IDLE";
bool otaBusy = false;
bool otaServiceReady = false;
bool apActive = false;
volatile bool sppInitSeen = false;
volatile uint32_t sppCloseAt = 0;
volatile uint32_t sppCloseHandle = 0;
volatile bool sppCloseSeen = false;
NetworkAuditState networkAuditState = NetworkAuditState::IDLE;
uint32_t networkAuditStateAt = 0;
bool networkAuditSma = false;
uint32_t networkAuditRestoreStartedAt = 0;
SmaLifecycleState smaLifecycleState = SmaLifecycleState::BT_OFF;
constexpr uint32_t SMA_FAILURE_BACKOFF_MS = 60000;
constexpr uint32_t SCHEDULER_BOOT_GRACE_MS = 60000;
constexpr uint32_t SCHEDULER_SLOT_SPACING_MS = 120000;
constexpr uint32_t SMA_DISCONNECT_GUARD_MS = 3000;
constexpr uint32_t SMA_DIAGNOSTIC_READY_DELAY_MS = 2000;
constexpr size_t RFCOMM_MIN_INTERNAL_FREE = 11860;
constexpr size_t RFCOMM_MIN_INTERNAL_LARGEST = 9204;
uint32_t smaLastAttemptAt = 0;
uint32_t smaLastSuccessAt = 0;
uint32_t smaNextAttemptAt = 0;
uint32_t smaStopAt = 0;
uint32_t smaBtReadyAt = 0;
uint32_t smaSuccessfulTransactions = 0;
uint32_t smaFailedConnections = 0;
size_t smaSelectedSlot = 0;
AcquisitionMemory smaAcquisitionMemory;
SchedulerState schedulerState = SchedulerState::GRACE;
uint32_t schedulerNextSlotAt = 0;
size_t schedulerNextSlot = 0;
bool schedulerAcquisitionActive = false;
size_t schedulerCurrentSlot = 0;
SchedulerSlotRecord schedulerCurrentRecord;
SchedulerSlotRecord schedulerHistory[8];
size_t schedulerHistoryCount = 0;
size_t schedulerHistoryNext = 0;
char schedulerLastResult[24] = "NONE";
uint8_t schedulerAttempt = 0;
bool schedulerRetryPending = false;
uint32_t schedulerRetryAt = 0;
uint32_t schedulerConsecutiveFailures[ProductConfig::kInverterCount]{};
SoakStats soakStats;
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
void serviceNetworkAudit();
void connectSta();

SmaBluetoothClient smaClient(serialBt, smaLogAdapter);
MqttOutputService mqttOutput;
ProductConfig::Settings productSettings;
SbfspotCompat::InverterSnapshot inverterSnapshots[ProductConfig::kInverterCount];

time_t currentUnixTime() { return time(nullptr); }
bool timeSynchronized() { return currentUnixTime() >= MIN_VALID_UNIX_TIME; }

SbfspotCompat::ValueState snapshotState(SmaPhase3::FieldState state) {
  if (state == SmaPhase3::FieldState::Valid) return SbfspotCompat::ValueState::Valid;
  if (state == SmaPhase3::FieldState::Unavailable) return SbfspotCompat::ValueState::Unavailable;
  return SbfspotCompat::ValueState::NotImplemented;
}

const char* valueStateName(SbfspotCompat::ValueState state) {
  if (state == SbfspotCompat::ValueState::Valid) return "VALID";
  if (state == SbfspotCompat::ValueState::Unavailable) return "UNAVAILABLE";
  return "NOT_YET_ACQUIRED";
}

template <typename T>
void copyRawValue(SbfspotCompat::Value<T>& target, const SmaPhase3::RawField& source) {
  target.state = snapshotState(source.state);
  target.value = source.state == SmaPhase3::FieldState::Valid ? static_cast<T>(source.raw) : T{};
}

void formatAttribute(uint64_t raw, char* output, size_t capacity) {
  snprintf(output, capacity, "tag:%lu", static_cast<unsigned long>(raw));
}

void formatKnownAttribute(uint64_t raw, const char* knownValue, uint64_t knownTag,
                          char* output, size_t capacity) {
  if (raw == knownTag) strlcpy(output, knownValue, capacity);
  else formatAttribute(raw, output, capacity);
}

void formatSoftwareVersion(uint32_t raw, char* output, size_t capacity) {
  static constexpr char RELEASE_TYPES[] = "NEABRS";
  const uint8_t release = static_cast<uint8_t>(raw & 0xFFU);
  const uint8_t build = static_cast<uint8_t>((raw >> 8) & 0xFFU);
  const uint8_t minor = static_cast<uint8_t>((raw >> 16) & 0xFFU);
  const uint8_t major = static_cast<uint8_t>((raw >> 24) & 0xFFU);
  snprintf(output, capacity, "%u%u.%u%u.%02u.%c", major >> 4, major & 0x0FU,
           minor >> 4, minor & 0x0FU, build,
           release < sizeof(RELEASE_TYPES) - 1 ? RELEASE_TYPES[release] : '?');
}

void refreshInverterSnapshot() {
  const time_t now = currentUnixTime();
  for (size_t slot = 0; slot < ProductConfig::kInverterCount; ++slot) {
    SbfspotCompat::InverterSnapshot& snapshot = inverterSnapshots[slot];
    snapshot.inverterSerial = productSettings.inverters[slot].serial;
    strlcpy(snapshot.plantName, productSettings.plantName, sizeof(snapshot.plantName));
  }
  if (smaClient.phase3Complete() && smaClient.datasetUsable()) {
    SbfspotCompat::InverterSnapshot& snapshot = inverterSnapshots[smaSelectedSlot];
    const auto& values = smaClient.measurements();
    ProductConfig::formatLocalTime(now, snapshot.timestamp, sizeof(snapshot.timestamp));
    snapshot.inverterTime[0] = 0;
    snapshot.inverterWakeupTime[0] = 0;
    snapshot.inverterSleepTime[0] = 0;
    time_t sunrise = 0, sunset = 0;
    if (ProductConfig::calculateSunTimes(now, productSettings.latitude,
                                         productSettings.longitude, sunrise, sunset)) {
      ProductConfig::formatLocalTime(sunrise, snapshot.sunrise, sizeof(snapshot.sunrise));
      ProductConfig::formatLocalTime(sunset, snapshot.sunset, sizeof(snapshot.sunset));
    }
    copyRawValue(snapshot.acTotalPower, values.acTotalPower);
    copyRawValue(snapshot.acPower1, values.acPower1);
    copyRawValue(snapshot.acVoltage1, values.acVoltage1);
    copyRawValue(snapshot.acCurrent1, values.acCurrent1);
    copyRawValue(snapshot.gridFrequency, values.gridFrequency);
    copyRawValue(snapshot.dcPower1, values.dcPower1);
    copyRawValue(snapshot.dcTotalPower, values.dcPower1);
    copyRawValue(snapshot.dcVoltage1, values.dcVoltage1);
    copyRawValue(snapshot.dcCurrent1, values.dcCurrent1);
    copyRawValue(snapshot.totalEnergy, values.totalEnergy);
    copyRawValue(snapshot.todayEnergy, values.todayEnergy);
    copyRawValue(snapshot.inverterTemperature, values.temperature);
    copyRawValue(snapshot.operatingTime, values.operatingTime);
    copyRawValue(snapshot.feedInTime, values.feedInTime);
    snapshot.inverterNameState = snapshotState(values.inverterNameState);
    if (values.inverterNameState == SmaPhase3::FieldState::Valid)
      strlcpy(snapshot.inverterName, values.inverterName, sizeof(snapshot.inverterName));
    snapshot.inverterClassState = snapshotState(values.inverterClass.state);
    if (values.inverterClass.state == SmaPhase3::FieldState::Valid)
      formatKnownAttribute(values.inverterClass.raw, "Solar Inverters", 8001,
                           snapshot.inverterClass, sizeof(snapshot.inverterClass));
    snapshot.inverterTypeState = snapshotState(values.inverterType.state);
    if (values.inverterType.state == SmaPhase3::FieldState::Valid)
      formatKnownAttribute(values.inverterType.raw, "SB 2500HF-30", 9072,
                           snapshot.inverterType, sizeof(snapshot.inverterType));
    snapshot.inverterSoftwareVersionState = snapshotState(values.softwareVersion.state);
    if (values.softwareVersion.state == SmaPhase3::FieldState::Valid)
      formatSoftwareVersion(static_cast<uint32_t>(values.softwareVersion.raw),
                            snapshot.inverterSoftwareVersion, sizeof(snapshot.inverterSoftwareVersion));
    snapshot.inverterStatusState = snapshotState(values.inverterStatus.state);
    if (values.inverterStatus.state == SmaPhase3::FieldState::Valid)
      formatKnownAttribute(values.inverterStatus.raw, "Ok", 307,
                           snapshot.inverterStatus, sizeof(snapshot.inverterStatus));
    snapshot.inverterGridRelayState = snapshotState(values.gridRelay.state);
    if (values.gridRelay.state == SmaPhase3::FieldState::Valid) {
      if (values.gridRelay.raw == 51)
        strlcpy(snapshot.inverterGridRelay, "Closed", sizeof(snapshot.inverterGridRelay));
      else if (values.gridRelay.raw == 311)
        strlcpy(snapshot.inverterGridRelay, "Open", sizeof(snapshot.inverterGridRelay));
      else
        formatAttribute(values.gridRelay.raw, snapshot.inverterGridRelay,
                        sizeof(snapshot.inverterGridRelay));
    }
    snapshot.acquisitionValid = true;
    snapshot.acquisitionComplete = !smaClient.datasetPartial() && smaClient.acPowerValid();
    snapshot.lastSuccessfulAcquisition = currentUnixTime();
    const uint32_t inverterTime = values.todayEnergy.timestamp ? values.todayEnergy.timestamp
                                                               : values.totalEnergy.timestamp;
    if (inverterTime)
      ProductConfig::formatLocalTime(inverterTime, snapshot.inverterTime, sizeof(snapshot.inverterTime));
    if (values.inverterNameTimestamp)
      ProductConfig::formatLocalTime(values.inverterNameTimestamp, snapshot.inverterWakeupTime,
                                     sizeof(snapshot.inverterWakeupTime));
    // SBFspot semantics: InvSleepTm is the timestamp carried by LRI 0x263F.
    if (values.acTotalPower.timestamp)
      ProductConfig::formatLocalTime(values.acTotalPower.timestamp, snapshot.inverterSleepTime,
                                     sizeof(snapshot.inverterSleepTime));
  }
}

void loadProductSettings() {
  preferences.begin("sma-monitor", false);
  productSettings.maintenanceMode = preferences.getBool("maintenance", false);
  String value = preferences.getString("apPass", "");
  if (ProductConfig::validWpaPassword(value.c_str())) strlcpy(productSettings.apPassword, value.c_str(), sizeof(productSettings.apPassword));
  value = preferences.getString("apSsid", "");
  if (value.length() <= 32) strlcpy(productSettings.apSsid, value.c_str(), sizeof(productSettings.apSsid));
  value = preferences.getString("plantName", "");
  if (value.length() < sizeof(productSettings.plantName))
    strlcpy(productSettings.plantName, value.c_str(), sizeof(productSettings.plantName));
  productSettings.latitude = preferences.getDouble("latitude", 0);
  productSettings.longitude = preferences.getDouble("longitude", 0);
  value = preferences.getString("timezone", "CET-1CEST,M3.5.0,M10.5.0/3");
  if (!value.isEmpty() && value.length() < sizeof(productSettings.timezone))
    strlcpy(productSettings.timezone, value.c_str(), sizeof(productSettings.timezone));
  value = preferences.getString("tzName", "Europe/Brussels");
  if (value.length() < sizeof(productSettings.timezoneName))
    strlcpy(productSettings.timezoneName, value.c_str(), sizeof(productSettings.timezoneName));
  for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
    char key[12];
    snprintf(key, sizeof(key), "inv%uEn", static_cast<unsigned>(i + 1));
    productSettings.inverters[i].enabled = preferences.getBool(key, false);
    snprintf(key, sizeof(key), "inv%uMac", static_cast<unsigned>(i + 1));
    value = preferences.getString(key, "");
    strlcpy(productSettings.inverters[i].mac, value.c_str(), sizeof(productSettings.inverters[i].mac));
    snprintf(key, sizeof(key), "inv%uSerial", static_cast<unsigned>(i + 1));
    productSettings.inverters[i].serial = preferences.getUInt(key, 0);
    snprintf(key, sizeof(key), "inv%uName", static_cast<unsigned>(i + 1));
    value = preferences.getString(key, "");
    if (value.isEmpty()) snprintf(productSettings.inverters[i].name, sizeof(productSettings.inverters[i].name),
                                  "INV%u", static_cast<unsigned>(i + 1));
    else strlcpy(productSettings.inverters[i].name, value.c_str(), sizeof(productSettings.inverters[i].name));
    snprintf(key, sizeof(key), "inv%uPass", static_cast<unsigned>(i + 1));
    value = preferences.getString(key, "");
    if (value.length() <= SmaPhase3::kPasswordLength)
      strlcpy(productSettings.inverters[i].userPassword, value.c_str(),
              sizeof(productSettings.inverters[i].userPassword));
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
    case ScanState::PREPARING: return "PREPARING";
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
      smaAcquisitionMemory.rfcommInternal =
          heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      smaAcquisitionMemory.rfcommLargest =
          heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      addLog("[SPP] open t=%lu status=%d handle=%lu internal=%u internal_largest=%u",
             static_cast<unsigned long>(millis()), parameter->open.status,
             static_cast<unsigned long>(parameter->open.handle),
             heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
      break;
    case ESP_SPP_CLOSE_EVT:
      sppCloseAt = millis();
      sppCloseHandle = parameter->close.handle;
      sppCloseSeen = true;
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
  smaAcquisitionMemory.btEndInternal =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  smaAcquisitionMemory.btEndLargest =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  addLog("[SMA-MEM] bt_off_internal free=%u largest=%u integrity=%s",
         heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
         heap_caps_check_integrity_all(true) ? "ok" : "failed");
}

void logBtStartResources(const char* stage) {
  const bool heapIntegrity = heap_caps_check_integrity_all(true);
  addLog("[BT-MEM] %s free=%u min=%u largest=%u internal=%u internal_largest=%u dma=%u mqtt=%s wifi=%d loop_hwm=%u integrity=%s",
         stage, ESP.getFreeHeap(), ESP.getMinFreeHeap(),
         heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
         heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
         heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
         heap_caps_get_free_size(MALLOC_CAP_DMA), mqttOutput.connected() ? "connected" : "disconnected",
         static_cast<int>(WiFi.status()), static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
         heapIntegrity ? "ok" : "failed");
}

bool startNetworkWindow() {
  if (networkAuditState != NetworkAuditState::IDLE && networkAuditState != NetworkAuditState::COMPLETE &&
      networkAuditState != NetworkAuditState::FAILED) return false;
  if (bluetoothReady || smaLifecycleState != SmaLifecycleState::BT_OFF) return false;
  networkAuditSma = false;
  logBtStartResources("network_normal");
  mqttOutput.disconnect();
  networkAuditStateAt = millis();
  networkAuditState = NetworkAuditState::MQTT_SETTLE;
  addLog("[NETWORK] acquisition window started");
  return true;
}

bool startManagedBtScan() {
  if (managedScanActive || schedulerAcquisitionActive || inverterTestActive || otaBusy ||
      bluetoothReady || smaLifecycleState != SmaLifecycleState::BT_OFF) return false;
  networkAuditScan = true;
  managedScanActive = true;
  scanState = ScanState::PREPARING;
  if (!startNetworkWindow()) {
    networkAuditScan = false;
    managedScanActive = false;
    scanState = ScanState::ERROR;
    return false;
  }
  addLog("[SCAN] managed workflow requested; scheduler suspended");
  return true;
}

bool startSafeSmaAcquisition(size_t slot) {
  if (slot >= ProductConfig::kInverterCount || !productSettings.inverters[slot].enabled) return false;
  if ((networkAuditState != NetworkAuditState::IDLE &&
       networkAuditState != NetworkAuditState::COMPLETE &&
       networkAuditState != NetworkAuditState::FAILED) ||
      bluetoothReady || smaLifecycleState != SmaLifecycleState::BT_OFF) return false;
  const auto& inverter = productSettings.inverters[slot];
  if (!ProductConfig::validMac(inverter.mac) || !ProductConfig::validSerial(inverter.serial) ||
      !smaClient.setUserPassword(inverter.userPassword) ||
      !smaClient.setTarget(inverter.mac, inverter.serial)) return false;
  const uint32_t beforeInternal =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t beforeLargest =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!startNetworkWindow()) return false;
  smaSelectedSlot = slot;
  smaAcquisitionMemory = {};
  smaAcquisitionMemory.beforeInternal = beforeInternal;
  smaAcquisitionMemory.beforeLargest = beforeLargest;
  networkAuditSma = true;
  addLog("[ACQUIRE] safe SMA PACTot cycle requested slot=%u",
         static_cast<unsigned>(slot + 1));
  return true;
}

bool startInverterIdentityTest(size_t slot) {
  if (slot >= ProductConfig::kInverterCount || managedScanActive || inverterTestActive ||
      schedulerAcquisitionActive || bluetoothReady ||
      smaLifecycleState != SmaLifecycleState::BT_OFF) return false;
  const auto& inverter = productSettings.inverters[slot];
  if (!ProductConfig::validMac(inverter.mac) ||
      !smaClient.setUserPassword(inverter.userPassword) ||
      !smaClient.setTarget(inverter.mac, inverter.serial)) return false;
  if (!startNetworkWindow()) return false;
  smaSelectedSlot = slot;
  smaAcquisitionMemory = {};
  networkAuditSma = true;
  return true;
}

const char* schedulerStateName(SchedulerState value) {
  switch (value) {
    case SchedulerState::GRACE: return "GRACE";
    case SchedulerState::RUNNING: return "RUNNING";
    case SchedulerState::ACQUIRING: return "ACQUIRING";
    case SchedulerState::MAINTENANCE: return "MAINTENANCE";
  }
  return "RUNNING";
}

void recordSoakEvent(size_t slot, const char* event) {
  SoakEvent& item = soakStats.events[soakStats.eventNext];
  item.atMs = millis();
  item.atUnix = currentUnixTime();
  item.slot = static_cast<uint8_t>(slot);
  strlcpy(item.event, event, sizeof(item.event));
  soakStats.eventNext = (soakStats.eventNext + 1) % 16;
  if (soakStats.eventCount < 16) ++soakStats.eventCount;
}

void updateSoakMemory() {
  const uint32_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  soakStats.currentInternalFree = internalFree;
  soakStats.currentLargestBlock = largest;
  if (internalFree < soakStats.minimumInternalFree) soakStats.minimumInternalFree = internalFree;
  if (largest < soakStats.minimumLargestBlock) soakStats.minimumLargestBlock = largest;
}

void storeSchedulerRecord(const SchedulerSlotRecord& record) {
  schedulerHistory[schedulerHistoryNext] = record;
  schedulerHistoryNext = (schedulerHistoryNext + 1) % 8;
  if (schedulerHistoryCount < 8) ++schedulerHistoryCount;
  strlcpy(schedulerLastResult, record.result, sizeof(schedulerLastResult));
}

void skipSchedulerSlot(uint32_t scheduledAt, size_t slot, const char* reason) {
  SchedulerSlotRecord record{};
  record.scheduledAt = scheduledAt;
  record.completedAt = millis();
  record.slot = static_cast<uint8_t>(slot);
  strlcpy(record.result, reason, sizeof(record.result));
  storeSchedulerRecord(record);
  ++soakStats.scheduledSlots;
  ++soakStats.skippedSlots;
  ++soakStats.inverter[slot].slots;
  ++soakStats.inverter[slot].skipped;
  strlcpy(soakStats.inverter[slot].lastResult, reason,
          sizeof(soakStats.inverter[slot].lastResult));
  recordSoakEvent(slot, reason);
  addLog("[SCHED] slot=%u scheduled=%lu result=%s", static_cast<unsigned>(slot + 1),
         static_cast<unsigned long>(scheduledAt), reason);
}

void serviceScheduler() {
  const uint32_t now = millis();
  if (managedScanActive || inverterTestActive) return;
  if (schedulerAcquisitionActive) {
    schedulerState = SchedulerState::ACQUIRING;
    return;
  }
  if (productSettings.maintenanceMode) schedulerState = SchedulerState::MAINTENANCE;
  else if (static_cast<int32_t>(now - (bootAt + SCHEDULER_BOOT_GRACE_MS)) < 0)
    schedulerState = SchedulerState::GRACE;
  else schedulerState = SchedulerState::RUNNING;

  if (static_cast<int32_t>(now - schedulerNextSlotAt) < 0) return;
  const uint32_t scheduledAt = schedulerNextSlotAt;
  const size_t slot = schedulerNextSlot;
  schedulerNextSlotAt += SCHEDULER_SLOT_SPACING_MS;
  schedulerNextSlot = (schedulerNextSlot + 1) % ProductConfig::kInverterCount;

  if (productSettings.maintenanceMode) { skipSchedulerSlot(scheduledAt, slot, "SKIP_MAINTENANCE"); return; }
  if (otaBusy) { skipSchedulerSlot(scheduledAt, slot, "SKIP_OTA"); return; }
  const bool networkBusy = networkAuditState != NetworkAuditState::IDLE &&
                           networkAuditState != NetworkAuditState::COMPLETE &&
                           networkAuditState != NetworkAuditState::FAILED;
  if (networkBusy || bluetoothReady || smaLifecycleState != SmaLifecycleState::BT_OFF) {
    skipSchedulerSlot(scheduledAt, slot, "SKIP_BUSY"); return;
  }
  if (!productSettings.inverters[slot].enabled) {
    skipSchedulerSlot(scheduledAt, slot, "SKIP_DISABLED"); return;
  }
  if (WiFi.status() != WL_CONNECTED ||
      (mqttOutput.config().enabled && !mqttOutput.connected())) {
    skipSchedulerSlot(scheduledAt, slot, "SKIP_NETWORK"); return;
  }
  const uint32_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const bool integrity = heap_caps_check_integrity_all(true);
  if (!integrity ||
      internalFree < RFCOMM_MIN_INTERNAL_FREE || largest < RFCOMM_MIN_INTERNAL_LARGEST) {
    if (!integrity) ++soakStats.heapIntegrityFailures;
    skipSchedulerSlot(scheduledAt, slot, "SKIP_MEMORY"); return;
  }

  schedulerCurrentRecord = {};
  schedulerCurrentRecord.scheduledAt = scheduledAt;
  schedulerCurrentRecord.startedAt = now;
  schedulerCurrentRecord.slot = static_cast<uint8_t>(slot);
  schedulerCurrentRecord.attempts = 1;
  schedulerCurrentRecord.beforeInternal = internalFree;
  schedulerCurrentRecord.beforeLargest = largest;
  ++soakStats.scheduledSlots;
  ++soakStats.inverter[slot].slots;
  if (!startSafeSmaAcquisition(slot)) {
    strlcpy(schedulerCurrentRecord.result, "SKIP_BUSY", sizeof(schedulerCurrentRecord.result));
    schedulerCurrentRecord.completedAt = millis();
    storeSchedulerRecord(schedulerCurrentRecord);
    ++soakStats.skippedSlots;
    ++soakStats.inverter[slot].skipped;
    strlcpy(soakStats.inverter[slot].lastResult, "SKIP_BUSY",
            sizeof(soakStats.inverter[slot].lastResult));
    recordSoakEvent(slot, "SKIP_BUSY");
    return;
  }
  schedulerCurrentSlot = slot;
  schedulerAttempt = 1;
  schedulerRetryPending = false;
  schedulerAcquisitionActive = true;
  schedulerState = SchedulerState::ACQUIRING;
  addLog("[SCHED] slot=%u scheduled=%lu started=%lu", static_cast<unsigned>(slot + 1),
         static_cast<unsigned long>(scheduledAt), static_cast<unsigned long>(now));
}

void restartNetworkAuditServices() {
  server.begin();
  if (!otaPassword.isEmpty()) {
    ArduinoOTA.begin();
    otaServiceReady = true;
  }
  addLog("[NETWORK] services_restarted web=true ota=%s mdns=%s",
         otaServiceReady ? "true" : "false", otaServiceReady ? "true" : "false");
  networkAuditState = NetworkAuditState::WAIT_MQTT;
}

void serviceNetworkAudit() {
  const uint32_t now = millis();
  switch (networkAuditState) {
    case NetworkAuditState::MQTT_SETTLE:
      if (now - networkAuditStateAt >= 2000) {
        logBtStartResources("network_mqtt_off");
        if (otaServiceReady) {
          ArduinoOTA.end();  // ArduinoOTA 3.3.11 also calls MDNS.end().
          otaServiceReady = false;
        }
        networkAuditStateAt = now;
        networkAuditState = NetworkAuditState::OTA_MDNS_SETTLE;
      }
      break;
    case NetworkAuditState::OTA_MDNS_SETTLE:
      if (now - networkAuditStateAt >= 2000) {
        logBtStartResources("network_ota_mdns_off");
        server.stop();
        networkAuditStateAt = now;
        networkAuditState = NetworkAuditState::WEB_SETTLE;
      }
      break;
    case NetworkAuditState::WEB_SETTLE:
      if (now - networkAuditStateAt >= 2000) {
        logBtStartResources("network_web_off");
        const bool stopped = WiFi.disconnect(true, false);
        addLog("[NETWORK] wifi_stop result=%s", stopped ? "true" : "false");
        networkAuditStateAt = now;
        networkAuditState = NetworkAuditState::WIFI_SETTLE;
      }
      break;
    case NetworkAuditState::WIFI_SETTLE:
      if (now - networkAuditStateAt >= 3000) {
        logBtStartResources("network_wifi_off");
        if (networkAuditScan) {
          sppInitSeen = false;
          bluetoothReady = serialBt.begin("SMA-SunnyBoy-Monitor", true, true);
          if (bluetoothReady) {
            serialBt.setPin("0000", 4);
            serialBt.onAuthComplete(onBtAuthComplete);
            serialBt.register_callback(onSppEvent);
            networkAuditStateAt = now;
            networkAuditState = NetworkAuditState::WAIT_BT;
            addLog("[SCAN] Bluetooth begin accepted");
          } else {
            scanState = ScanState::ERROR;
            networkAuditState = NetworkAuditState::WAIT_BT;
            addLog("[SCAN] Bluetooth begin failed");
          }
        } else if (networkAuditSma) {
          if (startSmaLifecycle()) {
            networkAuditState = NetworkAuditState::WAIT_SMA;
          } else {
            networkAuditState = NetworkAuditState::FAILED;
            networkAuditRestoreStartedAt = millis();
            connectSta();
            restartNetworkAuditServices();
          }
        } else {
          networkAuditState = NetworkAuditState::FAILED;
          networkAuditRestoreStartedAt = millis();
          connectSta();
          addLog("[NETWORK] wifi_ip_restore_ms=%lu connected=%s",
                 static_cast<unsigned long>(millis() - networkAuditRestoreStartedAt),
                 WiFi.status() == WL_CONNECTED ? "true" : "false");
          restartNetworkAuditServices();
        }
      }
      break;
    case NetworkAuditState::WAIT_SMA:
      if (smaLifecycleState == SmaLifecycleState::BACKOFF && schedulerRetryPending) {
        if (static_cast<int32_t>(now - schedulerRetryAt) < 0) break;
        const uint32_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        const uint32_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        const bool integrity = heap_caps_check_integrity_all(true);
        if (bluetoothReady || !integrity ||
            internalFree < RFCOMM_MIN_INTERNAL_FREE || largest < RFCOMM_MIN_INTERNAL_LARGEST) {
          addLog("[SCHED] retry_no_go bt=%s internal=%u largest=%u integrity=%s",
                 bluetoothReady ? "on" : "off", internalFree, largest,
                 integrity ? "ok" : "failed");
          schedulerRetryPending = false;
          break;
        }
        schedulerRetryPending = false;
        schedulerAttempt = 2;
        schedulerCurrentRecord.attempts = 2;
        addLog("[SCHED] slot=%u attempt=2 fresh_bt_lifecycle",
               static_cast<unsigned>(schedulerCurrentSlot + 1));
        if (!startSmaLifecycle()) {
          strlcpy(smaLifecycleLastError, "retry_start_failed", sizeof(smaLifecycleLastError));
          break;
        }
        break;
      }
      if (smaLifecycleState == SmaLifecycleState::BT_OFF ||
          smaLifecycleState == SmaLifecycleState::BACKOFF) {
        networkAuditRestoreStartedAt = millis();
        connectSta();
        addLog("[NETWORK] wifi_ip_restore_ms=%lu connected=%s",
               static_cast<unsigned long>(millis() - networkAuditRestoreStartedAt),
               WiFi.status() == WL_CONNECTED ? "true" : "false");
        restartNetworkAuditServices();
      }
      break;
    case NetworkAuditState::WAIT_BT:
      if (networkAuditScan && (scanState == ScanState::COMPLETE || scanState == ScanState::ERROR)) {
        stopBluetoothService();
        networkAuditRestoreStartedAt = millis();
        connectSta();
        restartNetworkAuditServices();
      }
      break;
    case NetworkAuditState::WAIT_MQTT:
      if (!mqttOutput.config().enabled || mqttOutput.connected()) {
        addLog("[NETWORK] mqtt_restore_ms=%lu",
               static_cast<unsigned long>(millis() - networkAuditRestoreStartedAt));
        logBtStartResources("network_restored");
        smaAcquisitionMemory.restoredInternal =
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        smaAcquisitionMemory.restoredLargest =
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (inverterTestActive) {
          const bool pass = !smaLifecycleLastError[0] && smaClient.datasetUsable() &&
                            smaClient.decodedSerial() == smaClient.expectedSerial();
          if (pass && productSettings.inverters[inverterTestSlot].serial == 0) {
            productSettings.inverters[inverterTestSlot].serial = smaClient.decodedSerial();
            char key[12]; snprintf(key, sizeof(key), "inv%uSerial",
                                   static_cast<unsigned>(inverterTestSlot + 1));
            preferences.begin("sma-monitor", false);
            preferences.putUInt(key, smaClient.decodedSerial()); preferences.end();
          }
          strlcpy(inverterTestResult, pass ? "PASS" : "FAIL", sizeof(inverterTestResult));
          inverterTestCompletedAt = millis();
          inverterTestActive = false;
          addLog("[INV-TEST] slot=%u result=%s", static_cast<unsigned>(inverterTestSlot + 1),
                 inverterTestResult);
        }
        if (networkAuditScan) {
          networkAuditScan = false;
          managedScanActive = false;
          addLog("[SCAN] network restored; scheduler resumed");
        }
        if (schedulerAcquisitionActive) {
          schedulerCurrentRecord.completedAt = millis();
          schedulerCurrentRecord.btReadyInternal = smaAcquisitionMemory.btReadyInternal;
          schedulerCurrentRecord.btReadyLargest = smaAcquisitionMemory.btReadyLargest;
          schedulerCurrentRecord.rfcommInternal = smaAcquisitionMemory.rfcommInternal;
          schedulerCurrentRecord.rfcommLargest = smaAcquisitionMemory.rfcommLargest;
          schedulerCurrentRecord.btEndInternal = smaAcquisitionMemory.btEndInternal;
          schedulerCurrentRecord.btEndLargest = smaAcquisitionMemory.btEndLargest;
          schedulerCurrentRecord.restoredInternal = smaAcquisitionMemory.restoredInternal;
          schedulerCurrentRecord.restoredLargest = smaAcquisitionMemory.restoredLargest;
          schedulerCurrentRecord.minimumHeap = smaAcquisitionMemory.minimumHeap;
          schedulerCurrentRecord.heapIntegrity = smaAcquisitionMemory.heapIntegrity;
          schedulerCurrentRecord.success = !smaLifecycleLastError[0] && smaClient.datasetUsable();
          if (schedulerCurrentRecord.success) {
            schedulerCurrentRecord.pactotW = smaClient.acPowerW();
            const bool partial = smaClient.datasetPartial() || !smaClient.acPowerValid();
            strlcpy(schedulerCurrentRecord.result, partial ? "PARTIAL_SUCCESS" : "SUCCESS",
                    sizeof(schedulerCurrentRecord.result));
            schedulerConsecutiveFailures[schedulerCurrentSlot] = 0;
            ++soakStats.successfulSlots;
            auto& invStats = soakStats.inverter[schedulerCurrentSlot];
            if (schedulerCurrentRecord.attempts == 1) ++invStats.firstAttemptSuccess;
            else ++invStats.secondAttemptSuccess;
            invStats.consecutiveFailures = 0;
            invStats.lastSuccessTimestamp = smaClient.measurementTimestamp();
            invStats.lastPactotW = smaClient.acPowerW();
            strlcpy(invStats.lastResult, partial ? "PARTIAL_SUCCESS" : "SUCCESS",
                    sizeof(invStats.lastResult));
            mqttOutput.requestPublish(schedulerCurrentSlot);
          } else {
            strlcpy(schedulerCurrentRecord.result, "ACQUISITION_FAILED",
                    sizeof(schedulerCurrentRecord.result));
            ++schedulerConsecutiveFailures[schedulerCurrentSlot];
            ++soakStats.failedSlots;
            auto& invStats = soakStats.inverter[schedulerCurrentSlot];
            ++invStats.completeFailures;
            ++invStats.consecutiveFailures;
            if (invStats.consecutiveFailures > invStats.maxConsecutiveFailures)
              invStats.maxConsecutiveFailures = invStats.consecutiveFailures;
            strlcpy(invStats.lastResult, "ACQUISITION_FAILED", sizeof(invStats.lastResult));
            recordSoakEvent(schedulerCurrentSlot, "ACQUISITION_FAILED");
          }
          if (!schedulerCurrentRecord.heapIntegrity) ++soakStats.heapIntegrityFailures;
          storeSchedulerRecord(schedulerCurrentRecord);
          addLog("[SCHED] slot=%u complete result=%s", static_cast<unsigned>(schedulerCurrentSlot + 1),
                 schedulerCurrentRecord.result);
          schedulerAcquisitionActive = false;
          schedulerState = productSettings.maintenanceMode ? SchedulerState::MAINTENANCE
                                                           : SchedulerState::RUNNING;
        }
        networkAuditState = NetworkAuditState::COMPLETE;
      }
      break;
    default: break;
  }
}

void failSmaLifecycle(const char* error) {
  smaAcquisitionMemory.minimumHeap = ESP.getMinFreeHeap();
  smaAcquisitionMemory.heapIntegrity = heap_caps_check_integrity_all(true);
  recordCheckpoint(CrashCheckpoint::CLEANUP_START);
  strlcpy(smaLifecycleLastError, error && error[0] ? error : "unknown", sizeof(smaLifecycleLastError));
  ++smaFailedConnections;
  setSmaLifecycleState(SmaLifecycleState::FAILED);
  if (smaClient.state() != SmaBluetoothClient::State::DISCONNECTED) smaClient.requestDisconnect();
  setSmaLifecycleState(SmaLifecycleState::BT_STOPPING);
  stopBluetoothService();
  if (schedulerAcquisitionActive && schedulerAttempt < 2) {
    ++soakStats.secondAttemptsUsed;
    recordSoakEvent(schedulerCurrentSlot, "SECOND_ATTEMPT_USED");
    schedulerRetryPending = true;
    schedulerRetryAt = millis() + SMA_DISCONNECT_GUARD_MS;
    smaNextAttemptAt = schedulerRetryAt;
    addLog("[SCHED] slot=%u attempt=1 failed; fresh_lifecycle_retry_at=%lu",
           static_cast<unsigned>(schedulerCurrentSlot + 1),
           static_cast<unsigned long>(schedulerRetryAt));
  } else {
    schedulerRetryPending = false;
    smaNextAttemptAt = millis() + SMA_FAILURE_BACKOFF_MS;
  }
  setSmaLifecycleState(SmaLifecycleState::BACKOFF);
  addLog("[SMA-LIFECYCLE] failure=%s backoff_ms=%lu", smaLifecycleLastError,
         static_cast<unsigned long>(schedulerRetryPending ? SMA_DISCONNECT_GUARD_MS
                                                         : SMA_FAILURE_BACKOFF_MS));
}

bool startSmaLifecycle() {
  const uint32_t now = millis();
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
        logBtStartResources("sma_bt_ready");
        smaAcquisitionMemory.btReadyInternal =
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        smaAcquisitionMemory.btReadyLargest =
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        setSmaLifecycleState(SmaLifecycleState::BT_READY);
      } else if (now - smaLastAttemptAt >= 10000) {
        failSmaLifecycle("bt_ready_timeout");
      }
      break;
    case SmaLifecycleState::BT_READY:
      if (now - smaBtReadyAt < SMA_DIAGNOSTIC_READY_DELAY_MS) break;
      {
        const size_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        const size_t internalLargest =
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        const bool integrity = heap_caps_check_integrity_all(true);
        const bool memoryReady =
            internalFree >= RFCOMM_MIN_INTERNAL_FREE &&
            internalLargest >= RFCOMM_MIN_INTERNAL_LARGEST;
        addLog("[SMA-MEM] rfcomm_guard slot=%u internal=%u largest=%u integrity=%s result=%s",
               static_cast<unsigned>(smaSelectedSlot + 1), static_cast<unsigned>(internalFree),
               static_cast<unsigned>(internalLargest), integrity ? "ok" : "failed",
               memoryReady && integrity ? "GO" : "NO_GO");
        smaAcquisitionMemory.guardPassed = memoryReady && integrity;
        if (!memoryReady || !integrity) {
          failSmaLifecycle(integrity ? "rfcomm_memory_no_go" : "heap_integrity_failed");
          break;
        }
      }
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
        // Commit only after the complete bounded query sequence. A failed
        // attempt therefore preserves the previous valid snapshot.
        refreshInverterSnapshot();
        smaAcquisitionMemory.minimumHeap = ESP.getMinFreeHeap();
        smaAcquisitionMemory.heapIntegrity = heap_caps_check_integrity_all(true);
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
  for (size_t index = 0; index < ProductConfig::kInverterCount; ++index) {
    if (productSettings.inverters[index].enabled &&
        strcasecmp(mac, productSettings.inverters[index].mac) == 0) return static_cast<int8_t>(index);
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
    const auto& sma = productSettings.inverters[incoming.knownIndex];
    addLog("[BT] MATCH %s SN=%lu", sma.name, static_cast<unsigned long>(sma.serial));
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
  if (scanState == ScanState::PREPARING) {
    // begin()/isReady() can become true just before ESP_SPP_INIT_EVT. Inquiry
    // is safe only after the public SPP init callback has actually arrived.
    if (bluetoothReady && sppInitSeen) {
      if (!startBtScan()) {
        scanState = ScanState::ERROR;
        addLog("[SCAN] ERROR managed_scan_start_failed");
      }
    } else if (networkAuditState == NetworkAuditState::WAIT_BT &&
               millis() - networkAuditStateAt >= 10000) {
      scanState = ScanState::ERROR;
      addLog("[SCAN] ERROR spp_ready_timeout");
    }
    return;
  }
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
  json += F("\",\"active\":"); json += managedScanActive ? F("true") : F("false");
  json += F(",\"cycle\":"); json += scanNumber;
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
      const auto& sma = productSettings.inverters[item.knownIndex];
      json += F("{\"label\":\""); json += sma.name; json += F("\",\"serial\":"); json += sma.serial; json += '}';
    } else json += F("null");
    json += '}';
  }
  json += F("]}");
  return json;
}

String statusJson() {
  String json;
  json.reserve(2000);
  json += F("{\"firmwareName\":\""); json += SmaVersion::kFirmwareName;
  json += F("\",\"firmwareVersion\":\""); json += SmaVersion::kFirmwareVersion;
  json += F("\",\"schemaVersion\":\""); json += SmaVersion::kSchemaVersion;
  json += F("\",\"uptimeMs\":"); json += millis() - bootAt;
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
  json += F(",\"staSsid\":\""); json += jsonEscape(WiFi.SSID().c_str()); json += '"';
  json += F(",\"apIp\":"); if (apActive) { json += '"'; json += WiFi.softAPIP().toString(); json += '"'; } else json += F("null");
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
  const time_t now = currentUnixTime();
  char localTime[24]{}, offset[8]{};
  tm local{};
  if (timeSynchronized() && localtime_r(&now, &local)) strftime(offset, sizeof(offset), "%z", &local);
  ProductConfig::formatLocalTime(now, localTime, sizeof(localTime));
  time_t rise = 0, set = 0; char riseText[24]{}, setText[24]{};
  if (ProductConfig::calculateSunTimes(now, productSettings.latitude, productSettings.longitude, rise, set)) {
    ProductConfig::formatLocalTime(rise, riseText, sizeof(riseText));
    ProductConfig::formatLocalTime(set, setText, sizeof(setText));
  }
  json += F("},\"time\":{\"synchronized\":"); json += timeSynchronized() ? F("true") : F("false");
  json += F(",\"local\":\""); json += localTime; json += F("\",\"utcOffset\":\""); json += offset;
  json += F("\",\"dst\":"); json += local.tm_isdst > 0 ? F("true") : F("false");
  json += F(",\"timezone\":\""); json += productSettings.timezoneName;
  json += F("\",\"sunrise\":"); if (riseText[0]) { json += '"'; json += riseText; json += '"'; } else json += F("null");
  json += F(",\"sunset\":"); if (setText[0]) { json += '"'; json += setText; json += '"'; } else json += F("null");
  json += F("},\"ota\":{\"busy\":"); json += otaBusy ? F("true") : F("false");
  json += F(",\"ready\":"); json += otaServiceReady ? F("true") : F("false");
  json += F(",\"browserUpdate\":"); json += otaPassword.isEmpty() ? F("false") : F("true");
  json += F("}}");
  return json;
}

String dashboardJson() {
  String json(F("{\"inverters\":["));
  json.reserve(6000);
  const time_t now = currentUnixTime();
  for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
    if (i) json += ',';
    const auto& cfg = productSettings.inverters[i];
    const auto& s = inverterSnapshots[i];
    json += F("{\"slot\":"); json += static_cast<unsigned>(i + 1);
    json += F(",\"name\":\""); json += cfg.name; json += '"';
    json += F(",\"enabled\":"); json += cfg.enabled ? F("true") : F("false");
    json += F(",\"serial\":"); json += cfg.serial;
    json += F(",\"dataValid\":"); json += s.acquisitionValid ? F("true") : F("false");
    json += F(",\"dataComplete\":"); json += s.acquisitionComplete ? F("true") : F("false");
    json += F(",\"acquisitionResult\":\"");
    json += s.acquisitionComplete ? F("success") : (s.acquisitionValid ? F("partial") : F("none"));
    json += '"';
    json += F(",\"consecutiveFailures\":"); json += schedulerConsecutiveFailures[i];
    json += F(",\"lastSuccess\":");
    if (s.lastSuccessfulAcquisition) { char formatted[24]{}; ProductConfig::formatLocalTime(s.lastSuccessfulAcquisition, formatted, sizeof(formatted)); json += '"'; json += formatted; json += '"'; }
    else json += F("null");
    json += F(",\"ageSeconds\":");
    if (s.lastSuccessfulAcquisition && now >= s.lastSuccessfulAcquisition)
      json += static_cast<unsigned long>(now - s.lastSuccessfulAcquisition);
    else json += F("null");
#define DASH_VALUE(key, member, divisor) do { \
    json += F(",\"" key "\":"); \
    if (s.member.state == SbfspotCompat::ValueState::Valid) json += static_cast<double>(s.member.value) / divisor; \
    else json += F("null"); \
    json += F(",\"" key "State\":\""); json += valueStateName(s.member.state); json += '"'; \
  } while (0)
    DASH_VALUE("uac", acVoltage1, 100.0);
    DASH_VALUE("iac", acCurrent1, 1000.0);
    DASH_VALUE("pac", acTotalPower, 1.0);
    DASH_VALUE("pac1", acPower1, 1.0);
    DASH_VALUE("eToday", todayEnergy, 1000.0);
    DASH_VALUE("eTotal", totalEnergy, 1000.0);
    DASH_VALUE("udc", dcVoltage1, 100.0);
    DASH_VALUE("idc", dcCurrent1, 1000.0);
    DASH_VALUE("pdc", dcPower1, 1.0);
    DASH_VALUE("pdcTotal", dcTotalPower, 1.0);
    DASH_VALUE("frequency", gridFrequency, 100.0);
    DASH_VALUE("temperature", inverterTemperature, 100.0);
    DASH_VALUE("operationHours", operatingTime, 3600.0);
    DASH_VALUE("feedInHours", feedInTime, 3600.0);
#undef DASH_VALUE
    json += F(",\"status\":"); if (s.inverterStatusState == SbfspotCompat::ValueState::Valid) { json += '"'; json += jsonEscape(s.inverterStatus); json += '"'; } else json += F("null");
    json += F(",\"gridRelay\":"); if (s.inverterGridRelayState == SbfspotCompat::ValueState::Valid) { json += '"'; json += jsonEscape(s.inverterGridRelay); json += '"'; } else json += F("null");
    json += F(",\"inverterName\":"); if (s.inverterNameState == SbfspotCompat::ValueState::Valid) { json += '"'; json += jsonEscape(s.inverterName); json += '"'; } else json += F("null");
    json += F(",\"inverterClass\":"); if (s.inverterClassState == SbfspotCompat::ValueState::Valid) { json += '"'; json += jsonEscape(s.inverterClass); json += '"'; } else json += F("null");
    json += F(",\"inverterType\":"); if (s.inverterTypeState == SbfspotCompat::ValueState::Valid) { json += '"'; json += jsonEscape(s.inverterType); json += '"'; } else json += F("null");
    json += F(",\"softwareVersion\":"); if (s.inverterSoftwareVersionState == SbfspotCompat::ValueState::Valid) { json += '"'; json += jsonEscape(s.inverterSoftwareVersion); json += '"'; } else json += F("null");
    json += F(",\"inverterTime\":"); if (s.inverterTime[0]) { json += '"'; json += s.inverterTime; json += '"'; } else json += F("null");
    json += F(",\"wakeupTime\":"); if (s.inverterWakeupTime[0]) { json += '"'; json += s.inverterWakeupTime; json += '"'; } else json += F("null");
    json += F(",\"sleepTime\":"); if (s.inverterSleepTime[0]) { json += '"'; json += s.inverterSleepTime; json += '"'; } else json += F("null");
    json += '}';
  }
  json += F("],\"scheduler\":{\"state\":\""); json += schedulerStateName(schedulerState);
  json += F("\",\"currentSlot\":");
  if (schedulerAcquisitionActive) json += static_cast<unsigned>(schedulerCurrentSlot + 1); else json += F("null");
  json += F(",\"nextSlot\":"); json += static_cast<unsigned>(schedulerNextSlot + 1);
  json += F(",\"nextScheduledAtMs\":"); json += schedulerNextSlotAt;
  json += F(",\"lastResult\":\""); json += schedulerLastResult;
  json += F("\",\"maintenanceMode\":"); json += productSettings.maintenanceMode ? F("true") : F("false");
  json += F(",\"history\":[");
  const size_t oldest = (schedulerHistoryNext + 8 - schedulerHistoryCount) % 8;
  for (size_t index = 0; index < schedulerHistoryCount; ++index) {
    if (index) json += ',';
    const auto& record = schedulerHistory[(oldest + index) % 8];
    json += F("{\"slot\":"); json += static_cast<unsigned>(record.slot + 1);
    json += F(",\"attempts\":"); json += static_cast<unsigned>(record.attempts);
    json += F(",\"scheduledAt\":"); json += record.scheduledAt;
    json += F(",\"startedAt\":"); json += record.startedAt;
    json += F(",\"completedAt\":"); json += record.completedAt;
    json += F(",\"result\":\""); json += record.result;
    json += F("\",\"pactotW\":"); if (record.success) json += record.pactotW; else json += F("null");
    json += F(",\"beforeInternal\":"); json += record.beforeInternal;
    json += F(",\"beforeLargest\":"); json += record.beforeLargest;
    json += F(",\"btReadyInternal\":"); json += record.btReadyInternal;
    json += F(",\"btReadyLargest\":"); json += record.btReadyLargest;
    json += F(",\"rfcommInternal\":"); json += record.rfcommInternal;
    json += F(",\"rfcommLargest\":"); json += record.rfcommLargest;
    json += F(",\"btEndInternal\":"); json += record.btEndInternal;
    json += F(",\"btEndLargest\":"); json += record.btEndLargest;
    json += F(",\"restoredInternal\":"); json += record.restoredInternal;
    json += F(",\"restoredLargest\":"); json += record.restoredLargest;
    json += F(",\"minimumHeap\":"); json += record.minimumHeap;
    json += F(",\"heapIntegrity\":"); json += record.heapIntegrity ? F("true") : F("false");
    json += '}';
  }
  json += F("]},\"location\":{\"latitude\":"); json += String(productSettings.latitude, 6);
  json += F(",\"longitude\":"); json += String(productSettings.longitude, 6);
  json += F(",\"timezone\":\""); json += jsonEscape(productSettings.timezone); json += F("\"}}");
  return json;
}

String publicConfigJson() {
  String json(F("{\"latitude\":")); json.reserve(1200);
  json += String(productSettings.latitude, 6); json += F(",\"longitude\":"); json += String(productSettings.longitude, 6);
  json += F(",\"plantName\":\""); json += jsonEscape(productSettings.plantName); json += '"';
  json += F(",\"timezoneName\":\""); json += jsonEscape(productSettings.timezoneName); json += '"';
  preferences.begin("sma-monitor", true);
  const String configuredSsid = preferences.getString("ssid", "");
  const String configuredWifiPassword = preferences.getString("wifiPass", "");
  preferences.end();
  json += F(",\"network\":{\"ssid\":\""); json += jsonEscape(configuredSsid.c_str()); json += '"';
  json += F(",\"password\":\""); json += jsonEscape(configuredWifiPassword.c_str()); json += '"';
  json += F(",\"apSsid\":\""); json += jsonEscape(apSsid.c_str()); json += '"';
  json += F(",\"apPassword\":\""); json += jsonEscape(apPassword.c_str()); json += F("\"}");
  json += F(",\"otaPassword\":\""); json += jsonEscape(otaPassword.c_str()); json += '"';
  json += F(",\"adminPasswordConfigured\":"); json += otaPassword.isEmpty() ? F("false") : F("true");
  json += F(",\"maintenanceMode\":"); json += productSettings.maintenanceMode ? F("true") : F("false");
  json += F(",\"timezone\":\""); json += jsonEscape(productSettings.timezone);
  json += F("\",\"mqtt\":{\"enabled\":"); json += mqttOutput.config().enabled ? F("true") : F("false");
  json += F(",\"broker\":\""); json += jsonEscape(mqttOutput.config().broker); json += F("\",\"port\":"); json += mqttOutput.config().port;
  json += F(",\"username\":\""); json += jsonEscape(mqttOutput.config().username); json += F("\",\"prefix\":\""); json += jsonEscape(mqttOutput.config().topicPrefix);
  json += F("\",\"password\":\""); json += jsonEscape(mqttOutput.config().password); json += '"';
  json += F(",\"connected\":"); json += mqttOutput.connected() ? F("true") : F("false");
  json += F(",\"interval\":"); json += mqttOutput.config().publishIntervalSeconds; json += F("},\"inverters\":[");
  for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
    if (i) json += ','; const auto& inv = productSettings.inverters[i];
    json += F("{\"enabled\":"); json += inv.enabled ? F("true") : F("false");
    json += F(",\"name\":\""); json += jsonEscape(inv.name); json += '"';
    json += F(",\"mac\":\""); json += jsonEscape(inv.mac); json += '"';
    json += F(",\"serial\":"); json += inv.serial;
    json += F(",\"password\":\""); json += jsonEscape(inv.userPassword); json += '"';
    const auto& snapshot = inverterSnapshots[i];
    json += F(",\"type\":");
    if (snapshot.inverterTypeState == SbfspotCompat::ValueState::Valid) {
      json += '"'; json += jsonEscape(snapshot.inverterType); json += '"';
    } else json += F("null");
    json += F(",\"softwareVersion\":");
    if (snapshot.inverterSoftwareVersionState == SbfspotCompat::ValueState::Valid) {
      json += '"'; json += jsonEscape(snapshot.inverterSoftwareVersion); json += '"';
    } else json += F("null");
    json += '}';
  }
  json += F("]}"); return json;
}

String schedulerStatusJson() {
  String json(F("{\"soakStartMs\":"));
  json.reserve(3600);
  json += soakStats.startMs;
  json += F(",\"soakStartUnix\":"); json += static_cast<unsigned long>(soakStats.startUnix);
  json += F(",\"state\":\""); json += schedulerStateName(schedulerState);
  json += F("\",\"maintenanceMode\":"); json += productSettings.maintenanceMode ? F("true") : F("false");
  json += F(",\"nextSlot\":"); json += static_cast<unsigned>(schedulerNextSlot + 1);
  json += F(",\"nextScheduledAtMs\":"); json += schedulerNextSlotAt;
  json += F(",\"scheduledSlots\":"); json += soakStats.scheduledSlots;
  json += F(",\"successfulSlots\":"); json += soakStats.successfulSlots;
  json += F(",\"failedSlots\":"); json += soakStats.failedSlots;
  json += F(",\"skippedSlots\":"); json += soakStats.skippedSlots;
  json += F(",\"secondAttemptsUsed\":"); json += soakStats.secondAttemptsUsed;
  json += F(",\"btCleanupFailures\":"); json += soakStats.btCleanupFailures;
  json += F(",\"networkRestoreFailures\":"); json += soakStats.networkRestoreFailures;
  json += F(",\"heapIntegrityFailures\":"); json += soakStats.heapIntegrityFailures;
  json += F(",\"mqttPublishFailures\":");
  json += mqttOutput.publishFailures() - soakStats.mqttPublishFailuresAtStart;
  json += F(",\"memory\":{\"initialInternalFree\":"); json += soakStats.initialInternalFree;
  json += F(",\"minimumInternalFree\":"); json += soakStats.minimumInternalFree;
  json += F(",\"currentInternalFree\":"); json += soakStats.currentInternalFree;
  json += F(",\"initialLargestBlock\":"); json += soakStats.initialLargestBlock;
  json += F(",\"minimumLargestBlock\":"); json += soakStats.minimumLargestBlock;
  json += F(",\"currentLargestBlock\":"); json += soakStats.currentLargestBlock;
  json += F("},\"inverters\":[");
  for (size_t slot = 0; slot < ProductConfig::kInverterCount; ++slot) {
    if (slot) json += ',';
    const auto& stats = soakStats.inverter[slot];
    json += F("{\"slot\":"); json += static_cast<unsigned>(slot + 1);
    json += F(",\"slots\":"); json += stats.slots;
    json += F(",\"firstAttemptSuccess\":"); json += stats.firstAttemptSuccess;
    json += F(",\"secondAttemptSuccess\":"); json += stats.secondAttemptSuccess;
    json += F(",\"completeFailures\":"); json += stats.completeFailures;
    json += F(",\"skipped\":"); json += stats.skipped;
    json += F(",\"consecutiveFailures\":"); json += stats.consecutiveFailures;
    json += F(",\"maxConsecutiveFailures\":"); json += stats.maxConsecutiveFailures;
    json += F(",\"lastResult\":\""); json += stats.lastResult;
    json += F("\",\"lastSuccessTimestamp\":"); json += static_cast<unsigned long>(stats.lastSuccessTimestamp);
    json += F(",\"lastPactotW\":"); json += stats.lastPactotW;
    json += '}';
  }
  json += F("],\"events\":[");
  const size_t oldest = (soakStats.eventNext + 16 - soakStats.eventCount) % 16;
  for (size_t index = 0; index < soakStats.eventCount; ++index) {
    if (index) json += ',';
    const auto& event = soakStats.events[(oldest + index) % 16];
    json += F("{\"atMs\":"); json += event.atMs;
    json += F(",\"atUnix\":"); json += static_cast<unsigned long>(event.atUnix);
    json += F(",\"slot\":"); json += static_cast<unsigned>(event.slot + 1);
    json += F(",\"event\":\""); json += event.event; json += F("\"}");
  }
  json += F("]}");
  return json;
}

String smaStatusJson() {
  // BluetoothSerial::end() deletes its internal SPP EventGroup.  In
  // Arduino-ESP32 3.3.11 connected() does not guard that null handle, so the
  // status endpoint must use our lifecycle state while Bluetooth is off.
  const bool bluetoothConnected = bluetoothReady && smaClient.bluetoothConnected();
  String json;
  json.reserve(4300);
  json += F("{\"selectedSlot\":"); json += static_cast<unsigned>(smaSelectedSlot + 1);
  json += F(",\"expectedSerial\":"); json += smaClient.expectedSerial();
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
  json += F(",\"acPowerClassification\":\""); json += smaClient.acPowerClassification(); json += '"';
  json += F(",\"acPowerW\":"); if (smaClient.acPowerValid()) json += smaClient.acPowerW(); else json += F("null");
  json += F(",\"acPowerPacketId\":"); json += smaClient.acPowerPacketId();
  json += F(",\"returnedLri\":"); json += smaClient.returnedLri();
  json += F(",\"recordType\":"); json += smaClient.returnedRecordType();
  json += F(",\"recordSize\":"); json += smaClient.returnedRecordSize();
  json += F(",\"measurementTimestamp\":"); json += smaClient.measurementTimestamp(); json += '}';
  json += F(",\"dataset\":{\"nextQueryIndex\":"); json += smaClient.datasetQueryIndex();
  json += F(",\"lastQueryIndex\":"); json += smaClient.lastDatasetQueryIndex();
  json += F(",\"lastDecodeResult\":");
  json += static_cast<unsigned>(smaClient.lastDatasetDecodeResult());
  json += F(",\"captures\":[");
  for (size_t index = 0; index < SmaBluetoothClient::DATASET_DIAGNOSTIC_COUNT; ++index) {
    if (index) json += ',';
    json += F("{\"index\":"); json += static_cast<unsigned>(index);
    json += F(",\"l1Fragments\":"); json += smaClient.datasetL1Fragments(index);
    json += F(",\"l1Bytes\":"); json += smaClient.datasetL1Bytes(index);
    json += F(",\"l2Packets\":"); json += smaClient.datasetL2Packets(index);
    json += F(",\"l2Bytes\":"); json += smaClient.datasetL2Bytes(index);
    json += F(",\"decode\":");
    json += static_cast<unsigned>(smaClient.datasetDecodeResult(index)); json += '}';
  }
  json += F("]}");
  json += F(",\"acquisitionMemory\":{\"beforeInternal\":"); json += smaAcquisitionMemory.beforeInternal;
  json += F(",\"beforeLargest\":"); json += smaAcquisitionMemory.beforeLargest;
  json += F(",\"btReadyInternal\":"); json += smaAcquisitionMemory.btReadyInternal;
  json += F(",\"btReadyLargest\":"); json += smaAcquisitionMemory.btReadyLargest;
  json += F(",\"rfcommInternal\":"); json += smaAcquisitionMemory.rfcommInternal;
  json += F(",\"rfcommLargest\":"); json += smaAcquisitionMemory.rfcommLargest;
  json += F(",\"minimumHeap\":"); json += smaAcquisitionMemory.minimumHeap;
  json += F(",\"btEndInternal\":"); json += smaAcquisitionMemory.btEndInternal;
  json += F(",\"btEndLargest\":"); json += smaAcquisitionMemory.btEndLargest;
  json += F(",\"restoredInternal\":"); json += smaAcquisitionMemory.restoredInternal;
  json += F(",\"restoredLargest\":"); json += smaAcquisitionMemory.restoredLargest;
  json += F(",\"guardPassed\":"); json += smaAcquisitionMemory.guardPassed ? F("true") : F("false");
  json += F(",\"heapIntegrity\":"); json += smaAcquisitionMemory.heapIntegrity ? F("true") : F("false");
  json += '}';
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

const char PAGE_V3[] PROGMEM = R"HTML(<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>SMA Monitor</title><style>*{box-sizing:border-box}:root{--bg:#eef3f7;--card:#fff;--ink:#17212b;--muted:#667580;--accent:#0874c9}body{margin:0;background:var(--bg);color:var(--ink);font:14px system-ui}header{background:#173149;color:#fff}.bar,.wrap{max-width:1180px;margin:auto;padding:14px 18px}.bar{display:flex;align-items:center;gap:20px}.brand{font-size:19px;font-weight:750;margin-right:auto}nav{display:flex;gap:4px;flex-wrap:wrap}nav button{border:0;background:transparent;color:#dce9f4;padding:9px;border-radius:7px}nav button.active{background:#ffffff20;color:#fff}.wrap{padding-top:20px}.page{display:none}.page.active{display:block}.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:14px}.card,.panel{background:var(--card);border-radius:13px;box-shadow:0 3px 14px #1b304012;padding:17px}.hero{font-size:38px;font-weight:750;margin:12px 0}.metrics{display:grid;grid-template-columns:1fr auto;gap:8px 12px;border-top:1px solid #e5ebef;padding-top:12px}.metrics b{text-align:right}.chips{display:flex;gap:7px;flex-wrap:wrap;margin-bottom:14px}.chip,.badge{background:#dfe8ef;padding:5px 9px;border-radius:99px;font-size:12px}.ok{background:#dff4ea;color:#087147}.partial{background:#fff0d6;color:#8b5300}.muted{color:var(--muted)}form{max-width:720px}.row{display:grid;grid-template-columns:1fr 1fr;gap:12px}label{display:block;margin:10px 0}input,select,button{font:inherit;padding:9px;border:1px solid #cbd6de;border-radius:7px}input:not([type=checkbox]),select{width:100%}.secret{display:flex;gap:6px}.secret input{flex:1}.actions{display:flex;gap:8px;margin-top:16px}.primary{background:var(--accent);color:#fff;border-color:var(--accent)}.dirty{color:#a76500}table{width:100%;border-collapse:collapse}td,th{padding:7px;border-bottom:1px solid #e4eaee;text-align:left}pre{white-space:pre-wrap;max-height:360px;overflow:auto;font-size:12px}@media(max-width:800px){.bar{align-items:flex-start;flex-direction:column}.brand{margin:0}.grid,.row{grid-template-columns:1fr}}</style></head><body><header><div class="bar"><div class="brand">SMA SunnyBoy Monitor</div><nav id="nav"></nav></div></header><div class="wrap"><div id="notice" class="muted"></div>
<section id="Dashboard" class="page active"><div id="chips" class="chips"></div><div id="totals" class="panel"></div><br><div id="cards" class="grid"></div></section>
<section id="Inverters" class="page"><h2>Onduleurs</h2><form id="inverterForm"><div id="invFields"></div><div class="actions"><button class="primary">Enregistrer</button><button type="button" class="cancel">Annuler</button><span class="dirty"></span></div></form></section>
<section id="Network" class="page"><h2>Réseau</h2><div id="networkState" class="panel"></div><form id="networkForm"><label>Réseau Wi-Fi <span id="wifiConfigured"></span><select name="ssid" id="ssid"></select></label><button type="button" id="scan">Scanner</button><label>Nouveau mot de passe Wi-Fi<div class="secret"><input name="password" type="password" maxlength="64" autocomplete="new-password"><button type="button" class="reveal">Afficher</button></div><small>Vide : conserver le secret configuré.</small></label><label>SSID du point d'accès de secours<input name="apSsid" maxlength="32"></label><label>Nouveau mot de passe AP<div class="secret"><input name="apPassword" type="password" maxlength="63" autocomplete="new-password"><button type="button" class="reveal">Afficher</button></div><small>Vide : conserver le secret configuré.</small></label><div class="actions"><button class="primary">Enregistrer</button><button type="button" class="cancel">Annuler</button><span class="dirty"></span></div><p class="muted">Redémarrage requis pour appliquer le réseau/AP.</p></form></section>
<section id="MQTT" class="page"><h2>MQTT</h2><div id="mqttState" class="panel"></div><form id="mqttForm"><label><input name="enabled" type="checkbox" value="1"> Activé</label><div class="row"><label>Broker<input name="broker" maxlength="63"></label><label>Port<input name="port" type="number" min="1" max="65535"></label></div><label>Utilisateur<input name="username" maxlength="39"></label><label>Nouveau mot de passe<div class="secret"><input name="password" type="password" maxlength="63" autocomplete="new-password"><button type="button" class="reveal">Afficher</button></div><small>Vide : conserver le secret configuré.</small></label><label>Préfixe des topics<input name="prefix" maxlength="47"></label><label>Intervalle (s)<input name="interval" type="number" min="10" max="86400"></label><div class="actions"><button class="primary">Enregistrer</button><button type="button" class="cancel">Annuler</button><span class="dirty"></span></div></form></section>
<section id="System" class="page"><h2>Système</h2><div id="clock" class="panel"></div><form id="systemForm"><label>Nom de l'installation<input name="plantName" maxlength="39"></label><div class="row"><label>Latitude<input name="latitude" type="number" step=".000001" min="-90" max="90"></label><label>Longitude<input name="longitude" type="number" step=".000001" min="-180" max="180"></label></div><label>Fuseau horaire<select name="timezoneName"><option>Europe/Brussels</option><option>UTC</option></select></label><label><input name="maintenanceMode" type="checkbox" value="1"> Mode maintenance</label><p class="muted">Suspend les acquisitions planifiées, mais maintient Wi-Fi, Web, MQTT et OTA.</p><label>Nouveau mot de passe OTA<div class="secret"><input name="otaPassword" type="password" maxlength="63" autocomplete="new-password"><button type="button" class="reveal">Afficher</button></div><small>Vide : conserver le secret configuré. Redémarrage requis après remplacement.</small></label><div class="actions"><button class="primary">Enregistrer</button><button type="button" class="cancel">Annuler</button><span class="dirty"></span></div></form><hr><h3>Mise à jour firmware (.bin)</h3><form id="otaForm"><label>Mot de passe OTA<input name="auth" type="password" required></label><input name="firmware" type="file" accept=".bin" required><button class="primary">Installer</button><progress id="progress" max="100" value="0"></progress><span id="otaResult"></span></form></section>
<section id="Diagnostics" class="page"><h2>Diagnostics</h2><pre id="diag"></pre><pre id="logs"></pre></section></div><script>
const pages=['Dashboard','Inverters','Network','MQTT','System','Diagnostics'],q=s=>document.querySelector(s),qa=s=>[...document.querySelectorAll(s)];let cfg,status,dash,scheduler,sma;nav.innerHTML=pages.map(x=>`<button data-page="${x}">${x}</button>`).join('');function show(p){qa('.page').forEach(x=>x.classList.toggle('active',x.id===p));qa('nav button').forEach(x=>x.classList.toggle('active',x.dataset.page===p))}nav.onclick=e=>{if(e.target.dataset.page)show(e.target.dataset.page)};show('Dashboard');const val=(v,u='',d)=>v==null?'—':`${d==null?v:Number(v).toFixed(d)}${u?' '+u:''}`;async function get(u){let r=await fetch(u);if(!r.ok)throw Error(await r.text());return r.json()}async function post(u,f){let r=await fetch(u,{method:'POST',body:new FormData(f)});if(!r.ok)throw Error(await r.text());return r.json()}function card(x){let partial=x.acquisitionResult==='partial';return `<article class="card"><div><b>${x.name}</b> <span class="badge ${partial?'partial':'ok'}">${partial?'Partiel / nuit':'Complet'}</span></div><div class="hero">${val(x.pac,'W')}</div><div class="metrics"><span>DC total</span><b>${val(x.pdcTotal,'W')}</b><span>EToday / ETotal</span><b>${val(x.eToday,'kWh',3)} / ${val(x.eTotal,'kWh',3)}</b><span>AC L1</span><b>${val(x.pac1,'W')}</b><span>AC V / A / Hz</span><b>${val(x.uac,'V',2)} / ${val(x.iac,'A',3)} / ${val(x.frequency,'Hz',2)}</b><span>DC V / A</span><b>${val(x.udc,'V',2)} / ${val(x.idc,'A',3)}</b><span>Température</span><b>${val(x.temperature,'°C',2)}</b><span>Statut / relais</span><b>${x.status??'—'} / ${x.gridRelay??'—'}</b></div><p class="muted">${x.lastSuccess??'Jamais'} · ${x.inverterType??'—'}</p></article>`}function fill(){let f=q('#networkForm');f.ssid.innerHTML=`<option>${cfg.network.ssid}</option>`;f.apSsid.value=cfg.network.apSsid;wifiConfigured.textContent=cfg.network.passwordConfigured?'— mot de passe configuré':'';f=q('#mqttForm');f.enabled.checked=cfg.mqtt.enabled;['broker','port','username','prefix'].forEach(k=>f[k].value=cfg.mqtt[k]);f.interval.value=cfg.mqtt.interval;invFields.innerHTML=cfg.inverters.map((x,n)=>`<div class="panel"><h3>INV${n+1}</h3><label><input name="inv${n+1}Enabled" type="checkbox" value="1" ${x.enabled?'checked':''}> Activé</label><label>Nom convivial<input name="inv${n+1}Name" value="${x.name}" maxlength="23"></label><label>Adresse Bluetooth<input name="inv${n+1}Mac" value="${x.mac}" maxlength="17"></label><label>Numéro de série<input name="inv${n+1}Serial" value="${x.serial}" type="number"></label><label>Nouveau mot de passe SMA USER<div class="secret"><input name="inv${n+1}Password" type="password" maxlength="12" autocomplete="new-password"><button type="button" class="reveal">Afficher</button></div><small>${x.passwordConfigured?'Configuré — ':''}vide : conserver.</small></label></div>`).join('');f=q('#systemForm');['plantName','latitude','longitude','timezoneName'].forEach(k=>f[k].value=cfg[k]);f.maintenanceMode.checked=cfg.maintenanceMode}async function refresh(){try{[cfg,status,dash,scheduler,sma]=await Promise.all(['/api/config','/api/status','/api/dashboard','/api/scheduler/status','/api/sma/status'].map(get));chips.innerHTML=`<span class="chip ${status.wifi.connected?'ok':''}">WiFi ${status.wifi.connected?'OK':'OFF'}</span><span class="chip ${status.mqtt.mqttConnected?'ok':''}">MQTT ${status.mqtt.mqttConnected?'OK':'OFF'}</span><span class="chip">${scheduler.state}</span><span class="chip">${status.firmwareVersion}</span>`;cards.innerHTML=dash.inverters.map(card).join('');let p=dash.inverters.map(x=>x.pac),e=dash.inverters.map(x=>x.eToday);totals.textContent=`Puissance totale : ${p.every(x=>x!=null)?p.reduce((a,b)=>a+b,0)+' W':'—'} · EToday : ${e.every(x=>x!=null)?e.reduce((a,b)=>a+b,0).toFixed(3)+' kWh':'—'}`;networkState.textContent=`STA ${status.wifi.connected?'connecté '+status.wifi.ip:'déconnecté'} · AP ${status.wifi.apIp??'inactif'}`;mqttState.textContent=`${status.mqtt.mqttConnected?'Connecté':'Déconnecté'} · publications ${status.mqtt.publishCount} · erreurs ${status.mqtt.publishFailures}`;clock.textContent=`${status.time.local} · ${status.time.timezone} · UTC ${status.time.utcOffset} · DST ${status.time.dst?'actif':'inactif'} · NTP ${status.time.synchronized?'OK':'non synchronisé'} · lever ${status.time.sunrise??'—'} · coucher ${status.time.sunset??'—'}`;diag.textContent=JSON.stringify({status,scheduler,sma},null,2);logs.textContent=(await get('/api/log')).lines.join('\n');fill()}catch(e){notice.textContent=e}}qa('form:not(#otaForm)').forEach(f=>{f.oninput=()=>q(`#${f.id} .dirty`).textContent='Modifications non enregistrées';f.querySelector('.cancel').onclick=()=>{fill();q(`#${f.id} .dirty`).textContent=''}});q('#networkForm').onsubmit=async e=>{e.preventDefault();await post('/api/network/config',e.target);notice.textContent='Réseau enregistré — redémarrage requis'};q('#mqttForm').onsubmit=async e=>{e.preventDefault();await post('/api/mqtt/config',e.target);notice.textContent='MQTT enregistré';await refresh()};q('#inverterForm').onsubmit=async e=>{e.preventDefault();await post('/api/inverters/config',e.target);notice.textContent='Onduleurs enregistrés';await refresh()};q('#systemForm').onsubmit=async e=>{e.preventDefault();let r=await post('/api/system/config',e.target);notice.textContent=r.restartRequired?'Système enregistré — redémarrage requis':'Système enregistré';await refresh()};document.body.onclick=e=>{if(e.target.classList.contains('reveal')){let i=e.target.previousElementSibling;i.type=i.type==='password'?'text':'password';e.target.textContent=i.type==='password'?'Afficher':'Masquer'}};scan.onclick=async()=>{scan.disabled=true;let n=await get('/api/wifi/networks');ssid.innerHTML=n.networks.map(x=>`<option>${x.ssid}</option>`).join('');scan.disabled=false};q('#otaForm').onsubmit=e=>{e.preventDefault();let f=e.target,fd=new FormData();fd.append('firmware',f.firmware.files[0]);let x=new XMLHttpRequest();x.open('POST','/api/firmware');x.setRequestHeader('Authorization','Basic '+btoa('admin:'+f.auth.value));x.upload.onprogress=p=>progress.value=p.lengthComputable?p.loaded*100/p.total:0;x.onload=()=>otaResult.textContent=x.status===200?'Installé, redémarrage…':'Échec '+x.responseText;x.onerror=()=>otaResult.textContent='Connexion interrompue';x.send(fd)};refresh();setInterval(refresh,10000);</script></body></html>)HTML";

String renderedPage() {
  String page(FPSTR(PAGE_V3));
  // Configuration forms are filled once so periodic operational refreshes do
  // not erase unsaved edits.
  page.replace("fill()}catch(e)",
               "if(!window.configLoaded){fill();window.configLoaded=true}}catch(e)");
  page.replace("Nouveau mot de passe Wi-Fi", "Mot de passe Wi-Fi");
  page.replace("Nouveau mot de passe AP", "Mot de passe AP");
  page.replace("Nouveau mot de passe<div", "Mot de passe<div");
  page.replace("Nouveau mot de passe SMA USER", "Mot de passe SMA USER");
  page.replace("Nouveau mot de passe OTA", "Mot de passe OTA");
  page.replace("<small>Vide : conserver le secret configuré.</small>", "");
  page.replace("<small>${x.passwordConfigured?'Configuré — ':''}vide : conserver.</small>", "");
  page.replace("<small>Vide : conserver le secret configuré. Redémarrage requis après remplacement.</small>",
               "<small>Redémarrage requis après modification.</small>");
  page.replace("<label>Mot de passe OTA<div class=\"secret\"><input name=\"otaPassword\"",
               "<div class=\"panel\"><b>Un seul mot de passe administrateur / OTA</b><p class=\"muted\">Il protège la connexion Web admin, ArduinoOTA et l'installation d'un fichier .bin depuis le navigateur. Sur un appareil neuf il est vide : connectez-vous avec admin et un mot de passe vide, puis définissez-en un ici. Tant qu'il reste vide, les deux mises à jour OTA sont désactivées.</p></div><label>Modifier le mot de passe administrateur / OTA<div class=\"secret\"><input name=\"otaPassword\"");
  page.replace("f.apSsid.value=cfg.network.apSsid;wifiConfigured.textContent=cfg.network.passwordConfigured?'— mot de passe configuré':'';",
               "f.apSsid.value=cfg.network.apSsid;f.password.value=cfg.network.password;f.apPassword.value=cfg.network.apPassword;wifiConfigured.textContent='';");
  page.replace("f.interval.value=cfg.mqtt.interval;invFields.innerHTML=",
               "f.interval.value=cfg.mqtt.interval;f.password.value=cfg.mqtt.password;invFields.innerHTML=");
  page.replace("autocomplete=\"new-password\"", "autocomplete=\"current-password\"");
  page.replace(").join('');f=q('#systemForm')",
               ").join('');cfg.inverters.forEach((x,n)=>q('#inverterForm')[`inv${n+1}Password`].value=x.password);f=q('#systemForm')");
  page.replace("f.maintenanceMode.checked=cfg.maintenanceMode}",
               "f.maintenanceMode.checked=cfg.maintenanceMode;f.otaPassword.value=cfg.otaPassword;q('#otaForm').auth.value=cfg.otaPassword}");
  page.replace("<label>Mot de passe OTA<input name=\"auth\" type=\"password\" required></label>",
               "<label>Confirmation du mot de passe administrateur / OTA<div class=\"secret\"><input name=\"auth\" type=\"password\" required><button type=\"button\" class=\"reveal\">Afficher</button></div><small>Ce champ ne crée pas un second mot de passe : il autorise uniquement cette installation.</small></label>");
  page.replace("<section id=\"Inverters\" class=\"page\"><h2>Onduleurs</h2>",
               "<section id=\"Inverters\" class=\"page\"><h2>Onduleurs</h2><div class=\"panel\"><h3>Découverte Bluetooth Classic</h3><p class=\"muted\">Le réseau sera brièvement indisponible pendant le scan; le scheduler reprendra automatiquement.</p><button type=\"button\" id=\"btScan\">Scanner Bluetooth</button> <span id=\"btScanState\"></span><div id=\"btResults\"></div><div class=\"row\"><label>Assigner à<select id=\"assignSlot\"><option value=\"1\">INV1</option><option value=\"2\">INV2</option><option value=\"3\">INV3</option></select></label><div class=\"actions\"><button type=\"button\" id=\"assignBt\" disabled>Assigner le périphérique sélectionné</button></div></div><hr><div class=\"row\"><label>Vérifier<select id=\"testSlot\"><option value=\"1\">INV1</option><option value=\"2\">INV2</option><option value=\"3\">INV3</option></select></label><div class=\"actions\"><button type=\"button\" id=\"testInv\">Tester la connexion</button></div></div><div id=\"testInvState\" class=\"muted\"></div></div><br>");
  page.replace("<label>Adresse Bluetooth<input name=\"inv${n+1}Mac\"",
               "<label>Adresse Bluetooth (information technique)<input readonly name=\"inv${n+1}Mac\"");
  page.replace("<label>Numéro de série<input name=\"inv${n+1}Serial\"",
               "<label>Numéro de série détecté<input readonly name=\"inv${n+1}Serial\"");
  page.replace("<label>Mot de passe SMA USER",
               "<p class=\"muted\">Type/version : ${x.type??'—'} / ${x.softwareVersion??'—'}</p><label>Mot de passe SMA USER");
  page.replace("q('#inverterForm').onsubmit=",
               "async function postData(u,v){let r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(v)});if(!r.ok)throw Error(await r.text());return r.json()}function renderBt(r){btScanState.textContent=r.active?'Scan en cours…':r.state==='COMPLETE'?`${r.devices.length} périphérique(s) trouvé(s)`:`État : ${r.state}`;btResults.innerHTML=r.devices.length?`<table><tr><th></th><th>Nom</th><th>MAC</th><th>RSSI</th><th>Information fiable</th></tr>${r.devices.map(d=>`<tr><td><input type=\"radio\" name=\"btChoice\" value=\"${d.mac}\"></td><td>${d.name??'Inconnu'}</td><td><code>${d.mac}</code></td><td>${d.rssi??'—'}</td><td>${d.sma?'Configuré : '+d.sma.label:'—'}</td></tr>`).join('')}</table>`:'<p class=\"muted\">Aucun résultat.</p>';qa('input[name=btChoice]').forEach(x=>x.onchange=()=>assignBt.disabled=false)}async function pollBt(){try{let r=await get('/api/bt/results');renderBt(r);if(r.active)setTimeout(pollBt,2000);else btScan.disabled=false}catch(e){btScanState.textContent='Réseau interrompu pendant le scan…';setTimeout(pollBt,3000)}}btScan.onclick=async()=>{btScan.disabled=true;assignBt.disabled=true;btScanState.textContent='Préparation du scan…';try{await postData('/api/bt/scan',{});setTimeout(pollBt,1500)}catch(e){btScan.disabled=false;btScanState.textContent=e}};assignBt.onclick=async()=>{let d=q('input[name=btChoice]:checked');if(!d)return;assignBt.disabled=true;try{await postData('/api/inverters/assign',{slot:assignSlot.value,mac:d.value});cfg=await get('/api/config');window.configLoaded=false;fill();window.configLoaded=true;btScanState.textContent=`Assigné à INV${assignSlot.value}`}catch(e){btScanState.textContent=e}finally{assignBt.disabled=false}};async function pollInvTest(){try{let r=await get('/api/inverters/test/status');testInvState.textContent=r.active?'Test SMA en cours…':r.result==='PASS'?`PASS — SN ${r.detectedSerial??'—'} — ${r.type??'type inconnu'} — ${r.softwareVersion??'version inconnue'}`:`${r.result}`;if(r.active)setTimeout(pollInvTest,2500);else{testInv.disabled=false;cfg=await get('/api/config');window.configLoaded=false;fill();window.configLoaded=true}}catch(e){testInvState.textContent='Réseau interrompu pendant le test…';setTimeout(pollInvTest,3000)}}testInv.onclick=async()=>{testInv.disabled=true;testInvState.textContent='Préparation du test…';try{await postData('/api/inverters/test',{slot:testSlot.value});setTimeout(pollInvTest,1500)}catch(e){testInv.disabled=false;testInvState.textContent=e}};q('#inverterForm').onsubmit=");
  return page;
#if 0
  page.replace("<h2>Onduleurs</h2>",
               "<h2>Onduleurs</h2><div id=\"scheduler\" class=\"muted\"></div>");
  page.replace("<fieldset><legend>NETWORK</legend>",
               "<fieldset><legend>SCHEDULER</legend><label><input name=\"maintenanceMode\" type=\"checkbox\" value=\"1\"> Maintenance mode</label></fieldset><fieldset><legend>NETWORK</legend>");
  page.replace("f.latitude.value=c.latitude;",
               "f.maintenanceMode.checked=c.maintenanceMode;f.latitude.value=c.latitude;");
  page.replace("cards.innerHTML=d.inverters.map(card).join('');",
               "scheduler.textContent=`Scheduler: ${d.scheduler.state} | next INV${d.scheduler.nextSlot} | last: ${d.scheduler.lastResult}`;cards.innerHTML=d.inverters.map(card).join('');");
  page.replace("readP.hidden=false;readPState.hidden=false;",
               "readP.hidden=true;readPState.hidden=true;");
  page.replace("</form></section><section><h2>Diagnostic",
               String("</form></section>") + FPSTR(WIFI_CONFIGURATION_SECTION) + "<section><h2>Diagnostic");
  return page;
#endif
}

void sendJson(const String& value, int status = 200) { server.send(status, "application/json", value); }

void noteHttpRequest(const char* route) {
  (void)route;
  if (scanState == ScanState::SCANNING) ++httpDuringScanCount;
}

bool requireWebConfigAuthentication() {
  if (server.authenticate("admin", otaPassword.c_str())) return true;
  server.requestAuthentication();
  return false;
}

void registerRoutes() {
  server.on("/", HTTP_GET, [] {
    if (!requireWebConfigAuthentication()) return;
    noteHttpRequest("/"); server.send(200, "text/html; charset=utf-8", renderedPage());
  });
  server.on("/api/status", HTTP_GET, [] { noteHttpRequest("status"); sendJson(statusJson()); });
  server.on("/api/dashboard", HTTP_GET, [] { noteHttpRequest("dashboard"); sendJson(dashboardJson()); });
  server.on("/api/config", HTTP_GET, [] {
    if (!requireWebConfigAuthentication()) return;
    noteHttpRequest("config"); sendJson(publicConfigJson());
  });
  server.on("/api/bt/results", HTTP_GET, [] {
    if (!requireWebConfigAuthentication()) return;
    noteHttpRequest("results"); sendJson(resultsJson());
  });
  server.on("/api/log", HTTP_GET, [] { noteHttpRequest("log"); sendJson(logJson()); });
  server.on("/api/sma/status", HTTP_GET, [] { noteHttpRequest("sma_status"); sendJson(smaStatusJson()); });
  server.on("/api/scheduler/status", HTTP_GET, [] {
    noteHttpRequest("scheduler_status"); sendJson(schedulerStatusJson());
  });
  server.on("/api/mqtt/config", HTTP_POST, [] {
    if (!requireWebConfigAuthentication()) return;
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
    strlcpy(config.password, password.c_str(), sizeof(config.password));
    if (!mqttOutput.save(preferences, config, true)) {
      sendJson("{\"error\":\"invalid_mqtt_configuration\"}", 400); return;
    }
    addLog("[MQTT] configuration saved enabled=%s broker=%s port=%u prefix=%s interval=%lu",
           config.enabled ? "true" : "false", config.broker, config.port, config.topicPrefix,
           static_cast<unsigned long>(config.publishIntervalSeconds));
    sendJson("{\"saved\":true}");
  });
  server.on("/api/network/config", HTTP_POST, [] {
    if (!requireWebConfigAuthentication()) return;
    const String ssid = server.arg("ssid"), password = server.arg("password");
    const String fallbackSsid = server.arg("apSsid"), fallbackPassword = server.arg("apPassword");
    if (ssid.isEmpty() || ssid.length() > 32 || password.length() > 64 ||
        fallbackSsid.isEmpty() || fallbackSsid.length() > 32 ||
        (!fallbackPassword.isEmpty() && !ProductConfig::validWpaPassword(fallbackPassword.c_str()))) {
      sendJson("{\"error\":\"invalid_network_configuration\"}", 400); return;
    }
    preferences.begin("sma-monitor", false);
    preferences.putString("ssid", ssid);
    preferences.putString("wifiPass", password);
    preferences.putString("apSsid", fallbackSsid);
    preferences.putString("apPass", fallbackPassword);
    preferences.end();
    sendJson("{\"saved\":true,\"restartRequired\":true}");
  });
  server.on("/api/inverters/config", HTTP_POST, [] {
    if (!requireWebConfigAuthentication()) return;
    ProductConfig::InverterSlot updated[ProductConfig::kInverterCount];
    memcpy(updated, productSettings.inverters, sizeof(updated));
    for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
      const String base = "inv" + String(i + 1);
      const String name = server.arg(base + "Name"), mac = server.arg(base + "Mac");
      const String password = server.arg(base + "Password");
      const uint32_t serial = strtoul(server.arg(base + "Serial").c_str(), nullptr, 10);
      if (name.isEmpty() || name.length() >= ProductConfig::kNameCapacity ||
          !ProductConfig::validMac(mac.c_str()) || !ProductConfig::validSerial(serial) ||
          password.length() > SmaPhase3::kPasswordLength) {
        sendJson("{\"error\":\"invalid_inverter_configuration\"}", 400); return;
      }
      updated[i].enabled = server.hasArg(base + "Enabled");
      strlcpy(updated[i].name, name.c_str(), sizeof(updated[i].name));
      strlcpy(updated[i].mac, mac.c_str(), sizeof(updated[i].mac));
      updated[i].serial = serial;
      strlcpy(updated[i].userPassword, password.c_str(), sizeof(updated[i].userPassword));
    }
    preferences.begin("sma-monitor", false);
    for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
      char key[12]; snprintf(key, sizeof(key), "inv%uEn", static_cast<unsigned>(i + 1)); preferences.putBool(key, updated[i].enabled);
      snprintf(key, sizeof(key), "inv%uName", static_cast<unsigned>(i + 1)); preferences.putString(key, updated[i].name);
      snprintf(key, sizeof(key), "inv%uMac", static_cast<unsigned>(i + 1)); preferences.putString(key, updated[i].mac);
      snprintf(key, sizeof(key), "inv%uSerial", static_cast<unsigned>(i + 1)); preferences.putUInt(key, updated[i].serial);
      const String password = server.arg("inv" + String(i + 1) + "Password");
      snprintf(key, sizeof(key), "inv%uPass", static_cast<unsigned>(i + 1)); preferences.putString(key, password);
    }
    preferences.end(); memcpy(productSettings.inverters, updated, sizeof(updated));
    refreshInverterSnapshot(); sendJson("{\"saved\":true}");
  });
  server.on("/api/system/config", HTTP_POST, [] {
    if (!requireWebConfigAuthentication()) return;
    const String plant = server.arg("plantName"), zone = server.arg("timezoneName");
    const String otaCandidate = server.arg("otaPassword");
    const double latitude = server.arg("latitude").toDouble(), longitude = server.arg("longitude").toDouble();
    const char* posix = zone == "UTC" ? "UTC0" :
                        zone == "Europe/Brussels" ? "CET-1CEST,M3.5.0,M10.5.0/3" : nullptr;
    if (plant.length() >= sizeof(productSettings.plantName) || !posix ||
        !ProductConfig::validLatitude(latitude) || !ProductConfig::validLongitude(longitude) ||
        !ProductConfig::validAdminPassword(otaCandidate.c_str())) {
      sendJson("{\"error\":\"invalid_system_configuration\"}", 400); return;
    }
    preferences.begin("sma-monitor", false);
    preferences.putString("plantName", plant); preferences.putDouble("latitude", latitude);
    preferences.putDouble("longitude", longitude); preferences.putString("tzName", zone);
    preferences.putString("timezone", posix); preferences.putBool("maintenance", server.hasArg("maintenanceMode"));
    preferences.putString("otaPass", otaCandidate);
    preferences.end();
    strlcpy(productSettings.plantName, plant.c_str(), sizeof(productSettings.plantName));
    productSettings.latitude = latitude; productSettings.longitude = longitude;
    strlcpy(productSettings.timezoneName, zone.c_str(), sizeof(productSettings.timezoneName));
    strlcpy(productSettings.timezone, posix, sizeof(productSettings.timezone));
    productSettings.maintenanceMode = server.hasArg("maintenanceMode");
    setenv("TZ", productSettings.timezone, 1); tzset();
    sendJson("{\"saved\":true,\"restartRequired\":true}");
  });
  server.on("/api/firmware", HTTP_POST, [] {
    if (otaPassword.isEmpty()) { sendJson("{\"error\":\"admin_password_required\"}", 403); return; }
    if (!server.authenticate("admin", otaPassword.c_str())) { server.requestAuthentication(); return; }
    const bool ok = !Update.hasError();
    sendJson(ok ? "{\"updated\":true,\"restarting\":true}" : "{\"error\":\"update_failed\"}", ok ? 200 : 500);
    if (ok) { delay(250); ESP.restart(); }
    else otaBusy = false;
  }, [] {
    if (otaPassword.isEmpty()) return;
    if (!server.authenticate("admin", otaPassword.c_str())) return;
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      if (bluetoothReady || schedulerAcquisitionActive || otaBusy) { Update.abort(); return; }
      otaBusy = true;
      if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (!Update.end(true)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      Update.abort(); otaBusy = false;
    }
  });
  server.on("/api/product/config", HTTP_POST, [] {
    if (!requireWebConfigAuthentication()) return;
    const bool maintenanceMode = server.hasArg("maintenanceMode");
    const double latitude = server.arg("latitude").toDouble();
    const double longitude = server.arg("longitude").toDouble();
    const String timezone = server.arg("timezone");
    const String apCandidate = server.arg("apPassword");
    const String otaCandidate = server.arg("otaPassword");
    const String plantName = server.arg("plantName");
    if (!ProductConfig::validLatitude(latitude) || !ProductConfig::validLongitude(longitude) ||
        timezone.isEmpty() || timezone.length() >= sizeof(productSettings.timezone) ||
        plantName.length() >= sizeof(productSettings.plantName) ||
        (!apCandidate.isEmpty() && !ProductConfig::validWpaPassword(apCandidate.c_str())) ||
        (!otaCandidate.isEmpty() && !ProductConfig::validOtaPassword(otaCandidate.c_str()))) {
      sendJson("{\"error\":\"invalid_product_configuration\"}", 400); return;
    }
    ProductConfig::InverterSlot slots[ProductConfig::kInverterCount];
    for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
      const String base = "inv" + String(i + 1);
      slots[i].enabled = server.hasArg(base + "Enabled");
      const uint32_t serial = strtoul(server.arg(base + "Serial").c_str(), nullptr, 10);
      if (!ProductConfig::validMac(productSettings.inverters[i].mac) ||
          !ProductConfig::validSerial(serial)) {
        sendJson("{\"error\":\"invalid_inverter_configuration\"}", 400); return;
      }
      strlcpy(slots[i].mac, productSettings.inverters[i].mac, sizeof(slots[i].mac));
      slots[i].serial = serial;
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
    preferences.putString("plantName", plantName);
    preferences.putBool("maintenance", maintenanceMode);
    preferences.putString("timezone", timezone);
    if (!apCandidate.isEmpty()) preferences.putString("apPass", apCandidate);
    if (!otaCandidate.isEmpty()) preferences.putString("otaPass", otaCandidate);
    for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
      char key[12]; snprintf(key, sizeof(key), "inv%uEn", static_cast<unsigned>(i + 1)); preferences.putBool(key, slots[i].enabled);
      snprintf(key, sizeof(key), "inv%uMac", static_cast<unsigned>(i + 1)); preferences.putString(key, slots[i].mac);
      snprintf(key, sizeof(key), "inv%uSerial", static_cast<unsigned>(i + 1)); preferences.putUInt(key, slots[i].serial);
    }
    preferences.end();
    productSettings.maintenanceMode = maintenanceMode;
    strlcpy(productSettings.plantName, plantName.c_str(), sizeof(productSettings.plantName));
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
    const long requestedSlot = server.arg("slot").toInt();
    if (requestedSlot < 1 || requestedSlot > static_cast<long>(ProductConfig::kInverterCount)) {
      sendJson("{\"error\":\"invalid_inverter_slot\"}", 400); return;
    }
    const size_t slot = static_cast<size_t>(requestedSlot - 1);
    if (!productSettings.inverters[slot].enabled) {
      sendJson("{\"error\":\"inverter_slot_disabled\"}", 409); return;
    }
    if (!startInverterIdentityTest(slot)) {
      if (smaLifecycleState == SmaLifecycleState::BACKOFF) {
        sendJson("{\"error\":\"sma_backoff_active\"}", 429);
      } else {
        sendJson("{\"error\":\"sma_lifecycle_busy\"}", 409);
      }
      return;
    }
    String response(F("{\"accepted\":true,\"state\":\"NETWORK_QUIESCE\",\"slot\":"));
    response += static_cast<unsigned>(slot + 1); response += '}'; sendJson(response, 202);
  });
  server.on("/api/bt/scan", HTTP_POST, [] {
    if (!requireWebConfigAuthentication()) return;
    noteHttpRequest("scan");
    if (managedScanActive || scanState == ScanState::PREPARING || scanState == ScanState::SCANNING) {
      sendJson("{\"error\":\"scan_already_running\"}", 409); return;
    }
    if (otaBusy) { sendJson("{\"error\":\"ota_in_progress\"}", 409); return; }
    if (!startManagedBtScan()) { sendJson("{\"error\":\"scan_busy\"}", 409); return; }
    sendJson("{\"accepted\":true,\"state\":\"PREPARING\"}", 202);
  });
  server.on("/api/inverters/assign", HTTP_POST, [] {
    if (!requireWebConfigAuthentication()) return;
    const long requestedSlot = server.arg("slot").toInt();
    String mac = server.arg("mac"); mac.toUpperCase();
    if (requestedSlot < 1 || requestedSlot > static_cast<long>(ProductConfig::kInverterCount) ||
        !ProductConfig::validMac(mac.c_str()) || managedScanActive) {
      sendJson("{\"error\":\"invalid_assignment\"}", 400); return;
    }
    bool discovered = false;
    portENTER_CRITICAL(&resultsMux);
    for (size_t i = 0; i < btResultCount; ++i)
      if (strcmp(btResults[i].mac, mac.c_str()) == 0) { discovered = true; break; }
    portEXIT_CRITICAL(&resultsMux);
    if (!discovered) { sendJson("{\"error\":\"device_not_in_scan\"}", 409); return; }
    const size_t slot = static_cast<size_t>(requestedSlot - 1);
    for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
      if (i != slot && strcmp(productSettings.inverters[i].mac, mac.c_str()) == 0) {
        sendJson("{\"error\":\"device_already_assigned\"}", 409); return;
      }
    }
    strlcpy(productSettings.inverters[slot].mac, mac.c_str(),
            sizeof(productSettings.inverters[slot].mac));
    char key[12]; snprintf(key, sizeof(key), "inv%uMac", static_cast<unsigned>(slot + 1));
    preferences.begin("sma-monitor", false); preferences.putString(key, mac); preferences.end();
    addLog("[CONFIG] discovered Bluetooth device assigned to slot=%u",
           static_cast<unsigned>(slot + 1));
    sendJson("{\"saved\":true}");
  });
  server.on("/api/inverters/test", HTTP_POST, [] {
    if (!requireWebConfigAuthentication()) return;
    const long requestedSlot = server.arg("slot").toInt();
    if (requestedSlot < 1 || requestedSlot > static_cast<long>(ProductConfig::kInverterCount) ||
        managedScanActive || inverterTestActive || schedulerAcquisitionActive) {
      sendJson("{\"error\":\"test_busy_or_invalid\"}", 409); return;
    }
    const size_t slot = static_cast<size_t>(requestedSlot - 1);
    if (!startSafeSmaAcquisition(slot)) {
      sendJson("{\"error\":\"test_start_failed\"}", 409); return;
    }
    inverterTestSlot = slot; inverterTestStartedAt = millis(); inverterTestCompletedAt = 0;
    strlcpy(inverterTestResult, "RUNNING", sizeof(inverterTestResult));
    inverterTestActive = true;
    sendJson("{\"accepted\":true,\"result\":\"RUNNING\"}", 202);
  });
  server.on("/api/inverters/test/status", HTTP_GET, [] {
    if (!requireWebConfigAuthentication()) return;
    String json(F("{\"active\":")); json += inverterTestActive ? F("true") : F("false");
    json += F(",\"slot\":"); json += static_cast<unsigned>(inverterTestSlot + 1);
    json += F(",\"result\":\""); json += inverterTestResult;
    json += F("\",\"startedAtMs\":"); json += inverterTestStartedAt;
    json += F(",\"completedAtMs\":"); json += inverterTestCompletedAt;
    json += F(",\"detectedSerial\":");
    if (smaClient.decodedSerial()) json += smaClient.decodedSerial(); else json += F("null");
    const auto& snapshot = inverterSnapshots[inverterTestSlot];
    json += F(",\"type\":");
    if (snapshot.inverterTypeState == SbfspotCompat::ValueState::Valid) {
      json += '"'; json += jsonEscape(snapshot.inverterType); json += '"';
    } else json += F("null");
    json += F(",\"softwareVersion\":");
    if (snapshot.inverterSoftwareVersionState == SbfspotCompat::ValueState::Valid) {
      json += '"'; json += jsonEscape(snapshot.inverterSoftwareVersion); json += '"';
    } else json += F("null");
    json += F("}"); sendJson(json);
  });
  server.on("/api/wifi/networks", HTTP_GET, [] {
    if (!requireWebConfigAuthentication()) return;
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
  const bool passwordKeyPresent = preferences.isKey("otaPass");
  const String storedPassword = passwordKeyPresent ? preferences.getString("otaPass", "") : String();
  char resolved[64]{};
  if (!ProductConfig::resolveAdminPassword(passwordKeyPresent, storedPassword.c_str(),
                                           resolved, sizeof(resolved))) {
    resolved[0] = '\0';
  }
  otaPassword = resolved;
  if (!passwordKeyPresent) preferences.putString("otaPass", "");
  preferences.end();
  otaServiceReady = false;
  if (otaPassword.isEmpty()) {
    addLog("[OTA] disabled until administrator password is configured");
    return;
  }
  ArduinoOTA.setHostname(hostname.c_str());
  ArduinoOTA.setPassword(otaPassword.c_str());
  ArduinoOTA.onStart([] { otaBusy = true; addLog("[OTA] start"); });
  ArduinoOTA.onEnd([] { addLog("[OTA] complete; rebooting"); });
  ArduinoOTA.onError([](ota_error_t error) { otaBusy = false; addLog("[OTA] error=%u", static_cast<unsigned>(error)); });
  ArduinoOTA.begin();
  otaServiceReady = true;
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
  if (productSettings.apSsid[0]) apSsid = productSettings.apSsid;
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
  configTzTime(productSettings.timezone, "pool.ntp.org", "time.nist.gov");
  smaClient.setAppSerial(APP_SERIAL_MIN + (esp_random() % APP_SERIAL_RANGE));
  addLog("[TIME] SNTP configured synchronized=%s unix=%lld app_serial=%lu",
         timeSynchronized() ? "true" : "false", static_cast<long long>(currentUnixTime()),
         static_cast<unsigned long>(smaClient.appSerial()));
  registerRoutes(); server.begin(); addLog("[WEB] ready port=80");
  configureOta();
  mqttOutput.load(preferences);
  mqttOutput.begin();
  refreshInverterSnapshot();
  soakStats = {};
  soakStats.startMs = millis();
  soakStats.startUnix = currentUnixTime();
  soakStats.mqttPublishFailuresAtStart = mqttOutput.publishFailures();
  soakStats.initialInternalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  soakStats.initialLargestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  soakStats.minimumInternalFree = soakStats.currentInternalFree = soakStats.initialInternalFree;
  soakStats.minimumLargestBlock = soakStats.currentLargestBlock = soakStats.initialLargestBlock;
  addLog("[MQTT] ready enabled=%s configured=%s broker=%s port=%u prefix=%s",
         mqttOutput.config().enabled ? "true" : "false", mqttOutput.configured() ? "true" : "false",
         mqttOutput.config().broker, mqttOutput.config().port, mqttOutput.config().topicPrefix);
  bluetoothReady = false;
  smaLifecycleState = SmaLifecycleState::BT_OFF;
  schedulerNextSlotAt = bootAt + SCHEDULER_BOOT_GRACE_MS;
  schedulerNextSlot = 0;
  schedulerState = productSettings.maintenanceMode ? SchedulerState::MAINTENANCE
                                                    : SchedulerState::GRACE;
  addLog("[SCHED] state=%s grace_ms=%lu spacing_ms=%lu next_slot=1 next_at=%lu",
         schedulerStateName(schedulerState), static_cast<unsigned long>(SCHEDULER_BOOT_GRACE_MS),
         static_cast<unsigned long>(SCHEDULER_SLOT_SPACING_MS),
         static_cast<unsigned long>(schedulerNextSlotAt));
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
  serviceNetworkAudit();
  serviceSmaLifecycle();
  serviceScheduler();
  serviceBtScan();
  if (otaServiceReady && scanState != ScanState::SCANNING) ArduinoOTA.handle();
  refreshInverterSnapshot();
  const bool networkAuditSuppressMqtt = networkAuditState != NetworkAuditState::IDLE &&
                                        networkAuditState != NetworkAuditState::COMPLETE &&
                                        networkAuditState != NetworkAuditState::FAILED &&
                                        networkAuditState != NetworkAuditState::WAIT_MQTT;
  const bool suppressMqtt = networkAuditSuppressMqtt;
  mqttOutput.tick(WiFi.status() == WL_CONNECTED && !suppressMqtt, otaBusy, inverterSnapshots);
  const wl_status_t wifiStatus = WiFi.status();
  if (wifiStatus != lastWifiStatus) {
    addLog("[WIFI] status_change old=%d new=%d scan=%s", static_cast<int>(lastWifiStatus),
           static_cast<int>(wifiStatus), scanStateName(scanState));
    lastWifiStatus = wifiStatus;
  }
  const bool networkAuditOwnsWifi = networkAuditState != NetworkAuditState::IDLE &&
                                    networkAuditState != NetworkAuditState::COMPLETE &&
                                    networkAuditState != NetworkAuditState::FAILED &&
                                    networkAuditState != NetworkAuditState::WAIT_MQTT;
  if (!networkAuditOwnsWifi && WiFi.status() != WL_CONNECTED &&
      static_cast<int32_t>(millis() - nextStaAttemptAt) >= 0) connectSta();
  updateSoakMemory();
  delay(2);
}
