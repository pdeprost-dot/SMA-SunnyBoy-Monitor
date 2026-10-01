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
  RFCOMM_HOLD, RFCOMM_DISCONNECT, HOLD, REUSE_WAIT, RFCOMM2_CONNECTING,
  RFCOMM2_HOLD, RFCOMM2_DISCONNECT, END, COMPLETE, FAILED
};
enum class NetworkAuditState : uint8_t {
  IDLE, MQTT_SETTLE, OTA_MDNS_SETTLE, WEB_SETTLE, WIFI_SETTLE, WAIT_BT,
  WAIT_SMA, WAIT_MQTT, RESTART, COMPLETE, FAILED
};
enum class CombinedState : uint8_t {
  IDLE, QUIESCING_NETWORK, ACQUIRING_INV1, ACQUIRING_INV2, ACQUIRING_INV3,
  RESTORING_NETWORK, COMPLETE, FAILED
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
struct CombinedInverterResult {
  uint32_t startedAt = 0;
  uint32_t rfcommOpenAt = 0;
  uint32_t completedAt = 0;
  uint32_t durationMs = 0;
  uint32_t openInternal = 0;
  uint32_t openLargest = 0;
  uint32_t afterDisconnectInternal = 0;
  uint32_t afterDisconnectLargest = 0;
  uint32_t minimumHeap = 0;
  uint32_t pactotW = 0;
  bool identity = false;
  bool login = false;
  bool pactotValid = false;
  bool heapIntegrity = false;
};
struct ReconnectReadinessSample {
  uint32_t offsetMs = 0;
  uint32_t internalFree = 0;
  uint32_t largestBlock = 0;
  bool sppCloseSeen = false;
  bool connected = false;
  bool closed = false;
  bool ready = false;
  bool heapIntegrity = false;
};
struct RfcommDelayResult {
  uint32_t requestedDelayMs = 0;
  uint32_t actualDelayMs = 0;
  uint32_t inv1ConnectAt = 0;
  uint32_t inv1OpenAt = 0;
  uint32_t inv1CloseAt = 0;
  uint32_t afterCloseInternal = 0;
  uint32_t afterCloseLargest = 0;
  uint32_t beforeInv2Internal = 0;
  uint32_t beforeInv2Largest = 0;
  uint32_t inv2ConnectAt = 0;
  uint32_t inv2OpenAt = 0;
  bool inv1Open = false;
  bool closeReady = false;
  bool inv2Open = false;
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
bool otaBusy = false;
bool apActive = false;
volatile bool sppInitSeen = false;
volatile uint32_t sppCloseAt = 0;
volatile uint32_t sppCloseHandle = 0;
volatile bool sppCloseSeen = false;
BtOnlyTestState btOnlyState = BtOnlyTestState::IDLE;
bool btOnlyDisconnectMqtt = false;
bool btOnlyRfcomm = false;
bool btOnlyMqttWasConnected = false;
bool btOnlyHeapIntegrity = false;
volatile bool btOnlyConnectDone = false;
volatile bool btOnlyConnectResult = false;
volatile UBaseType_t btOnlyConnectStackHwm = 0;
TaskHandle_t btOnlyConnectTaskHandle = nullptr;
uint8_t btOnlyTargetSlot = 0;
bool btReuseDiagnostic = false;
RfcommDelayResult rfcommDelayResult;
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
constexpr uint32_t SCHEDULER_BOOT_GRACE_MS = 60000;
constexpr uint32_t SCHEDULER_SLOT_SPACING_MS = 120000;
constexpr uint32_t SMA_DISCONNECT_GUARD_MS = 3000;
constexpr uint32_t SMA_RECONNECT_DIAGNOSTIC_MS = 10000;
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
size_t smaSelectedSlot = 0;
AcquisitionMemory smaAcquisitionMemory;
CombinedState combinedState = CombinedState::IDLE;
CombinedInverterResult combinedResults[ProductConfig::kInverterCount];
bool combinedAcquisition = false;
bool combinedActive = false;
uint32_t combinedDisconnectReturnedAt = 0;
uint8_t reconnectSampleIndex = 0;
ReconnectReadinessSample reconnectSamples[8];
constexpr uint32_t RECONNECT_SAMPLE_OFFSETS_MS[8] = {0, 250, 500, 1000, 2000, 3000, 5000, 10000};
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
uint32_t combinedCycleStartedAt = 0;
uint32_t combinedWifiOffAt = 0;
uint32_t combinedBtReadyAt = 0;
uint32_t combinedBtOffAt = 0;
uint32_t combinedIpRestoredAt = 0;
uint32_t combinedWebRestoredAt = 0;
uint32_t combinedMqttRestoredAt = 0;
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
  }
  if (smaClient.phase3Complete() && smaClient.datasetUsable()) {
    SbfspotCompat::InverterSnapshot& snapshot = inverterSnapshots[smaSelectedSlot];
    const auto& values = smaClient.measurements();
    ProductConfig::formatLocalTime(now, snapshot.timestamp, sizeof(snapshot.timestamp));
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
    if (values.gridRelay.state == SmaPhase3::FieldState::Valid)
      formatKnownAttribute(values.gridRelay.raw, "Closed", 51,
                           snapshot.inverterGridRelay, sizeof(snapshot.inverterGridRelay));
    snapshot.acquisitionValid = true;
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

const char* combinedStateName(CombinedState value) {
  switch (value) {
    case CombinedState::IDLE: return "IDLE";
    case CombinedState::QUIESCING_NETWORK: return "QUIESCING_NETWORK";
    case CombinedState::ACQUIRING_INV1: return "ACQUIRING_INV1";
    case CombinedState::ACQUIRING_INV2: return "ACQUIRING_INV2";
    case CombinedState::ACQUIRING_INV3: return "ACQUIRING_INV3";
    case CombinedState::RESTORING_NETWORK: return "RESTORING_NETWORK";
    case CombinedState::COMPLETE: return "COMPLETE";
    case CombinedState::FAILED: return "FAILED";
  }
  return "IDLE";
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
    case BtOnlyTestState::REUSE_WAIT: return "REUSE_WAIT";
    case BtOnlyTestState::RFCOMM2_CONNECTING: return "RFCOMM2_CONNECTING";
    case BtOnlyTestState::RFCOMM2_HOLD: return "RFCOMM2_HOLD";
    case BtOnlyTestState::RFCOMM2_DISCONNECT: return "RFCOMM2_DISCONNECT";
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
      smaAcquisitionMemory.rfcommInternal =
          heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      smaAcquisitionMemory.rfcommLargest =
          heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
      if (combinedAcquisition && smaSelectedSlot < ProductConfig::kInverterCount) {
        combinedResults[smaSelectedSlot].rfcommOpenAt = millis();
        combinedResults[smaSelectedSlot].openInternal = smaAcquisitionMemory.rfcommInternal;
        combinedResults[smaSelectedSlot].openLargest = smaAcquisitionMemory.rfcommLargest;
      }
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

bool startSafeSmaAcquisition(size_t slot) {
  if (slot >= ProductConfig::kInverterCount || !productSettings.inverters[slot].enabled) return false;
  if ((networkAuditState != NetworkAuditState::IDLE &&
       networkAuditState != NetworkAuditState::COMPLETE &&
       networkAuditState != NetworkAuditState::FAILED) ||
      bluetoothReady || smaLifecycleState != SmaLifecycleState::BT_OFF) return false;
  const auto& inverter = productSettings.inverters[slot];
  if (!ProductConfig::validMac(inverter.mac) || !ProductConfig::validSerial(inverter.serial) ||
      !smaClient.setTarget(inverter.mac, inverter.serial)) return false;
  const uint32_t beforeInternal =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t beforeLargest =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!startNetworkAudit(true, true, false)) return false;
  combinedAcquisition = false;
  combinedActive = false;
  combinedState = CombinedState::IDLE;
  smaSelectedSlot = slot;
  smaAcquisitionMemory = {};
  smaAcquisitionMemory.beforeInternal = beforeInternal;
  smaAcquisitionMemory.beforeLargest = beforeLargest;
  networkAuditSma = true;
  addLog("[ACQUIRE] safe SMA PACTot cycle requested slot=%u",
         static_cast<unsigned>(slot + 1));
  return true;
}

bool startCombinedSmaAcquisition() {
  if ((networkAuditState != NetworkAuditState::IDLE &&
       networkAuditState != NetworkAuditState::COMPLETE &&
       networkAuditState != NetworkAuditState::FAILED) ||
      bluetoothReady || smaLifecycleState != SmaLifecycleState::BT_OFF) return false;
  for (size_t slot = 0; slot < ProductConfig::kInverterCount; ++slot) {
    const auto& inverter = productSettings.inverters[slot];
    if (!inverter.enabled || !ProductConfig::validMac(inverter.mac) ||
        !ProductConfig::validSerial(inverter.serial)) return false;
  }
  if (!smaClient.setTarget(productSettings.inverters[0].mac,
                           productSettings.inverters[0].serial)) return false;
  const uint32_t beforeInternal =
      heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const uint32_t beforeLargest =
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!startNetworkAudit(true, true, false)) return false;
  smaSelectedSlot = 0;
  smaAcquisitionMemory = {};
  smaAcquisitionMemory.beforeInternal = beforeInternal;
  smaAcquisitionMemory.beforeLargest = beforeLargest;
  memset(combinedResults, 0, sizeof(combinedResults));
  memset(reconnectSamples, 0, sizeof(reconnectSamples));
  reconnectSampleIndex = 0;
  combinedDisconnectReturnedAt = 0;
  sppCloseAt = 0;
  sppCloseHandle = 0;
  sppCloseSeen = false;
  combinedCycleStartedAt = millis();
  combinedWifiOffAt = combinedBtReadyAt = combinedBtOffAt = 0;
  combinedIpRestoredAt = combinedWebRestoredAt = combinedMqttRestoredAt = 0;
  combinedAcquisition = true;
  combinedActive = true;
  combinedState = CombinedState::QUIESCING_NETWORK;
  networkAuditSma = true;
  addLog("[COMBINED] cycle requested");
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
  const bool btOnlyBusy = btOnlyState != BtOnlyTestState::IDLE &&
                          btOnlyState != BtOnlyTestState::COMPLETE &&
                          btOnlyState != BtOnlyTestState::FAILED;
  const bool networkBusy = networkAuditState != NetworkAuditState::IDLE &&
                           networkAuditState != NetworkAuditState::COMPLETE &&
                           networkAuditState != NetworkAuditState::FAILED;
  if (networkBusy || btOnlyBusy || bluetoothReady || smaLifecycleState != SmaLifecycleState::BT_OFF) {
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
      internalFree < PREVIOUS_BT_READY_INTERNAL_FREE + RFCOMM_DIAGNOSTIC_MIN_GAIN ||
      largest < PREVIOUS_BT_READY_INTERNAL_LARGEST + RFCOMM_DIAGNOSTIC_MIN_GAIN) {
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
  ArduinoOTA.begin();
  if (combinedAcquisition) {
    combinedWebRestoredAt = millis();
    combinedState = CombinedState::RESTORING_NETWORK;
  }
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
          if (combinedAcquisition) combinedWifiOffAt = millis();
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
      if (smaLifecycleState == SmaLifecycleState::BACKOFF && schedulerRetryPending) {
        if (static_cast<int32_t>(now - schedulerRetryAt) < 0) break;
        const uint32_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        const uint32_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        const bool integrity = heap_caps_check_integrity_all(true);
        if (bluetoothReady || !integrity ||
            internalFree < PREVIOUS_BT_READY_INTERNAL_FREE + RFCOMM_DIAGNOSTIC_MIN_GAIN ||
            largest < PREVIOUS_BT_READY_INTERNAL_LARGEST + RFCOMM_DIAGNOSTIC_MIN_GAIN) {
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
        if (combinedAcquisition && WiFi.status() == WL_CONNECTED)
          combinedIpRestoredAt = millis();
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
      if (!mqttOutput.config().enabled || mqttOutput.connected()) {
        addLog("[NET-AUDIT] mqtt_restore_ms=%lu",
               static_cast<unsigned long>(millis() - networkAuditRestoreStartedAt));
        logBtStartResources("network_restored");
        smaAcquisitionMemory.restoredInternal =
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        smaAcquisitionMemory.restoredLargest =
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (combinedAcquisition) {
          combinedMqttRestoredAt = millis();
          mqttOutput.requestPublishAll();
          combinedState = smaLifecycleLastError[0] ? CombinedState::FAILED : CombinedState::COMPLETE;
          combinedActive = false;
          combinedAcquisition = false;
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

void btOnlyConnectTask(void*) {
  recordCheckpoint(CrashCheckpoint::RFCOMM_CONNECT_ENTER);
  addLog("[BT-ONLY] rfcomm_connect_enter t=%lu slot=%u channel=1",
         static_cast<unsigned long>(millis()), static_cast<unsigned>(btOnlyTargetSlot + 1));
  uint8_t target[6]{};
  unsigned octets[6]{};
  const char* mac = productSettings.inverters[btOnlyTargetSlot].mac;
  const bool parsed = sscanf(mac, "%02x:%02x:%02x:%02x:%02x:%02x", &octets[0], &octets[1],
                             &octets[2], &octets[3], &octets[4], &octets[5]) == 6;
  if (parsed) for (size_t index = 0; index < 6; ++index) target[index] = octets[index];
  btOnlyConnectResult = parsed && serialBt.connect(target, 1);
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
  btOnlyTargetSlot = 0;
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
          const bool characterizationMargin = !btReuseDiagnostic ||
              (internalFree >= 22U * 1024U && internalLargest >= 16U * 1024U);
          addLog("[BT-ONLY] rfcomm_gate gain_free=%d gain_largest=%d result=%s",
                 static_cast<int>(internalFree) - static_cast<int>(PREVIOUS_BT_READY_INTERNAL_FREE),
                 static_cast<int>(internalLargest) - static_cast<int>(PREVIOUS_BT_READY_INTERNAL_LARGEST),
                 materiallyImproved ? "GO" : "NO_GO");
          if (!materiallyImproved || !characterizationMargin || !btOnlyHeapIntegrity) {
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
        if (btReuseDiagnostic) rfcommDelayResult.inv1ConnectAt = now;
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
        if (btReuseDiagnostic) {
          rfcommDelayResult.inv1Open = btOnlyConnectResult;
          if (btOnlyConnectResult) rfcommDelayResult.inv1OpenAt = now;
        }
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
      sppCloseSeen = false;
      sppCloseAt = 0;
      addLog("[BT-ONLY] rfcomm_disconnect result=%s",
             serialBt.disconnect() ? "true" : "false");
      logBtStartResources("after_rfcomm_disconnect");
      btOnlyStateAt = now;
      if (btReuseDiagnostic) {
        rfcommDelayResult.inv1CloseAt = sppCloseAt;
        rfcommDelayResult.afterCloseInternal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        rfcommDelayResult.afterCloseLargest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        rfcommDelayResult.closeReady = sppCloseSeen && !serialBt.connected(0) &&
                                       serialBt.isClosed() && serialBt.isReady(false, 0);
        rfcommDelayResult.heapIntegrity = heap_caps_check_integrity_all(true);
        btOnlyState = BtOnlyTestState::REUSE_WAIT;
      } else {
        btOnlyState = BtOnlyTestState::HOLD;
      }
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
    case BtOnlyTestState::REUSE_WAIT:
      if (!rfcommDelayResult.closeReady || !rfcommDelayResult.heapIntegrity) {
        failBtOnlyTest("reuse_close_not_ready");
        break;
      }
      if (now - rfcommDelayResult.inv1CloseAt >= rfcommDelayResult.requestedDelayMs) {
        rfcommDelayResult.actualDelayMs = now - rfcommDelayResult.inv1CloseAt;
        rfcommDelayResult.beforeInv2Internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        rfcommDelayResult.beforeInv2Largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        rfcommDelayResult.heapIntegrity = heap_caps_check_integrity_all(true);
        if (rfcommDelayResult.beforeInv2Internal < 22U * 1024U ||
            rfcommDelayResult.beforeInv2Largest < 16U * 1024U || !rfcommDelayResult.heapIntegrity ||
            !sppCloseSeen || serialBt.connected(0) || !serialBt.isClosed() || !serialBt.isReady(false, 0)) {
          failBtOnlyTest("reuse_memory_or_state_no_go");
          break;
        }
        btOnlyTargetSlot = 1;
        btOnlyConnectDone = false;
        btOnlyConnectResult = false;
        rfcommDelayResult.inv2ConnectAt = now;
        if (xTaskCreatePinnedToCore(btOnlyConnectTask, "bt-reuse-connect", 4096, nullptr, 1,
                                    &btOnlyConnectTaskHandle, 0) != pdPASS) {
          failBtOnlyTest("reuse_task_create_failed");
          break;
        }
        btOnlyStateAt = now;
        btOnlyState = BtOnlyTestState::RFCOMM2_CONNECTING;
      }
      break;
    case BtOnlyTestState::RFCOMM2_CONNECTING:
      if (btOnlyConnectDone) {
        rfcommDelayResult.inv2Open = btOnlyConnectResult;
        if (btOnlyConnectResult) rfcommDelayResult.inv2OpenAt = now;
        addLog("[BT-REUSE] inv2_result=%s connect_ms=%lu", btOnlyConnectResult ? "OPEN" : "FAIL",
               static_cast<unsigned long>(now - rfcommDelayResult.inv2ConnectAt));
        btOnlyStateAt = now;
        btOnlyState = btOnlyConnectResult ? BtOnlyTestState::RFCOMM2_HOLD : BtOnlyTestState::END;
      } else if (now - btOnlyStateAt >= 15000) {
        failBtOnlyTest("reuse_rfcomm_task_timeout");
      }
      break;
    case BtOnlyTestState::RFCOMM2_HOLD:
      if (now - btOnlyStateAt >= 250) btOnlyState = BtOnlyTestState::RFCOMM2_DISCONNECT;
      break;
    case BtOnlyTestState::RFCOMM2_DISCONNECT:
      addLog("[BT-REUSE] inv2_disconnect=%s", serialBt.disconnect() ? "true" : "false");
      btOnlyState = BtOnlyTestState::END;
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
  smaAcquisitionMemory.minimumHeap = ESP.getMinFreeHeap();
  smaAcquisitionMemory.heapIntegrity = heap_caps_check_integrity_all(true);
  recordCheckpoint(CrashCheckpoint::CLEANUP_START);
  strlcpy(smaLifecycleLastError, error && error[0] ? error : "unknown", sizeof(smaLifecycleLastError));
  if (combinedAcquisition) {
    combinedState = CombinedState::FAILED;
    combinedActive = false;
  }
  ++smaFailedConnections;
  setSmaLifecycleState(SmaLifecycleState::FAILED);
  if (smaClient.state() != SmaBluetoothClient::State::DISCONNECTED) smaClient.requestDisconnect();
  setSmaLifecycleState(SmaLifecycleState::BT_STOPPING);
  stopBluetoothService();
  if (combinedAcquisition) combinedBtOffAt = millis();
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
        logBtStartResources("sma_bt_ready");
        smaAcquisitionMemory.btReadyInternal =
            heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        smaAcquisitionMemory.btReadyLargest =
            heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (combinedAcquisition) combinedBtReadyAt = now;
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
            internalFree >= PREVIOUS_BT_READY_INTERNAL_FREE + RFCOMM_DIAGNOSTIC_MIN_GAIN &&
            internalLargest >= PREVIOUS_BT_READY_INTERNAL_LARGEST + RFCOMM_DIAGNOSTIC_MIN_GAIN;
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
      if (combinedAcquisition) {
        combinedResults[smaSelectedSlot].startedAt = now;
        combinedState = static_cast<CombinedState>(
            static_cast<uint8_t>(CombinedState::ACQUIRING_INV1) + smaSelectedSlot);
      }
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
        if (combinedAcquisition) {
          auto& result = combinedResults[smaSelectedSlot];
          result.completedAt = now;
          result.durationMs = now - result.startedAt;
          result.identity = smaClient.decodedSerial() == smaClient.expectedSerial();
          result.login = smaClient.loginValid();
          result.pactotValid = smaClient.acPowerValid();
          result.pactotW = smaClient.acPowerW();
          result.minimumHeap = ESP.getMinFreeHeap();
          result.heapIntegrity = smaAcquisitionMemory.heapIntegrity;
        }
        recordCheckpoint(CrashCheckpoint::CLEANUP_START);
        if (combinedAcquisition) {
          sppCloseSeen = false;
          sppCloseAt = 0;
          sppCloseHandle = 0;
        }
        if (!smaClient.requestDisconnect()) {
          failSmaLifecycle("disconnect_failed");
          break;
        }
        combinedDisconnectReturnedAt = millis();
        reconnectSampleIndex = 0;
        smaStopAt = combinedAcquisition && smaSelectedSlot == 0
                        ? combinedDisconnectReturnedAt + SMA_RECONNECT_DIAGNOSTIC_MS
                        : combinedDisconnectReturnedAt + SMA_DISCONNECT_GUARD_MS;
        setSmaLifecycleState(SmaLifecycleState::DISCONNECTING);
      } else if (smaClient.state() == SmaBluetoothClient::State::ERROR) {
        failSmaLifecycle(smaClient.lastError());
      }
      break;
    case SmaLifecycleState::DISCONNECTING:
      if (combinedAcquisition && smaSelectedSlot == 0 && combinedDisconnectReturnedAt) {
        const uint32_t elapsed = now - combinedDisconnectReturnedAt;
        while (reconnectSampleIndex < 8 &&
               elapsed >= RECONNECT_SAMPLE_OFFSETS_MS[reconnectSampleIndex]) {
          auto& sample = reconnectSamples[reconnectSampleIndex];
          sample.offsetMs = elapsed;
          sample.internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          sample.largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          sample.sppCloseSeen = sppCloseSeen;
          sample.connected = serialBt.connected(0);
          sample.closed = serialBt.isClosed();
          sample.ready = serialBt.isReady(false, 0);
          sample.heapIntegrity = heap_caps_check_integrity_all(true);
          addLog("[REUSE] target=%lu actual=%lu close=%s conn=%s closed=%s ready=%s free=%u largest=%u integrity=%s",
                 static_cast<unsigned long>(RECONNECT_SAMPLE_OFFSETS_MS[reconnectSampleIndex]),
                 static_cast<unsigned long>(elapsed), sample.sppCloseSeen ? "yes" : "no",
                 sample.connected ? "yes" : "no", sample.closed ? "yes" : "no",
                 sample.ready ? "yes" : "no", sample.internalFree, sample.largestBlock,
                 sample.heapIntegrity ? "ok" : "failed");
          ++reconnectSampleIndex;
        }
      }
      if (static_cast<int32_t>(now - smaStopAt) >= 0) {
        if (combinedAcquisition) {
          auto& result = combinedResults[smaSelectedSlot];
          result.afterDisconnectInternal =
              heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          result.afterDisconnectLargest =
              heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
          const bool integrity = heap_caps_check_integrity_all(true);
          const bool btUsable = serialBt.isReady(false, 0);
          const bool memoryReady =
              result.afterDisconnectInternal >= PREVIOUS_BT_READY_INTERNAL_FREE + RFCOMM_DIAGNOSTIC_MIN_GAIN &&
              result.afterDisconnectLargest >= PREVIOUS_BT_READY_INTERNAL_LARGEST + RFCOMM_DIAGNOSTIC_MIN_GAIN;
          addLog("[COMBINED] inter_guard slot=%u internal=%u largest=%u bt_ready=%s integrity=%s result=%s",
                 static_cast<unsigned>(smaSelectedSlot + 1), result.afterDisconnectInternal,
                 result.afterDisconnectLargest, btUsable ? "true" : "false",
                 integrity ? "ok" : "failed", btUsable && integrity && memoryReady ? "GO" : "NO_GO");
          if (smaSelectedSlot == 0) {
            const bool rfcommReusable = sppCloseSeen && !serialBt.connected(0) &&
                                        serialBt.isClosed() && btUsable;
            addLog("[REUSE] criterion close_at=%lu handle=%lu reusable=%s",
                   static_cast<unsigned long>(sppCloseAt),
                   static_cast<unsigned long>(sppCloseHandle), rfcommReusable ? "yes" : "no");
            if (!rfcommReusable || !integrity || !memoryReady) {
              failSmaLifecycle(!integrity ? "inter_inverter_heap_failed" : "inter_inverter_bt_not_ready");
              break;
            }
            const size_t nextSlot = smaSelectedSlot + 1;
            const auto& target = productSettings.inverters[nextSlot];
            if (!smaClient.setTarget(target.mac, target.serial)) {
              failSmaLifecycle("inter_inverter_target_failed");
              break;
            }
            smaSelectedSlot = nextSlot;
            combinedResults[smaSelectedSlot].startedAt = now;
            combinedState = static_cast<CombinedState>(
                static_cast<uint8_t>(CombinedState::ACQUIRING_INV1) + smaSelectedSlot);
            recordCheckpoint(CrashCheckpoint::RFCOMM_CONNECT_ENTER);
            if (!smaClient.requestConnect()) {
              failSmaLifecycle("inter_inverter_connect_rejected");
              break;
            }
            setSmaLifecycleState(SmaLifecycleState::CONNECTING);
            break;
          }
        }
        setSmaLifecycleState(SmaLifecycleState::BT_STOPPING);
        stopBluetoothService();
        if (combinedAcquisition) combinedBtOffAt = millis();
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
  json.reserve(2000);
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
  json += F(",\"reuseDiagnostic\":"); json += btReuseDiagnostic ? F("true") : F("false");
  json += F(",\"reuseResult\":{\"requestedDelayMs\":"); json += rfcommDelayResult.requestedDelayMs;
  json += F(",\"actualDelayMs\":"); json += rfcommDelayResult.actualDelayMs;
  json += F(",\"inv1ConnectAt\":"); json += rfcommDelayResult.inv1ConnectAt;
  json += F(",\"inv1OpenAt\":"); json += rfcommDelayResult.inv1OpenAt;
  json += F(",\"inv1CloseAt\":"); json += rfcommDelayResult.inv1CloseAt;
  json += F(",\"afterCloseInternal\":"); json += rfcommDelayResult.afterCloseInternal;
  json += F(",\"afterCloseLargest\":"); json += rfcommDelayResult.afterCloseLargest;
  json += F(",\"beforeInv2Internal\":"); json += rfcommDelayResult.beforeInv2Internal;
  json += F(",\"beforeInv2Largest\":"); json += rfcommDelayResult.beforeInv2Largest;
  json += F(",\"inv2ConnectAt\":"); json += rfcommDelayResult.inv2ConnectAt;
  json += F(",\"inv2OpenAt\":"); json += rfcommDelayResult.inv2OpenAt;
  json += F(",\"inv1Open\":"); json += rfcommDelayResult.inv1Open ? F("true") : F("false");
  json += F(",\"closeReady\":"); json += rfcommDelayResult.closeReady ? F("true") : F("false");
  json += F(",\"inv2Open\":"); json += rfcommDelayResult.inv2Open ? F("true") : F("false");
  json += F(",\"heapIntegrity\":"); json += rfcommDelayResult.heapIntegrity ? F("true") : F("false");
  json += '}';
  json += F("},\"ota\":{\"busy\":"); json += otaBusy ? F("true") : F("false"); json += F("}}");
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
  String json(F("{\"latitude\":")); json.reserve(900);
  json += String(productSettings.latitude, 6); json += F(",\"longitude\":"); json += String(productSettings.longitude, 6);
  json += F(",\"maintenanceMode\":"); json += productSettings.maintenanceMode ? F("true") : F("false");
  json += F(",\"timezone\":\""); json += jsonEscape(productSettings.timezone);
  json += F("\",\"mqtt\":{\"enabled\":"); json += mqttOutput.config().enabled ? F("true") : F("false");
  json += F(",\"broker\":\""); json += jsonEscape(mqttOutput.config().broker); json += F("\",\"port\":"); json += mqttOutput.config().port;
  json += F(",\"username\":\""); json += jsonEscape(mqttOutput.config().username); json += F("\",\"prefix\":\""); json += jsonEscape(mqttOutput.config().topicPrefix);
  json += F("\",\"interval\":"); json += mqttOutput.config().publishIntervalSeconds; json += F("},\"inverters\":[");
  for (size_t i = 0; i < ProductConfig::kInverterCount; ++i) {
    if (i) json += ','; const auto& inv = productSettings.inverters[i];
    json += F("{\"enabled\":"); json += inv.enabled ? F("true") : F("false");
    json += F(",\"serial\":"); json += inv.serial; json += '}';
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
  json += F(",\"combined\":{\"active\":"); json += combinedActive ? F("true") : F("false");
  json += F(",\"state\":\""); json += combinedStateName(combinedState); json += '"';
  json += F(",\"cycleStartedAt\":"); json += combinedCycleStartedAt;
  json += F(",\"wifiOffAt\":"); json += combinedWifiOffAt;
  json += F(",\"btReadyAt\":"); json += combinedBtReadyAt;
  json += F(",\"btOffAt\":"); json += combinedBtOffAt;
  json += F(",\"ipRestoredAt\":"); json += combinedIpRestoredAt;
  json += F(",\"webRestoredAt\":"); json += combinedWebRestoredAt;
  json += F(",\"mqttRestoredAt\":"); json += combinedMqttRestoredAt;
  json += F(",\"disconnectReturnedAt\":"); json += combinedDisconnectReturnedAt;
  json += F(",\"sppCloseAt\":"); json += static_cast<uint32_t>(sppCloseAt);
  json += F(",\"sppCloseHandle\":"); json += static_cast<uint32_t>(sppCloseHandle);
  json += F(",\"readinessSamples\":[");
  for (size_t index = 0; index < 8; ++index) {
    if (index) json += ',';
    const auto& sample = reconnectSamples[index];
    json += F("{\"targetMs\":"); json += RECONNECT_SAMPLE_OFFSETS_MS[index];
    json += F(",\"actualMs\":"); json += sample.offsetMs;
    json += F(",\"internalFree\":"); json += sample.internalFree;
    json += F(",\"largestBlock\":"); json += sample.largestBlock;
    json += F(",\"sppCloseSeen\":"); json += sample.sppCloseSeen ? F("true") : F("false");
    json += F(",\"connected\":"); json += sample.connected ? F("true") : F("false");
    json += F(",\"closed\":"); json += sample.closed ? F("true") : F("false");
    json += F(",\"ready\":"); json += sample.ready ? F("true") : F("false");
    json += F(",\"heapIntegrity\":"); json += sample.heapIntegrity ? F("true") : F("false");
    json += '}';
  }
  json += ']';
  json += F(",\"inverters\":[");
  for (size_t slot = 0; slot < ProductConfig::kInverterCount; ++slot) {
    if (slot) json += ',';
    const auto& result = combinedResults[slot];
    json += F("{\"slot\":"); json += static_cast<unsigned>(slot + 1);
    json += F(",\"startedAt\":"); json += result.startedAt;
    json += F(",\"rfcommOpenAt\":"); json += result.rfcommOpenAt;
    json += F(",\"completedAt\":"); json += result.completedAt;
    json += F(",\"durationMs\":"); json += result.durationMs;
    json += F(",\"openInternal\":"); json += result.openInternal;
    json += F(",\"openLargest\":"); json += result.openLargest;
    json += F(",\"afterDisconnectInternal\":"); json += result.afterDisconnectInternal;
    json += F(",\"afterDisconnectLargest\":"); json += result.afterDisconnectLargest;
    json += F(",\"minimumHeap\":"); json += result.minimumHeap;
    json += F(",\"acPowerW\":"); json += result.pactotW;
    json += F(",\"identityValid\":"); json += result.identity ? F("true") : F("false");
    json += F(",\"loginValid\":"); json += result.login ? F("true") : F("false");
    json += F(",\"acPowerValid\":"); json += result.pactotValid ? F("true") : F("false");
    json += F(",\"heapIntegrity\":"); json += result.heapIntegrity ? F("true") : F("false");
    json += '}';
  }
  json += F("]}");
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
const unavailable=v=>v===null||v===undefined?'--':v;async function get(u,o){const r=await fetch(u,o),t=await r.text();if(!r.ok)throw Error(t);return t?JSON.parse(t):{}}function card(x){return `<div class="card"><h3>${x.name}${x.inverterName?` — ${x.inverterName}`:''}</h3><div>Série : ${x.serial||'--'}</div><div>État : ${x.enabled?(x.dataValid?'données valides':'non acquis'):'désactivé'}</div><div>Dernière acquisition : ${x.lastSuccess??'--'}</div><div class="metrics"><span>PACTot / PAC1</span><span class="value">${unavailable(x.pac)} / ${unavailable(x.pac1)} W</span><span>EToday / ETotal</span><span>${unavailable(x.eToday)} / ${unavailable(x.eTotal)} kWh</span><span>UAC / IAC</span><span>${unavailable(x.uac)} V / ${unavailable(x.iac)} A</span><span>Température</span><span>${unavailable(x.temperature)} °C</span><span>Statut / relais</span><span>${unavailable(x.status)} / ${unavailable(x.gridRelay)}</span><span>Fréquence</span><span>${unavailable(x.frequency)} Hz</span><span>UDC / IDC / PDC</span><span>${unavailable(x.udc)} V / ${unavailable(x.idc)} A / ${unavailable(x.pdc)} W</span><span>Opération / injection</span><span>${unavailable(x.operationHours)} / ${unavailable(x.feedInHours)} h</span><span>Type / version</span><span>${unavailable(x.inverterType)} / ${unavailable(x.softwareVersion)}</span></div>${x.enabled?`<button type="button" onclick="readPower(${x.slot},this)">Lire le snapshot SMA</button>`:''}<div id="readState${x.slot}" class="muted"></div></div>`}async function readPower(slot,button){button.disabled=true;const state=document.getElementById(`readState${slot}`);state.textContent='Acquisition en cours — retour réseau automatique…';try{await get(`/api/sma/connect?slot=${slot}`,{method:'POST'});setTimeout(()=>{button.disabled=false;state.textContent=''},60000)}catch(e){button.disabled=false;state.textContent=e}}async function refresh(){try{const [s,d,l]=await Promise.all([get('/api/status'),get('/api/dashboard'),get('/api/log')]);status.textContent=JSON.stringify(s,null,2);cards.innerHTML=d.inverters.map(card).join('');log.textContent=l.lines.join('\n')}catch(e){status.textContent=e}}readP.hidden=true;readPState.hidden=true;invConfig.innerHTML=[1,2,3].map(i=>`<fieldset><legend>INV${i}</legend><label><input name="inv${i}Enabled" type="checkbox" value="1"> Activé</label><label>Numéro de série <input name="inv${i}Serial" type="number" min="1" max="4294967295"></label></fieldset>`).join('');async function loadConfig(){const c=await get('/api/config'),f=document.forms[0];f.latitude.value=c.latitude;f.longitude.value=c.longitude;f.timezone.value=c.timezone;f.mqttEnabled.checked=c.mqtt.enabled;f.broker.value=c.mqtt.broker;f.port.value=c.mqtt.port;f.username.value=c.mqtt.username;f.prefix.value=c.mqtt.prefix;f.interval.value=c.mqtt.interval;c.inverters.forEach((x,n)=>{const i=n+1;f[`inv${i}Enabled`].checked=x.enabled;f[`inv${i}Serial`].value=x.serial})}setInterval(refresh,3000);loadConfig();refresh();
readP.textContent='Lire les 3 onduleurs';readP.onclick=async()=>{readP.disabled=true;readPState.textContent='Acquisition combinee en cours - retour reseau automatique...';try{await get('/api/sma/connect-all',{method:'POST'});setTimeout(()=>{readP.disabled=false;readPState.textContent=''},90000)}catch(e){readP.disabled=false;readPState.textContent=e}};readP.hidden=false;readPState.hidden=false;
</script></body></html>)HTML";

const char WIFI_CONFIGURATION_SECTION[] PROGMEM = R"HTML(<section><h2>Wi-Fi</h2><form method="post" action="/api/wifi/config"><label>SSID <input name="ssid" maxlength="32"></label><label>Mot de passe Wi-Fi <input name="password" type="password" maxlength="64"></label><button>Enregistrer et connecter</button></form><p class="muted">La modification Wi-Fi redémarre l'appareil. Le mot de passe n'est jamais relu dans cette page.</p></section>)HTML";

String renderedPage() {
  String page(FPSTR(PAGE));
  page.reserve(page.length() + strlen_P(WIFI_CONFIGURATION_SECTION) + 320);
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
  server.on("/api/scheduler/status", HTTP_GET, [] {
    noteHttpRequest("scheduler_status"); sendJson(schedulerStatusJson());
  });
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
    const bool maintenanceMode = server.hasArg("maintenanceMode");
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
    if (!startSafeSmaAcquisition(slot)) {
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
  server.on("/api/sma/connect-all", HTTP_POST, [] {
    noteHttpRequest("sma_connect_all");
    if (scanState == ScanState::SCANNING) {
      sendJson("{\"error\":\"bt_scan_in_progress\"}", 409); return;
    }
    if (!timeSynchronized()) {
      sendJson("{\"error\":\"time_not_synchronized\"}", 503); return;
    }
    if (!startCombinedSmaAcquisition()) {
      sendJson("{\"error\":\"combined_acquisition_busy_or_invalid\"}", 409); return;
    }
    sendJson("{\"accepted\":true,\"state\":\"QUIESCING_NETWORK\"}", 202);
  });
  server.on("/api/bt/reuse-test", HTTP_POST, [] {
    const uint32_t delayMs = static_cast<uint32_t>(server.arg("delay").toInt());
    if (delayMs != 500 && delayMs != 1000 && delayMs != 2000 &&
        delayMs != 3000 && delayMs != 5000) {
      sendJson("{\"error\":\"invalid_delay\"}", 400); return;
    }
    if (!ProductConfig::validMac(productSettings.inverters[0].mac) ||
        !ProductConfig::validMac(productSettings.inverters[1].mac)) {
      sendJson("{\"error\":\"invalid_target\"}", 409); return;
    }
    rfcommDelayResult = {};
    rfcommDelayResult.requestedDelayMs = delayMs;
    btReuseDiagnostic = true;
    if (!startNetworkAudit(true, true, true)) {
      btReuseDiagnostic = false;
      sendJson("{\"error\":\"diagnostic_busy\"}", 409); return;
    }
    String response(F("{\"accepted\":true,\"delayMs\":"));
    response += delayMs; response += '}'; sendJson(response, 202);
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
  serviceBtOnlyTest();
  serviceNetworkAudit();
  serviceSmaLifecycle();
  serviceScheduler();
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
  updateSoakMemory();
  delay(2);
}
