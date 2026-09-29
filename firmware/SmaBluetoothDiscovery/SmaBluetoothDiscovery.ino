#include <Arduino.h>
#include <BluetoothSerial.h>
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
constexpr uint32_t BETWEEN_SCANS_MS = 5000;
constexpr char LOCAL_BT_NAME[] = "SMA-SunnyBoy-Monitor";

struct KnownSma {
  const char* mac;
  const char* label;
  uint32_t serial;
  uint32_t sightings;
};

KnownSma knownSmas[] = {
  {"02:00:00:00:00:01", "SMA #1", 1000000001UL, 0},
  {"02:00:00:00:00:02", "SMA #2", 1000000002UL, 0},
  {"02:00:00:00:00:03", "SMA #3", 1000000003UL, 0},
};

BluetoothSerial serialBt;
uint32_t scanNumber = 0;

void printHeap(const char* stage) {
  Serial.printf(
    "[HEAP] stage=%s free=%u min=%u largest_internal=%u\n",
    stage,
    ESP.getFreeHeap(),
    ESP.getMinFreeHeap(),
    heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
  );
}

KnownSma* findKnownSma(const String& address) {
  for (KnownSma& sma : knownSmas) {
    if (address.equalsIgnoreCase(sma.mac)) return &sma;
  }
  return nullptr;
}

void printDevice(BTAdvertisedDevice* device) {
  if (device == nullptr) {
    Serial.println(F("[BT] ERROR null discovery result"));
    return;
  }

  const String address(device->getAddress().toString().c_str());
  Serial.printf("[BT] MAC=%s", address.c_str());

  if (device->haveRSSI()) Serial.printf(" RSSI=%d", device->getRSSI());
  else Serial.print(F(" RSSI=n/a"));

  if (device->haveName()) {
    const std::string name = device->getName();
    Serial.printf(" NAME=\"%s\"", name.c_str());
  } else {
    Serial.print(F(" NAME=n/a"));
  }

  if (device->haveCOD()) Serial.printf(" COD=0x%06lX", static_cast<unsigned long>(device->getCOD()));
  else Serial.print(F(" COD=n/a"));

  Serial.println();

  KnownSma* sma = findKnownSma(address);
  if (sma != nullptr) {
    ++sma->sightings;
    Serial.printf(
      "[BT] MATCH %s SN=%lu sightings=%lu\n",
      sma->label,
      static_cast<unsigned long>(sma->serial),
      static_cast<unsigned long>(sma->sightings)
    );
  }
}

void printSummary() {
  Serial.printf("[SCAN] SUMMARY cycle=%lu", static_cast<unsigned long>(scanNumber));
  for (const KnownSma& sma : knownSmas) {
    Serial.printf(" %s=%lu", sma.label, static_cast<unsigned long>(sma.sightings));
  }
  Serial.println();
}

void runInquiry() {
  ++scanNumber;
  Serial.printf(
    "[SCAN] START cycle=%lu duration_ms=%lu\n",
    static_cast<unsigned long>(scanNumber),
    static_cast<unsigned long>(INQUIRY_DURATION_MS)
  );
  printHeap("before_inquiry");

  BTScanResults* results = serialBt.discover(INQUIRY_DURATION_MS);
  if (results == nullptr) {
    Serial.println(F("[SCAN] ERROR inquiry_failed"));
  } else {
    const int count = results->getCount();
    Serial.printf("[SCAN] COMPLETE cycle=%lu devices=%d\n", static_cast<unsigned long>(scanNumber), count);
    for (int index = 0; index < count; ++index) printDevice(results->getDevice(index));
    serialBt.discoverClear();
  }

  printSummary();
  printHeap("after_inquiry");
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println(F("=== SMA SunnyBoy Monitor — Phase 1 ==="));
  Serial.printf("[SYSTEM] chip=%s revision=%u cores=%u cpu_mhz=%u\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(), ESP.getCpuFreqMHz());
  Serial.printf("[SYSTEM] flash=%u sdk=%s reset_reason=%d\n",
                ESP.getFlashChipSize(), ESP.getSdkVersion(), static_cast<int>(esp_reset_reason()));
  printHeap("boot");

  if (!serialBt.begin(LOCAL_BT_NAME, true)) {
    Serial.println(F("[BT] FATAL Bluetooth Classic initialization failed"));
    return;
  }

  Serial.printf("[BT] READY name=%s mode=master inquiry=classic\n", LOCAL_BT_NAME);
  printHeap("bt_ready");
}

void loop() {
  static uint32_t nextScanAt = 0;
  if (static_cast<int32_t>(millis() - nextScanAt) < 0) {
    delay(10);
    return;
  }

  runInquiry();
  nextScanAt = millis() + BETWEEN_SCANS_MS;
}
