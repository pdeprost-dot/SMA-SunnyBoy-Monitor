#include <cstdio>
#include <cstring>

#include "../firmware/SmaBluetoothDiscovery/SbfspotCompat.h"

namespace {
int failures = 0;
void check(bool condition, const char* name) {
  std::printf("%s %s\n", condition ? "PASS" : "FAIL", name);
  if (!condition) ++failures;
}
}

int main() {
  SbfspotCompat::InverterSnapshot snapshot{};
  std::strcpy(snapshot.inverterName, "A\\\"B");
  snapshot.inverterSerial = 123456;
  snapshot.acTotalPower.state = SbfspotCompat::ValueState::Valid;
  snapshot.acTotalPower.value = 2345;
  snapshot.acVoltage1.state = SbfspotCompat::ValueState::Unavailable;
  char payload[1024]; size_t bytes = 0;
  check(SbfspotCompat::serialize(snapshot, payload, sizeof(payload), bytes), "serializer accepts bounded snapshot");
  check(std::strstr(payload, "\"PrgVersion\"") && std::strstr(payload, "\"InvSleepTm\"") &&
        std::strstr(payload, "\"PACTot\":2345"), "field names and numeric representation");
  check(std::strstr(payload, "\"UAC1\":0") && std::strstr(payload, "A\\\\\\\"B"),
        "unavailable value and JSON escaping");
  std::memset(snapshot.inverterName, 'X', sizeof(snapshot.inverterName) - 1);
  snapshot.inverterName[sizeof(snapshot.inverterName) - 1] = 0;
  check(SbfspotCompat::serialize(snapshot, payload, sizeof(payload), bytes), "maximum bounded string");
  char tiny[64]; size_t tinyBytes = 0;
  check(!SbfspotCompat::serialize(snapshot, tiny, sizeof(tiny), tinyBytes), "payload overflow rejected");
  SbfspotCompat::MqttConfig config{};
  check(SbfspotCompat::validateMqttConfig(config), "default MQTT config valid");
  config.port = 0;
  check(!SbfspotCompat::validateMqttConfig(config), "invalid MQTT port rejected");
  config.port = 1883; config.publishIntervalSeconds = 1;
  check(!SbfspotCompat::validateMqttConfig(config), "unsafe publish interval rejected");
  return failures ? 1 : 0;
}
