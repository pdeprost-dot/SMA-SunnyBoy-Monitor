#include <cstdio>
#include <cstring>
#include <ctime>

#include "../firmware/SmaBluetoothDiscovery/ProductConfiguration.h"
#include "../firmware/SmaBluetoothDiscovery/SbfspotCompat.h"

namespace { int failures; void check(bool ok, const char* text) { std::printf("%s %s\n", ok ? "PASS" : "FAIL", text); if (!ok) ++failures; } }

int main() {
  char topic[64];
  check(ProductConfig::validTopicPrefix("smaesp") && !ProductConfig::validTopicPrefix("") &&
        !ProductConfig::validTopicPrefix("sma/esp") && !ProductConfig::validTopicPrefix("sma+"), "topic prefix validation");
  check(ProductConfig::makeTopic("smaesp", 0, topic, sizeof(topic)) && !std::strcmp(topic, "smaesp/inv1") &&
        ProductConfig::makeTopic("smaesp", 1, topic, sizeof(topic)) && !std::strcmp(topic, "smaesp/inv2") &&
        ProductConfig::makeTopic("smaesp", 2, topic, sizeof(topic)) && !std::strcmp(topic, "smaesp/inv3"), "three slot topics");
  check(ProductConfig::validSerial(1000000001U) && !ProductConfig::validSerial(0), "serial validation");
  check(ProductConfig::validMac("02:00:00:00:00:01") && !ProductConfig::validMac("02:00:00:00:00"), "Bluetooth MAC validation");
  check(ProductConfig::validWpaPassword("12345678") && !ProductConfig::validWpaPassword("short"), "AP password validation");
  check(ProductConfig::validOtaPassword("12345678") && !ProductConfig::validOtaPassword(""), "OTA password validation");
  check(ProductConfig::validLatitude(-90) && ProductConfig::validLatitude(90) && !ProductConfig::validLatitude(90.1) &&
        ProductConfig::validLongitude(-180) && ProductConfig::validLongitude(180) && !ProductConfig::validLongitude(180.1), "location limits");
  time_t sample = 1704067200; char formatted[24]{};
  check(ProductConfig::formatLocalTime(sample, formatted, sizeof(formatted)) && std::strlen(formatted) == 19 && formatted[2] == '/' && formatted[5] == '/', "local timestamp format");
  time_t sunrise = 0, sunset = 0;
  check(ProductConfig::calculateSunTimes(sample, 50.0, 4.0, sunrise, sunset) && sunrise < sunset && sunset - sunrise < 86400, "deterministic sunrise sunset");
  check(!std::strcmp(ProductConfig::webValueAvailable(false), "--") && std::strcmp(ProductConfig::webValueAvailable(false), "0"), "unavailable web representation");
  SbfspotCompat::InverterSnapshot snapshots[3]{}; snapshots[0].inverterSerial = 1; snapshots[1].inverterSerial = 2; snapshots[2].inverterSerial = 3;
  snapshots[1].acTotalPower.state = SbfspotCompat::ValueState::Valid; snapshots[1].acTotalPower.value = 0;
  check(snapshots[0].inverterSerial != snapshots[1].inverterSerial && snapshots[1].acTotalPower.state == SbfspotCompat::ValueState::Valid && snapshots[2].acTotalPower.state == SbfspotCompat::ValueState::NotImplemented, "three independent snapshots and measured zero");
  return failures ? 1 : 0;
}
