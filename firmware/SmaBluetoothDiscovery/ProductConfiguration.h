#pragma once

#include <stddef.h>
#include <stdint.h>
#include <time.h>

namespace ProductConfig {

constexpr size_t kInverterCount = 3;
constexpr size_t kTopicPrefixCapacity = 48;
constexpr size_t kMacCapacity = 18;
constexpr size_t kNameCapacity = 24;

struct InverterSlot {
  bool enabled = false;
  char mac[kMacCapacity]{};
  uint32_t serial = 0;
  char name[kNameCapacity]{};
  char userPassword[13]{};
};

struct Settings {
  bool maintenanceMode = false;
  char plantName[40]{};
  char apPassword[64]{};
  char apSsid[33]{};
  char otaPassword[64]{};
  double latitude = 0;
  double longitude = 0;
  char timezone[64] = "CET-1CEST,M3.5.0,M10.5.0/3";
  char timezoneName[32] = "Europe/Brussels";
  InverterSlot inverters[kInverterCount];
};

bool validTopicPrefix(const char* value);
bool makeTopic(const char* prefix, size_t slot, char* output, size_t capacity);
bool validSerial(uint32_t value);
bool validMac(const char* value);
bool validWpaPassword(const char* value);
bool validOtaPassword(const char* value);
bool validAdminPassword(const char* value);
bool resolveAdminPassword(bool storedKeyPresent, const char* storedValue,
                          char* output, size_t capacity);
bool validLatitude(double value);
bool validLongitude(double value);
bool formatLocalTime(time_t value, char* output, size_t capacity);
bool calculateSunTimes(time_t now, double latitude, double longitude,
                       time_t& sunrise, time_t& sunset);
const char* webValueAvailable(bool available);

}  // namespace ProductConfig
