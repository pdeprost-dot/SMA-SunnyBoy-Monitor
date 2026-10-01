#include "SbfspotCompat.h"
#include "ProductConfiguration.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace SbfspotCompat {
namespace {
bool append(char* output, size_t capacity, size_t& used, const char* format, ...) {
  if (!output || used >= capacity) return false;
  va_list args;
  va_start(args, format);
  const int count = vsnprintf(output + used, capacity - used, format, args);
  va_end(args);
  if (count < 0 || static_cast<size_t>(count) >= capacity - used) return false;
  used += static_cast<size_t>(count);
  return true;
}

bool appendEscaped(char* output, size_t capacity, size_t& used, const char* value) {
  if (!append(output, capacity, used, "\"")) return false;
  for (const unsigned char* p = reinterpret_cast<const unsigned char*>(value ? value : ""); *p; ++p) {
    if (*p == '"' || *p == '\\') {
      if (!append(output, capacity, used, "\\%c", *p)) return false;
    } else if (*p == '\n') {
      if (!append(output, capacity, used, "\\n")) return false;
    } else if (*p == '\r') {
      if (!append(output, capacity, used, "\\r")) return false;
    } else if (*p == '\t') {
      if (!append(output, capacity, used, "\\t")) return false;
    } else if (*p >= 0x20) {
      if (!append(output, capacity, used, "%c", *p)) return false;
    }
  }
  return append(output, capacity, used, "\"");
}

template <typename T>
unsigned long long compatibilityNumber(const Value<T>& value) {
  return value.state == ValueState::Valid ? static_cast<unsigned long long>(value.value) : 0ULL;
}

bool bounded(const char* value, size_t capacity) {
  return value && memchr(value, 0, capacity) != nullptr;
}
}  // namespace

bool validateMqttConfig(const MqttConfig& config) {
  if (!bounded(config.broker, sizeof(config.broker)) || !bounded(config.username, sizeof(config.username)) ||
      !bounded(config.password, sizeof(config.password)) || !bounded(config.topicPrefix, sizeof(config.topicPrefix))) return false;
  if (config.enabled && (config.broker[0] == 0 || config.topicPrefix[0] == 0)) return false;
  if (!ProductConfig::validTopicPrefix(config.topicPrefix)) return false;
  if (config.port == 0 || config.publishIntervalSeconds < 10 || config.publishIntervalSeconds > 86400) return false;
  return true;
}

bool serialize(const InverterSnapshot& s, char* output, size_t capacity, size_t& written) {
  written = 0;
  if (!output || capacity == 0) return false;
  output[0] = 0;
#define TEXT_FIELD(name, value, comma) \
  do { if (!append(output, capacity, written, comma "\"" name "\":")) return false; \
       if (!appendEscaped(output, capacity, written, value)) return false; } while (0)
#define NUM_FIELD(name, value) \
  do { if (!append(output, capacity, written, ",\"" name "\":%llu", compatibilityNumber(value))) return false; } while (0)
  if (!append(output, capacity, written, "{")) return false;
  TEXT_FIELD("PrgVersion", s.programVersion, "");
  TEXT_FIELD("Plantname", s.plantName, ",");
  TEXT_FIELD("Timestamp", s.timestamp, ",");
  TEXT_FIELD("InvTime", s.inverterTime, ",");
  TEXT_FIELD("SunRise", s.sunrise, ",");
  TEXT_FIELD("SunSet", s.sunset, ",");
  if (!append(output, capacity, written, ",\"InvSerial\":%lu", static_cast<unsigned long>(s.inverterSerial))) return false;
  TEXT_FIELD("InvName", s.inverterName, ",");
  TEXT_FIELD("InvClass", s.inverterClass, ",");
  TEXT_FIELD("InvType", s.inverterType, ",");
  TEXT_FIELD("InvSwVer", s.inverterSoftwareVersion, ",");
  TEXT_FIELD("InvStatus", s.inverterStatus, ",");
  NUM_FIELD("InvTemperature", s.inverterTemperature);
  TEXT_FIELD("InvGridRelay", s.inverterGridRelay, ",");
  NUM_FIELD("ETotal", s.totalEnergy);
  NUM_FIELD("EToday", s.todayEnergy);
  NUM_FIELD("PACTot", s.acTotalPower);
  NUM_FIELD("PDC1", s.dcPower1);
  NUM_FIELD("UDC1", s.dcVoltage1);
  NUM_FIELD("IDC1", s.dcCurrent1);
  NUM_FIELD("PDCTot", s.dcTotalPower);
  NUM_FIELD("OperTm", s.operatingTime);
  NUM_FIELD("FeedTm", s.feedInTime);
  NUM_FIELD("PAC1", s.acPower1);
  NUM_FIELD("UAC1", s.acVoltage1);
  NUM_FIELD("IAC1", s.acCurrent1);
  NUM_FIELD("GridFreq", s.gridFrequency);
  NUM_FIELD("BTSignal", s.bluetoothSignal);
  TEXT_FIELD("InvWakeupTm", s.inverterWakeupTime, ",");
  TEXT_FIELD("InvSleepTm", s.inverterSleepTime, ",");
  if (!append(output, capacity, written, "}")) return false;
#undef TEXT_FIELD
#undef NUM_FIELD
  return true;
}

}  // namespace SbfspotCompat
