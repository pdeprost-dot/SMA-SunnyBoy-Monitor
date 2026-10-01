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
#define NUM_FIELD(name, field) \
  do { if ((field).state == ValueState::Valid) { \
         if (!append(output, capacity, written, ",\"" name "\":%llu", \
                     static_cast<unsigned long long>((field).value))) return false; \
       } else if (!append(output, capacity, written, ",\"" name "\":null")) return false; } while (0)
#define SCALED_FIELD(name, field, divisor, decimals) \
  do { if ((field).state == ValueState::Valid) { \
         if (!append(output, capacity, written, ",\"" name "\":%.*f", decimals, \
                     static_cast<double>((field).value) / divisor)) return false; \
       } else if (!append(output, capacity, written, ",\"" name "\":null")) return false; } while (0)
#define STATE_TEXT_FIELD(name, value, state) \
  do { if (!append(output, capacity, written, ",\"" name "\":")) return false; \
       if ((state) == ValueState::Valid) { if (!appendEscaped(output, capacity, written, value)) return false; } \
       else if (!append(output, capacity, written, "null")) return false; } while (0)
  if (!append(output, capacity, written, "{")) return false;
  TEXT_FIELD("timestamp", s.timestamp, "");
  if (!append(output, capacity, written, ",\"serial\":%lu", static_cast<unsigned long>(s.inverterSerial))) return false;
  STATE_TEXT_FIELD("name", s.inverterName, s.inverterNameState);
  STATE_TEXT_FIELD("class", s.inverterClass, s.inverterClassState);
  STATE_TEXT_FIELD("type", s.inverterType, s.inverterTypeState);
  STATE_TEXT_FIELD("sw_version", s.inverterSoftwareVersion, s.inverterSoftwareVersionState);
  STATE_TEXT_FIELD("status", s.inverterStatus, s.inverterStatusState);
  STATE_TEXT_FIELD("grid_relay", s.inverterGridRelay, s.inverterGridRelayState);
  SCALED_FIELD("temperature_c", s.inverterTemperature, 100.0, 2);
  SCALED_FIELD("energy_total_kwh", s.totalEnergy, 1000.0, 3);
  SCALED_FIELD("energy_today_kwh", s.todayEnergy, 1000.0, 3);
  NUM_FIELD("ac_power_w", s.acTotalPower);
  NUM_FIELD("ac_power_l1_w", s.acPower1);
  SCALED_FIELD("ac_voltage_l1_v", s.acVoltage1, 100.0, 2);
  SCALED_FIELD("ac_current_l1_a", s.acCurrent1, 1000.0, 3);
  SCALED_FIELD("grid_frequency_hz", s.gridFrequency, 100.0, 2);
  NUM_FIELD("dc_power_w", s.dcPower1);
  SCALED_FIELD("dc_voltage_v", s.dcVoltage1, 100.0, 2);
  SCALED_FIELD("dc_current_a", s.dcCurrent1, 1000.0, 3);
  SCALED_FIELD("operating_time_h", s.operatingTime, 3600.0, 3);
  SCALED_FIELD("feed_in_time_h", s.feedInTime, 3600.0, 3);
  SCALED_FIELD("bt_signal_percent", s.bluetoothSignal, 1000.0, 3);
  if (!append(output, capacity, written, ",\"data_valid\":%s}",
              s.acquisitionValid ? "true" : "false")) return false;
#undef TEXT_FIELD
#undef NUM_FIELD
#undef SCALED_FIELD
#undef STATE_TEXT_FIELD
  return true;
}

}  // namespace SbfspotCompat
