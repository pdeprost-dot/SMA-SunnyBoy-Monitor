#pragma once

#include <stddef.h>
#include <stdint.h>

namespace SbfspotCompat {

enum class ValueState : uint8_t { NotImplemented, Unavailable, Valid };

template <typename T>
struct Value {
  ValueState state = ValueState::NotImplemented;
  T value{};
};

struct InverterSnapshot {
  bool acquisitionValid = false;
  bool acquisitionComplete = false;
  uint32_t lastSuccessfulAcquisition = 0;
  char schemaVersion[8] = "2";
  char firmwareVersion[40] = "SMA-SunnyBoy-Monitor 3.0";
  char plantName[40]{};
  char timestamp[24]{};
  char inverterTime[24]{};
  char sunrise[24]{};
  char sunset[24]{};
  uint32_t inverterSerial = 0;
  char inverterName[40]{};
  ValueState inverterNameState = ValueState::NotImplemented;
  char inverterClass[32] = "Solar Inverters";
  ValueState inverterClassState = ValueState::NotImplemented;
  char inverterType[40]{};
  ValueState inverterTypeState = ValueState::NotImplemented;
  char inverterSoftwareVersion[24]{};
  ValueState inverterSoftwareVersionState = ValueState::NotImplemented;
  char inverterStatus[40] = "Information not available";
  ValueState inverterStatusState = ValueState::NotImplemented;
  Value<int32_t> inverterTemperature;
  char inverterGridRelay[40] = "Information not available";
  ValueState inverterGridRelayState = ValueState::NotImplemented;
  Value<uint64_t> totalEnergy;
  Value<uint64_t> todayEnergy;
  Value<uint32_t> acTotalPower;
  Value<uint32_t> dcPower1;
  Value<uint32_t> dcVoltage1;
  Value<uint32_t> dcCurrent1;
  Value<uint32_t> dcTotalPower;
  Value<uint64_t> operatingTime;
  Value<uint64_t> feedInTime;
  Value<uint32_t> acPower1;
  Value<uint32_t> acVoltage1;
  Value<uint32_t> acCurrent1;
  Value<uint32_t> gridFrequency;
  Value<uint32_t> bluetoothSignal;
  char inverterWakeupTime[24]{};
  char inverterSleepTime[24]{};
};

struct MqttConfig {
  bool enabled = false;
  char broker[64] = "192.168.50.200";
  uint16_t port = 1883;
  char username[40]{};
  char password[64]{};
  char topicPrefix[48] = "smaesp";
  uint32_t publishIntervalSeconds = 300;
};

bool validateMqttConfig(const MqttConfig& config);
bool serialize(const InverterSnapshot& snapshot, char* output, size_t capacity, size_t& written);

}  // namespace SbfspotCompat
