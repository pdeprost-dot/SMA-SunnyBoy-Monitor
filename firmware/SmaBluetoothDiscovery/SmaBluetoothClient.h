#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>
#include "SmaLocalConfig.h"
#include "SmaPhase3Protocol.h"

class SmaBluetoothClient {
 public:
  static constexpr size_t DATASET_DIAGNOSTIC_COUNT = 15;
  enum class State : uint8_t {
    DISCONNECTED,
    CONNECTING,
    WAIT_ANNOUNCE,
    WAIT_LOCAL_ADDRESS,
    WAIT_IDENTITY,
    SESSION_READY,
    WAIT_LOGIN,
    WAIT_DATASET,
    PHASE3_COMPLETE,
    DISCONNECTING,
    ERROR
  };

  using LogFn = void (*)(const char* line);

  SmaBluetoothClient(BluetoothSerial& transport, LogFn logger);
  bool setTarget(const char* mac, uint32_t serial);
  bool requestConnect();
  bool requestDisconnect();
  void tick();
  void setAppSerial(uint32_t value) { appSerial_ = value; }

  State state() const { return state_; }
  const char* stateName() const;
  bool bluetoothConnected() const;
  bool sessionReady() const { return state_ == State::SESSION_READY; }
  bool startPhase3Transaction();
  bool phase3Complete() const { return state_ == State::PHASE3_COMPLETE; }
  bool loginValid() const { return loginValid_; }
  uint16_t loginStatus() const { return loginStatus_; }
  uint16_t loginPacketId() const { return loginPacketId_; }
  uint32_t loginTimestamp() const { return loginTimestamp_; }
  bool acPowerValid() const { return acPowerValid_; }
  bool acPowerUnavailable() const { return acPowerUnavailable_; }
  const char* acPowerClassification() const;
  uint32_t acPowerW() const { return acPowerW_; }
  uint16_t acPowerPacketId() const { return acPowerPacketId_; }
  uint16_t returnedLri() const { return returnedLri_; }
  uint8_t returnedRecordType() const { return returnedRecordType_; }
  uint16_t returnedRecordSize() const { return returnedRecordSize_; }
  uint32_t measurementTimestamp() const { return measurementTimestamp_; }
  const SmaPhase3::Measurements& measurements() const { return measurements_; }
  uint8_t datasetQueryIndex() const { return datasetQueryIndex_; }
  uint8_t lastDatasetQueryIndex() const { return lastDatasetQueryIndex_; }
  SmaPhase3::DecodeResult lastDatasetDecodeResult() const { return lastDatasetDecodeResult_; }
  uint16_t datasetL1Fragments(size_t index) const {
    return index < DATASET_DIAGNOSTIC_COUNT ? datasetL1Fragments_[index] : 0;
  }
  uint16_t datasetL1Bytes(size_t index) const {
    return index < DATASET_DIAGNOSTIC_COUNT ? datasetL1Bytes_[index] : 0;
  }
  uint16_t datasetL2Packets(size_t index) const {
    return index < DATASET_DIAGNOSTIC_COUNT ? datasetL2Packets_[index] : 0;
  }
  uint16_t datasetL2Bytes(size_t index) const {
    return index < DATASET_DIAGNOSTIC_COUNT ? datasetL2Bytes_[index] : 0;
  }
  SmaPhase3::DecodeResult datasetDecodeResult(size_t index) const {
    return index < DATASET_DIAGNOSTIC_COUNT ? datasetDecodeResults_[index]
                                             : SmaPhase3::DecodeResult::InvalidValue;
  }
  bool datasetUsable() const;
  bool datasetPartial() const { return datasetUnsupportedResponses_ != 0; }
  const char* lastError() const { return lastError_; }
  uint32_t txBytes() const { return txBytes_; }
  uint32_t rxBytes() const { return rxBytes_; }
  uint32_t validResponses() const { return validResponses_; }
  uint32_t lastValidResponseAt() const { return lastValidResponseAt_; }
  uint32_t decodedSerial() const { return decodedSerial_; }
  uint32_t expectedSerial() const { return expectedSerial_; }
  uint8_t netId() const { return netId_; }
  uint32_t appSerial() const { return appSerial_; }
  UBaseType_t connectTaskStackHighWater() const { return connectTaskStackHighWater_; }

 private:
  static constexpr size_t FRAME_CAPACITY = 384;
  static constexpr uint32_t STEP_TIMEOUT_MS = 15000;
  static constexpr uint32_t RECONNECT_GUARD_MS = 3000;

  BluetoothSerial& transport_;
  LogFn logger_;
  State state_ = State::DISCONNECTED;
  volatile bool connectTaskDone_ = false;
  volatile bool connectTaskResult_ = false;
  bool connectTaskStarted_ = false;
  TaskHandle_t connectTaskHandle_ = nullptr;
  uint8_t targetConnectAddress_[6] = {};
  uint8_t targetProtocolAddress_[6] = {};
  uint32_t expectedSerial_ = 0;
  uint8_t localProtocolAddress_[6] = {};
  uint8_t rxFrame_[FRAME_CAPACITY] = {};
  uint8_t l2Frame_[FRAME_CAPACITY] = {};
  uint8_t txFrame_[FRAME_CAPACITY] = {};
  size_t rxPosition_ = 0;
  size_t rxExpected_ = 0;
  size_t l2Position_ = 0;
  bool l2Active_ = false;
  bool l2EscapePending_ = false;
  uint16_t packetId_ = 1;
  uint32_t appSerial_ = 0;
  volatile UBaseType_t connectTaskStackHighWater_ = 0;
  uint8_t netId_ = 0;
  uint32_t decodedSerial_ = 0;
  uint32_t deadlineAt_ = 0;
  uint32_t reconnectAllowedAt_ = 0;
  uint32_t txBytes_ = 0;
  uint32_t rxBytes_ = 0;
  uint32_t validResponses_ = 0;
  uint32_t lastValidResponseAt_ = 0;
  bool loginValid_ = false;
  uint16_t loginStatus_ = 0xFFFF;
  uint16_t loginPacketId_ = 0;
  uint32_t loginTimestamp_ = 0;
  bool acPowerValid_ = false;
  bool acPowerUnavailable_ = false;
  uint32_t acPowerW_ = 0;
  uint16_t acPowerPacketId_ = 0;
  uint16_t returnedLri_ = 0;
  uint8_t returnedRecordType_ = 0;
  uint16_t returnedRecordSize_ = 0;
  uint32_t measurementTimestamp_ = 0;
  SmaPhase3::Measurements measurements_{};
  uint8_t datasetQueryIndex_ = 0;
  uint16_t datasetPacketId_ = 0;
  uint8_t datasetUnsupportedResponses_ = 0;
  uint8_t lastDatasetQueryIndex_ = 0;
  SmaPhase3::DecodeResult lastDatasetDecodeResult_ = SmaPhase3::DecodeResult::Ok;
  uint16_t datasetL1Fragments_[DATASET_DIAGNOSTIC_COUNT]{};
  uint16_t datasetL1Bytes_[DATASET_DIAGNOSTIC_COUNT]{};
  uint16_t datasetL2Packets_[DATASET_DIAGNOSTIC_COUNT]{};
  uint16_t datasetL2Bytes_[DATASET_DIAGNOSTIC_COUNT]{};
  SmaPhase3::DecodeResult datasetDecodeResults_[DATASET_DIAGNOSTIC_COUNT]{};
  uint16_t inverterSusyId_ = 0;
  uint32_t inverterSerial_ = 0;
  char lastError_[64] = {};

  static void connectTaskEntry(void* context);
  bool startConnectTask();
  void runConnectTask();
  void setState(State next);
  void fail(const char* error);
  void emit(const char* format, ...) const;
  void logMemory(const char* stage) const;
  void resetParser();
  void resetL2Parser();
  bool appendL2Payload(const uint8_t* data, size_t length);
  void receiveBytes();
  void processFrame();
  bool validTargetSource() const;
  bool sendLevel1Configuration();
  bool sendIdentityRequest();
  bool sendLoginRequest();
  bool sendNextDatasetRequest(uint16_t inverterSusyId, uint32_t inverterSerial);
  bool processData2Response(const uint8_t* l2, size_t length);
  bool sendRaw(const uint8_t* data, size_t length, const char* label);
  size_t buildL2Identity(uint8_t* output, size_t capacity);
  size_t wrapL2(const uint8_t* l2, size_t l2Length, uint8_t* output, size_t capacity);
  bool decodeIdentity(const uint8_t* l2, size_t length);
  static uint16_t read16(const uint8_t* data);
  static uint32_t read32(const uint8_t* data);
  static void write16(uint8_t* data, size_t& position, uint16_t value);
  static void write32(uint8_t* data, size_t& position, uint32_t value);
  static uint16_t fcs16(const uint8_t* data, size_t length);
};
