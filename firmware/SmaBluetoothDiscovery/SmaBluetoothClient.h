#pragma once

#include <Arduino.h>
#include <BluetoothSerial.h>

class SmaBluetoothClient {
 public:
  enum class State : uint8_t {
    DISCONNECTED,
    CONNECTING,
    WAIT_ANNOUNCE,
    WAIT_LOCAL_ADDRESS,
    WAIT_IDENTITY,
    SESSION_READY,
    DISCONNECTING,
    ERROR
  };

  using LogFn = void (*)(const char* line);

  SmaBluetoothClient(BluetoothSerial& transport, LogFn logger);
  bool requestConnect();
  bool requestDisconnect();
  void tick();

  State state() const { return state_; }
  const char* stateName() const;
  bool bluetoothConnected() const;
  bool sessionReady() const { return state_ == State::SESSION_READY; }
  const char* lastError() const { return lastError_; }
  uint32_t txBytes() const { return txBytes_; }
  uint32_t rxBytes() const { return rxBytes_; }
  uint32_t validResponses() const { return validResponses_; }
  uint32_t lastValidResponseAt() const { return lastValidResponseAt_; }
  uint32_t decodedSerial() const { return decodedSerial_; }
  uint8_t netId() const { return netId_; }

  static constexpr const char* TARGET_MAC = "02:00:00:00:00:01";
  static constexpr uint32_t TARGET_SERIAL = 1000000001UL;

 private:
  static constexpr size_t FRAME_CAPACITY = 384;
  static constexpr uint32_t STEP_TIMEOUT_MS = 15000;

  BluetoothSerial& transport_;
  LogFn logger_;
  State state_ = State::DISCONNECTED;
  volatile bool connectTaskDone_ = false;
  volatile bool connectTaskResult_ = false;
  TaskHandle_t connectTaskHandle_ = nullptr;
  uint8_t targetConnectAddress_[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
  uint8_t targetProtocolAddress_[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x02};
  uint8_t localProtocolAddress_[6] = {};
  uint8_t rxFrame_[FRAME_CAPACITY] = {};
  uint8_t l2Frame_[FRAME_CAPACITY] = {};
  uint8_t txFrame_[FRAME_CAPACITY] = {};
  size_t rxPosition_ = 0;
  size_t rxExpected_ = 0;
  uint16_t packetId_ = 1;
  uint8_t netId_ = 0;
  uint32_t decodedSerial_ = 0;
  uint32_t deadlineAt_ = 0;
  uint32_t txBytes_ = 0;
  uint32_t rxBytes_ = 0;
  uint32_t validResponses_ = 0;
  uint32_t lastValidResponseAt_ = 0;
  char lastError_[64] = {};

  static void connectTaskEntry(void* context);
  void runConnectTask();
  void setState(State next);
  void fail(const char* error);
  void emit(const char* format, ...) const;
  void logMemory(const char* stage) const;
  void resetParser();
  void receiveBytes();
  void processFrame();
  bool validTargetSource() const;
  bool sendLevel1Configuration();
  bool sendIdentityRequest();
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
