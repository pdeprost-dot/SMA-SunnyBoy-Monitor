#include "SmaBluetoothClient.h"

#include <esp_heap_caps.h>
#include <stdarg.h>

namespace {
constexpr uint8_t L1_START = 0x7E;
constexpr size_t L1_HEADER_SIZE = 18;
constexpr uint32_t L2_SIGNATURE = 0x656003FFUL;
constexpr uint16_t APP_SUSY_ID = 125;
constexpr uint32_t APP_SERIAL = 0;
constexpr uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
}

SmaBluetoothClient::SmaBluetoothClient(BluetoothSerial& transport, LogFn logger)
    : transport_(transport), logger_(logger) {}

const char* SmaBluetoothClient::stateName() const {
  switch (state_) {
    case State::DISCONNECTED: return "DISCONNECTED";
    case State::CONNECTING: return "CONNECTING";
    case State::WAIT_ANNOUNCE: return "WAIT_ANNOUNCE";
    case State::WAIT_LOCAL_ADDRESS: return "WAIT_LOCAL_ADDRESS";
    case State::WAIT_IDENTITY: return "WAIT_IDENTITY";
    case State::SESSION_READY: return "SESSION_READY";
    case State::DISCONNECTING: return "DISCONNECTING";
    case State::ERROR: return "ERROR";
  }
  return "ERROR";
}

bool SmaBluetoothClient::bluetoothConnected() const {
  return transport_.connected(0);
}

void SmaBluetoothClient::emit(const char* format, ...) const {
  if (logger_ == nullptr) return;
  char line[128];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  logger_(line);
}

void SmaBluetoothClient::logMemory(const char* stage) const {
  emit("[SMA-MEM] %s free=%u min=%u largest=%u", stage, ESP.getFreeHeap(), ESP.getMinFreeHeap(),
       heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

void SmaBluetoothClient::setState(State next) {
  if (state_ == next) return;
  emit("[SMA] state %s -> %s", stateName(), [next]() {
    switch (next) {
      case State::DISCONNECTED: return "DISCONNECTED";
      case State::CONNECTING: return "CONNECTING";
      case State::WAIT_ANNOUNCE: return "WAIT_ANNOUNCE";
      case State::WAIT_LOCAL_ADDRESS: return "WAIT_LOCAL_ADDRESS";
      case State::WAIT_IDENTITY: return "WAIT_IDENTITY";
      case State::SESSION_READY: return "SESSION_READY";
      case State::DISCONNECTING: return "DISCONNECTING";
      case State::ERROR: return "ERROR";
    }
    return "ERROR";
  }());
  state_ = next;
}

void SmaBluetoothClient::fail(const char* error) {
  strlcpy(lastError_, error, sizeof(lastError_));
  emit("[SMA] error %s", lastError_);
  logMemory("error");
  setState(State::ERROR);
}

bool SmaBluetoothClient::requestConnect() {
  if (state_ != State::DISCONNECTED && state_ != State::ERROR) return false;
  if (transport_.connected(0)) transport_.disconnect();
  lastError_[0] = 0;
  decodedSerial_ = 0;
  validResponses_ = 0;
  lastValidResponseAt_ = 0;
  txBytes_ = 0;
  rxBytes_ = 0;
  packetId_ = 1;
  netId_ = 0;
  memset(localProtocolAddress_, 0, sizeof(localProtocolAddress_));
  resetParser();
  connectTaskDone_ = false;
  connectTaskResult_ = false;
  logMemory("before_connect");
  setState(State::CONNECTING);
  if (xTaskCreatePinnedToCore(connectTaskEntry, "sma-connect", 4096, this, 1, &connectTaskHandle_, 0) != pdPASS) {
    connectTaskHandle_ = nullptr;
    fail("connect_task_create_failed");
    return false;
  }
  return true;
}

void SmaBluetoothClient::connectTaskEntry(void* context) {
  static_cast<SmaBluetoothClient*>(context)->runConnectTask();
}

void SmaBluetoothClient::runConnectTask() {
  connectTaskResult_ = transport_.connect(targetConnectAddress_);
  connectTaskDone_ = true;
  connectTaskHandle_ = nullptr;
  vTaskDelete(nullptr);
}

bool SmaBluetoothClient::requestDisconnect() {
  if (state_ == State::DISCONNECTED || state_ == State::CONNECTING) return false;
  setState(State::DISCONNECTING);
  const bool result = transport_.disconnect();
  resetParser();
  setState(State::DISCONNECTED);
  logMemory("after_disconnect");
  return result;
}

void SmaBluetoothClient::tick() {
  if (state_ == State::CONNECTING && connectTaskDone_) {
    connectTaskDone_ = false;
    if (!connectTaskResult_) {
      fail("bt_connect_failed");
      return;
    }
    emit("[SMA] connect target=%s result=connected", TARGET_MAC);
    logMemory("after_bt_connect");
    deadlineAt_ = millis() + STEP_TIMEOUT_MS;
    setState(State::WAIT_ANNOUNCE);
  }

  if (state_ == State::WAIT_ANNOUNCE || state_ == State::WAIT_LOCAL_ADDRESS ||
      state_ == State::WAIT_IDENTITY || state_ == State::SESSION_READY) {
    if (!transport_.connected(0)) {
      fail("bt_disconnected");
      return;
    }
    receiveBytes();
  }

  if ((state_ == State::WAIT_ANNOUNCE || state_ == State::WAIT_LOCAL_ADDRESS ||
       state_ == State::WAIT_IDENTITY) && static_cast<int32_t>(millis() - deadlineAt_) >= 0) {
    fail("protocol_timeout");
  }
}

void SmaBluetoothClient::resetParser() {
  rxPosition_ = 0;
  rxExpected_ = 0;
}

void SmaBluetoothClient::receiveBytes() {
  size_t budget = 192;
  while (budget-- > 0 && transport_.available()) {
    const int value = transport_.read();
    if (value < 0) break;
    ++rxBytes_;
    const uint8_t byteValue = static_cast<uint8_t>(value);
    if (rxPosition_ == 0 && byteValue != L1_START) continue;
    if (rxPosition_ >= sizeof(rxFrame_)) {
      fail("rx_frame_overflow");
      resetParser();
      return;
    }
    rxFrame_[rxPosition_++] = byteValue;
    if (rxPosition_ == 4) {
      rxExpected_ = read16(rxFrame_ + 1);
      if (rxExpected_ < L1_HEADER_SIZE || rxExpected_ > sizeof(rxFrame_)) {
        fail("invalid_l1_length");
        resetParser();
        return;
      }
    }
    if (rxExpected_ != 0 && rxPosition_ == rxExpected_) {
      processFrame();
      resetParser();
      if (state_ == State::ERROR) return;
    }
  }
}

bool SmaBluetoothClient::validTargetSource() const {
  return memcmp(rxFrame_ + 4, targetProtocolAddress_, 6) == 0;
}

void SmaBluetoothClient::processFrame() {
  const uint16_t command = read16(rxFrame_ + 16);
  emit("[SMA] RX len=%u cmd=0x%04X state=%s", static_cast<unsigned>(rxPosition_), command, stateName());
  if ((rxFrame_[0] ^ rxFrame_[1] ^ rxFrame_[2]) != rxFrame_[3]) {
    fail("invalid_l1_checksum");
    return;
  }
  if (!validTargetSource()) {
    emit("[SMA] RX ignored unexpected_source cmd=0x%04X", command);
    return;
  }

  if (state_ == State::WAIT_ANNOUNCE && command == 0x0002) {
    if (rxPosition_ <= 22) { fail("announce_too_short"); return; }
    netId_ = rxFrame_[22];
    emit("[SMA] announce net_id=%u", netId_);
    if (!sendLevel1Configuration()) { fail("config_tx_failed"); return; }
    deadlineAt_ = millis() + STEP_TIMEOUT_MS;
    setState(State::WAIT_LOCAL_ADDRESS);
    return;
  }

  if (state_ == State::WAIT_LOCAL_ADDRESS && command == 0x0005) {
    if (rxPosition_ < 32) { fail("local_address_frame_too_short"); return; }
    memcpy(localProtocolAddress_, rxFrame_ + 26, 6);
    emit("[SMA] local_bt=%02X:%02X:%02X:%02X:%02X:%02X",
         localProtocolAddress_[5], localProtocolAddress_[4], localProtocolAddress_[3],
         localProtocolAddress_[2], localProtocolAddress_[1], localProtocolAddress_[0]);
    if (!sendIdentityRequest()) { fail("identity_tx_failed"); return; }
    deadlineAt_ = millis() + STEP_TIMEOUT_MS;
    setState(State::WAIT_IDENTITY);
    return;
  }

  if (state_ == State::WAIT_IDENTITY && command == 0x0001) {
    size_t l2Length = 0;
    bool escaped = false;
    for (size_t index = L1_HEADER_SIZE; index < rxPosition_; ++index) {
      uint8_t value = rxFrame_[index];
      if (escaped) { value ^= 0x20; escaped = false; }
      else if (value == 0x7D) { escaped = true; continue; }
      if (l2Length >= sizeof(l2Frame_)) { fail("l2_overflow"); return; }
      l2Frame_[l2Length++] = value;
    }
    if (!decodeIdentity(l2Frame_, l2Length)) return;
    ++validResponses_;
    lastValidResponseAt_ = millis();
    logMemory("after_decode");
    setState(State::SESSION_READY);
  }
}

bool SmaBluetoothClient::sendLevel1Configuration() {
  size_t position = 0;
  txFrame_[position++] = L1_START;
  txFrame_[position++] = 0;
  txFrame_[position++] = 0;
  txFrame_[position++] = 0;
  memcpy(txFrame_ + position, localProtocolAddress_, 6); position += 6;
  memcpy(txFrame_ + position, targetProtocolAddress_, 6); position += 6;
  write16(txFrame_, position, 0x0002);
  write32(txFrame_, position, 0x00700400UL);
  txFrame_[position++] = netId_;
  write32(txFrame_, position, 0);
  write32(txFrame_, position, 1);
  txFrame_[1] = position & 0xFF;
  txFrame_[2] = (position >> 8) & 0xFF;
  txFrame_[3] = txFrame_[0] ^ txFrame_[1] ^ txFrame_[2];
  logMemory("before_config_tx");
  return sendRaw(txFrame_, position, "l1_config");
}

size_t SmaBluetoothClient::buildL2Identity(uint8_t* output, size_t capacity) {
  if (capacity < 48) return 0;
  size_t position = 0;
  output[position++] = 0x7E;
  write32(output, position, L2_SIGNATURE);
  output[position++] = 0x09;
  output[position++] = 0xA0;
  write16(output, position, 0xFFFF);
  write32(output, position, 0xFFFFFFFFUL);
  write16(output, position, 0);
  write16(output, position, APP_SUSY_ID);
  write32(output, position, APP_SERIAL);
  write16(output, position, 0);
  write16(output, position, 0);
  write16(output, position, 0);
  write16(output, position, (++packetId_) | 0x8000);
  write32(output, position, 0x00000200UL);
  write32(output, position, 0);
  write32(output, position, 0);
  const uint16_t checksum = fcs16(output + 1, position - 1);
  write16(output, position, checksum);
  output[position++] = 0x7E;
  return position;
}

size_t SmaBluetoothClient::wrapL2(const uint8_t* l2, size_t l2Length, uint8_t* output, size_t capacity) {
  if (l2Length < 2 || capacity < L1_HEADER_SIZE + l2Length) return 0;
  size_t position = 0;
  output[position++] = L1_START;
  output[position++] = 0;
  output[position++] = 0;
  output[position++] = 0;
  memcpy(output + position, localProtocolAddress_, 6); position += 6;
  memcpy(output + position, BROADCAST, 6); position += 6;
  write16(output, position, 0x0001);
  output[position++] = l2[0];
  for (size_t index = 1; index + 1 < l2Length; ++index) {
    const uint8_t value = l2[index];
    if (value == 0x7D || value == 0x7E || value == 0x11 || value == 0x12 || value == 0x13) {
      if (position + 2 >= capacity) return 0;
      output[position++] = 0x7D;
      output[position++] = value ^ 0x20;
    } else {
      if (position + 1 >= capacity) return 0;
      output[position++] = value;
    }
  }
  output[position++] = l2[l2Length - 1];
  output[1] = position & 0xFF;
  output[2] = (position >> 8) & 0xFF;
  output[3] = output[0] ^ output[1] ^ output[2];
  return position;
}

bool SmaBluetoothClient::sendIdentityRequest() {
  uint8_t l2[64];
  const size_t l2Length = buildL2Identity(l2, sizeof(l2));
  const size_t frameLength = wrapL2(l2, l2Length, txFrame_, sizeof(txFrame_));
  if (frameLength == 0) return false;
  logMemory("before_identity_tx");
  return sendRaw(txFrame_, frameLength, "identity_0x00000200");
}

bool SmaBluetoothClient::sendRaw(const uint8_t* data, size_t length, const char* label) {
  const size_t written = transport_.write(data, length);
  txBytes_ += written;
  emit("[SMA] TX label=%s len=%u written=%u", label, static_cast<unsigned>(length), static_cast<unsigned>(written));
  const size_t shown = min(length, static_cast<size_t>(48));
  char line[128];
  size_t used = snprintf(line, sizeof(line), "[SMA] frame TX ");
  for (size_t index = 0; index < shown && used + 4 < sizeof(line); ++index) {
    used += snprintf(line + used, sizeof(line) - used, "%02X", data[index]);
  }
  if (shown < length && used + 4 < sizeof(line)) snprintf(line + used, sizeof(line) - used, "...");
  if (logger_) logger_(line);
  return written == length;
}

bool SmaBluetoothClient::decodeIdentity(const uint8_t* l2, size_t length) {
  logMemory("after_identity_rx");
  if (length < 64 || l2[0] != 0x7E || read32(l2 + 1) != L2_SIGNATURE || l2[length - 1] != 0x7E) {
    fail("invalid_identity_structure");
    return false;
  }
  const uint16_t receivedFcs = read16(l2 + length - 3);
  const uint16_t calculatedFcs = fcs16(l2 + 1, length - 4);
  if (receivedFcs != calculatedFcs) {
    emit("[SMA] FCS invalid received=%04X calculated=%04X", receivedFcs, calculatedFcs);
    fail("invalid_l2_fcs");
    return false;
  }
  const uint16_t receivedPacketId = read16(l2 + 27) & 0x7FFF;
  if (receivedPacketId != packetId_) { fail("identity_packet_id_mismatch"); return false; }
  decodedSerial_ = read32(l2 + 57);
  if (decodedSerial_ != TARGET_SERIAL) {
    emit("[SMA] identity serial=%lu expected=%lu", static_cast<unsigned long>(decodedSerial_),
         static_cast<unsigned long>(TARGET_SERIAL));
    fail("identity_serial_mismatch");
    return false;
  }
  emit("[SMA] session valid fcs=%04X packet_id=%u serial=%lu", receivedFcs, receivedPacketId,
       static_cast<unsigned long>(decodedSerial_));
  return true;
}

uint16_t SmaBluetoothClient::read16(const uint8_t* data) {
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t SmaBluetoothClient::read32(const uint8_t* data) {
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

void SmaBluetoothClient::write16(uint8_t* data, size_t& position, uint16_t value) {
  data[position++] = value & 0xFF;
  data[position++] = (value >> 8) & 0xFF;
}

void SmaBluetoothClient::write32(uint8_t* data, size_t& position, uint32_t value) {
  data[position++] = value & 0xFF;
  data[position++] = (value >> 8) & 0xFF;
  data[position++] = (value >> 16) & 0xFF;
  data[position++] = (value >> 24) & 0xFF;
}

uint16_t SmaBluetoothClient::fcs16(const uint8_t* data, size_t length) {
  uint16_t checksum = 0xFFFF;
  for (size_t index = 0; index < length; ++index) {
    checksum ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      checksum = (checksum & 1) ? ((checksum >> 1) ^ 0x8408) : (checksum >> 1);
    }
  }
  return checksum ^ 0xFFFF;
}
