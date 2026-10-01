#include "SmaBluetoothClient.h"
#include "SmaPhase3Protocol.h"

#include <esp_heap_caps.h>
#include <stdarg.h>

namespace {
constexpr uint8_t L1_START = 0x7E;
constexpr size_t L1_HEADER_SIZE = 18;
constexpr uint32_t L2_SIGNATURE = 0x656003FFUL;
constexpr uint16_t APP_SUSY_ID = 125;
constexpr uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr SmaPhase3::Query DATASET_QUERIES[] = {
  SmaPhase3::kEnergyQuery,
  SmaPhase3::kOperatingTimeQuery,
  SmaPhase3::kDcPowerQuery,
  SmaPhase3::kDcVoltageCurrentQuery,
  SmaPhase3::kAcPowerL1Query,
  SmaPhase3::kAcVoltageCurrentQuery,
  SmaPhase3::kAcPowerQuery,
  SmaPhase3::kGridFrequencyQuery,
  SmaPhase3::kTemperatureQuery,
  SmaPhase3::kStatusQuery,
  SmaPhase3::kGridRelayQuery,
  SmaPhase3::kInverterNameQuery,
  SmaPhase3::kInverterClassQuery,
  SmaPhase3::kInverterTypeQuery,
  SmaPhase3::kSoftwareVersionQuery,
};
constexpr size_t DATASET_QUERY_COUNT = sizeof(DATASET_QUERIES) / sizeof(DATASET_QUERIES[0]);
}

SmaBluetoothClient::SmaBluetoothClient(BluetoothSerial& transport, LogFn logger)
    : transport_(transport), logger_(logger) {}

bool SmaBluetoothClient::setTarget(const char* mac, uint32_t serial) {
  if (state_ != State::DISCONNECTED && state_ != State::ERROR) return false;
  unsigned values[6]{};
  char trailing = 0;
  if (mac == nullptr || serial == 0 ||
      sscanf(mac, "%2x:%2x:%2x:%2x:%2x:%2x%c", &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5], &trailing) != 6) return false;
  for (size_t index = 0; index < 6; ++index) {
    if (values[index] > 0xFF) return false;
    targetConnectAddress_[index] = static_cast<uint8_t>(values[index]);
    targetProtocolAddress_[5 - index] = static_cast<uint8_t>(values[index]);
  }
  expectedSerial_ = serial;
  return true;
}

const char* SmaBluetoothClient::stateName() const {
  switch (state_) {
    case State::DISCONNECTED: return "DISCONNECTED";
    case State::CONNECTING: return "CONNECTING";
    case State::WAIT_ANNOUNCE: return "WAIT_ANNOUNCE";
    case State::WAIT_LOCAL_ADDRESS: return "WAIT_LOCAL_ADDRESS";
    case State::WAIT_IDENTITY: return "WAIT_IDENTITY";
    case State::SESSION_READY: return "SESSION_READY";
    case State::WAIT_LOGIN: return "WAIT_LOGIN";
    case State::WAIT_DATASET: return "WAIT_DATASET";
    case State::PHASE3_COMPLETE: return "PHASE3_COMPLETE";
    case State::DISCONNECTING: return "DISCONNECTING";
    case State::ERROR: return "ERROR";
  }
  return "ERROR";
}

bool SmaBluetoothClient::bluetoothConnected() const {
  return transport_.connected(0);
}

const char* SmaBluetoothClient::acPowerClassification() const {
  if (acPowerValid_) return "PACTOT_VALID";
  if (acPowerUnavailable_) return "PACTOT_UNAVAILABLE";
  return "PACTOT_PROTOCOL_ERROR";
}

bool SmaBluetoothClient::datasetUsable() const {
  using S = SmaPhase3::FieldState;
  return measurements_.acTotalPower.state == S::Valid || measurements_.acPower1.state == S::Valid ||
         measurements_.acVoltage1.state == S::Valid || measurements_.acCurrent1.state == S::Valid ||
         measurements_.gridFrequency.state == S::Valid || measurements_.dcPower1.state == S::Valid ||
         measurements_.dcVoltage1.state == S::Valid || measurements_.dcCurrent1.state == S::Valid ||
         measurements_.totalEnergy.state == S::Valid || measurements_.todayEnergy.state == S::Valid ||
         measurements_.temperature.state == S::Valid || measurements_.operatingTime.state == S::Valid ||
         measurements_.feedInTime.state == S::Valid || measurements_.inverterNameState == S::Valid;
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
      case State::WAIT_LOGIN: return "WAIT_LOGIN";
      case State::WAIT_DATASET: return "WAIT_DATASET";
      case State::PHASE3_COMPLETE: return "PHASE3_COMPLETE";
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
  loginValid_ = false;
  loginStatus_ = 0xFFFF;
  loginPacketId_ = 0;
  loginTimestamp_ = 0;
  acPowerValid_ = false;
  acPowerUnavailable_ = false;
  acPowerW_ = 0;
  acPowerPacketId_ = 0;
  returnedLri_ = 0;
  returnedRecordType_ = 0;
  returnedRecordSize_ = 0;
  measurementTimestamp_ = 0;
  measurements_ = {};
  datasetQueryIndex_ = 0;
  datasetPacketId_ = 0;
  datasetUnsupportedResponses_ = 0;
  lastDatasetQueryIndex_ = 0;
  lastDatasetDecodeResult_ = SmaPhase3::DecodeResult::Ok;
  memset(datasetL1Fragments_, 0, sizeof(datasetL1Fragments_));
  memset(datasetL1Bytes_, 0, sizeof(datasetL1Bytes_));
  memset(datasetL2Packets_, 0, sizeof(datasetL2Packets_));
  memset(datasetL2Bytes_, 0, sizeof(datasetL2Bytes_));
  for (size_t index = 0; index < DATASET_DIAGNOSTIC_COUNT; ++index)
    datasetDecodeResults_[index] = SmaPhase3::DecodeResult::NoData;
  inverterSusyId_ = 0;
  inverterSerial_ = 0;
  txBytes_ = 0;
  rxBytes_ = 0;
  packetId_ = 1;
  netId_ = 0;
  memset(localProtocolAddress_, 0, sizeof(localProtocolAddress_));
  resetParser();
  resetL2Parser();
  connectTaskDone_ = false;
  connectTaskResult_ = false;
  connectTaskStarted_ = false;
  logMemory("before_connect");
  setState(State::CONNECTING);
  if (static_cast<int32_t>(millis() - reconnectAllowedAt_) < 0) {
    emit("[SMA] connect deferred guard_ms=%lu",
         static_cast<unsigned long>(reconnectAllowedAt_ - millis()));
    return true;
  }
  return startConnectTask();
}

bool SmaBluetoothClient::startConnectTask() {
  if (xTaskCreatePinnedToCore(connectTaskEntry, "sma-connect", 4096, this, 1, &connectTaskHandle_, 0) != pdPASS) {
    connectTaskHandle_ = nullptr;
    fail("connect_task_create_failed");
    return false;
  }
  connectTaskStarted_ = true;
  return true;
}

void SmaBluetoothClient::connectTaskEntry(void* context) {
  static_cast<SmaBluetoothClient*>(context)->runConnectTask();
}

void SmaBluetoothClient::runConnectTask() {
  // The SB2500HF-30 exposed RFCOMM channel 1 during the first measured SDP.
  // Connecting directly avoids the legacy inverter's intermittent SDP failure.
  emit("[BT-TIMELINE] connect_request t=%lu channel=1 ready=%s state=%s",
       static_cast<unsigned long>(millis()), transport_.isReady(false, 0) ? "true" : "false", stateName());
  connectTaskResult_ = transport_.connect(targetConnectAddress_, 1);
  emit("[BT-TIMELINE] connect_return t=%lu result=%s closed=%s connected=%s",
       static_cast<unsigned long>(millis()), connectTaskResult_ ? "true" : "false",
       transport_.isClosed() ? "true" : "false", transport_.connected(0) ? "true" : "false");
  connectTaskStackHighWater_ = uxTaskGetStackHighWaterMark(nullptr);
  emit("[BT-TIMELINE] connect_task_stack_high_water=%u",
       static_cast<unsigned>(connectTaskStackHighWater_));
  connectTaskDone_ = true;
  connectTaskHandle_ = nullptr;
  vTaskDelete(nullptr);
}

bool SmaBluetoothClient::requestDisconnect() {
  if (state_ == State::DISCONNECTED || state_ == State::CONNECTING) return false;
  setState(State::DISCONNECTING);
  const bool result = transport_.disconnect();
  emit("[SMA] transport.disconnect returned=%s connected=%s",
       result ? "true" : "false", transport_.connected(0) ? "true" : "false");
  reconnectAllowedAt_ = millis() + RECONNECT_GUARD_MS;
  resetParser();
  resetL2Parser();
  setState(State::DISCONNECTED);
  logMemory("after_disconnect");
  return result;
}

void SmaBluetoothClient::tick() {
  if (state_ == State::CONNECTING && !connectTaskStarted_ && !connectTaskDone_ &&
      static_cast<int32_t>(millis() - reconnectAllowedAt_) >= 0) {
    if (!startConnectTask()) return;
  }
  if (state_ == State::CONNECTING && connectTaskDone_) {
    connectTaskDone_ = false;
    connectTaskStarted_ = false;
    if (!connectTaskResult_) {
      fail("bt_connect_failed");
      return;
    }
    emit("[SMA] connect result=connected");
    logMemory("after_bt_connect");
    deadlineAt_ = millis() + STEP_TIMEOUT_MS;
    setState(State::WAIT_ANNOUNCE);
  }

  if (state_ == State::WAIT_ANNOUNCE || state_ == State::WAIT_LOCAL_ADDRESS ||
      state_ == State::WAIT_IDENTITY || state_ == State::SESSION_READY ||
      state_ == State::WAIT_LOGIN || state_ == State::WAIT_DATASET ||
      state_ == State::PHASE3_COMPLETE) {
    if (!transport_.connected(0)) {
      fail("bt_disconnected");
      return;
    }
    receiveBytes();
  }

  if ((state_ == State::WAIT_ANNOUNCE || state_ == State::WAIT_LOCAL_ADDRESS ||
       state_ == State::WAIT_IDENTITY || state_ == State::WAIT_LOGIN ||
       state_ == State::WAIT_DATASET) && static_cast<int32_t>(millis() - deadlineAt_) >= 0) {
    fail("protocol_timeout");
  }
}

void SmaBluetoothClient::resetParser() {
  rxPosition_ = 0;
  rxExpected_ = 0;
}

void SmaBluetoothClient::resetL2Parser() {
  l2Position_ = 0;
  l2Active_ = false;
  l2EscapePending_ = false;
}

bool SmaBluetoothClient::appendL2Payload(const uint8_t* data, size_t length) {
  for (size_t index = 0; index < length; ++index) {
    uint8_t value = data[index];
    if (l2EscapePending_) {
      value ^= 0x20;
      l2EscapePending_ = false;
    } else if (value == 0x7D) {
      l2EscapePending_ = true;
      continue;
    }
    if (l2Position_ >= sizeof(l2Frame_)) {
      fail("l2_overflow");
      resetL2Parser();
      return false;
    }
    l2Frame_[l2Position_++] = value;
  }
  return true;
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
  char hexLine[128];
  const size_t shown = min(rxPosition_, static_cast<size_t>(48));
  size_t used = snprintf(hexLine, sizeof(hexLine), "[SMA] frame RX ");
  for (size_t index = 0; index < shown && used + 4 < sizeof(hexLine); ++index) {
    used += snprintf(hexLine + used, sizeof(hexLine) - used, "%02X", rxFrame_[index]);
  }
  if (shown < rxPosition_ && used + 4 < sizeof(hexLine)) snprintf(hexLine + used, sizeof(hexLine) - used, "...");
  if (logger_) logger_(hexLine);
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
    uint8_t localAddress[6] = {};
    transport_.getBtAddress(localAddress);
    for (size_t index = 0; index < 6; ++index) localProtocolAddress_[index] = localAddress[5 - index];
    emit("[SMA] local_bt=%02X:%02X:%02X:%02X:%02X:%02X",
         localProtocolAddress_[5], localProtocolAddress_[4], localProtocolAddress_[3],
         localProtocolAddress_[2], localProtocolAddress_[1], localProtocolAddress_[0]);
    if (!sendIdentityRequest()) { fail("identity_tx_failed"); return; }
    deadlineAt_ = millis() + STEP_TIMEOUT_MS;
    setState(State::WAIT_IDENTITY);
    return;
  }

  if (state_ == State::WAIT_IDENTITY || state_ == State::WAIT_LOGIN ||
      state_ == State::WAIT_DATASET) {
    const size_t payloadLength = rxPosition_ - L1_HEADER_SIZE;
    const uint8_t* payload = rxFrame_ + L1_HEADER_SIZE;
    const bool startsL2 = payloadLength >= 5 && payload[0] == 0x7E &&
                          read32(payload + 1) == L2_SIGNATURE;
    if (!l2Active_ && startsL2) {
      resetL2Parser();
      l2Active_ = true;
      emit("[SMA] L2 start cmd=0x%04X fragment_bytes=%u", command,
           static_cast<unsigned>(payloadLength));
    }
    if (l2Active_) {
      if (state_ == State::WAIT_DATASET && datasetQueryIndex_ < DATASET_DIAGNOSTIC_COUNT) {
        ++datasetL1Fragments_[datasetQueryIndex_];
        datasetL1Bytes_[datasetQueryIndex_] += static_cast<uint16_t>(rxPosition_);
      }
      if (!appendL2Payload(payload, payloadLength)) return;
      emit("[SMA] L2 fragment cmd=0x%04X accumulated=%u", command,
           static_cast<unsigned>(l2Position_));
      if (command == 0x0001) {
        if (l2EscapePending_) {
          fail("l2_dangling_escape");
          resetL2Parser();
          return;
        }
        const size_t completeLength = l2Position_;
        if (state_ == State::WAIT_DATASET && datasetQueryIndex_ < DATASET_DIAGNOSTIC_COUNT) {
          ++datasetL2Packets_[datasetQueryIndex_];
          datasetL2Bytes_[datasetQueryIndex_] += static_cast<uint16_t>(completeLength);
        }
        l2Active_ = false;
        if (!processData2Response(l2Frame_, completeLength)) {
          resetL2Parser();
          return;
        }
        resetL2Parser();
      }
    }
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
  write32(output, position, appSerial_);
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

bool SmaBluetoothClient::startPhase3Transaction() {
  if (state_ != State::SESSION_READY) return false;
  return sendLoginRequest();
}

bool SmaBluetoothClient::setUserPassword(const char* value) {
  if (!value) return false;
  const size_t length = strnlen(value, sizeof(userPassword_));
  if (length == 0 || length > SmaPhase3::kPasswordLength) return false;
  strlcpy(userPassword_, value, sizeof(userPassword_));
  return true;
}

bool SmaBluetoothClient::sendLoginRequest() {
  loginTimestamp_ = static_cast<uint32_t>(time(nullptr));
  loginPacketId_ = ++packetId_;
  uint8_t l2[96];
  const size_t l2Length = SmaPhase3::buildLoginL2(
      l2, sizeof(l2), loginPacketId_, APP_SUSY_ID, appSerial_,
      SmaPhase3::UserRole::User, loginTimestamp_, userPassword_);
  const size_t frameLength = wrapL2(l2, l2Length, txFrame_, sizeof(txFrame_));
  if (frameLength == 0) return false;
  emit("[PHASE3] login TX packet_id=%u timestamp=%lu app_serial=%lu role=USER", loginPacketId_,
       static_cast<unsigned long>(loginTimestamp_), static_cast<unsigned long>(appSerial_));
  if (!sendRaw(txFrame_, frameLength, "login_user")) return false;
  deadlineAt_ = millis() + STEP_TIMEOUT_MS;
  setState(State::WAIT_LOGIN);
  return true;
}

bool SmaBluetoothClient::sendNextDatasetRequest(uint16_t inverterSusyId, uint32_t inverterSerial) {
  if (datasetQueryIndex_ >= DATASET_QUERY_COUNT) {
    setState(State::PHASE3_COMPLETE);
    return true;
  }
  const SmaPhase3::Query& query = DATASET_QUERIES[datasetQueryIndex_];
  datasetPacketId_ = ++packetId_;
  if (query.first == SmaPhase3::kAcPowerQuery.first) acPowerPacketId_ = datasetPacketId_;
  uint8_t l2[64];
  const size_t l2Length = SmaPhase3::buildQueryL2(
      l2, sizeof(l2), datasetPacketId_, APP_SUSY_ID, appSerial_,
      inverterSusyId, inverterSerial, query);
  const size_t frameLength = wrapL2(l2, l2Length, txFrame_, sizeof(txFrame_));
  if (frameLength == 0) return false;
  emit("[DATASET] TX index=%u packet_id=%u command=0x%08lX first=0x%08lX last=0x%08lX",
       static_cast<unsigned>(datasetQueryIndex_), datasetPacketId_,
       static_cast<unsigned long>(query.command), static_cast<unsigned long>(query.first),
       static_cast<unsigned long>(query.last));
  if (!sendRaw(txFrame_, frameLength, "dataset_query")) return false;
  deadlineAt_ = millis() + STEP_TIMEOUT_MS;
  setState(State::WAIT_DATASET);
  return true;
}

bool SmaBluetoothClient::processData2Response(const uint8_t* l2, size_t length) {
  if (state_ == State::WAIT_IDENTITY) {
    if (!decodeIdentity(l2, length)) return false;
    ++validResponses_;
    lastValidResponseAt_ = millis();
    logMemory("after_decode");
    setState(State::SESSION_READY);
    return true;
  }
  if (state_ == State::WAIT_LOGIN) {
    SmaPhase3::LoginResponse response{};
    const SmaPhase3::DecodeResult result = SmaPhase3::validateLoginResponse(
        l2, length, loginPacketId_, loginTimestamp_, response);
    loginStatus_ = response.status;
    emit("[PHASE3] login RX len=%u packet_id=%u status=0x%04X timestamp=%lu susy=%u serial=%lu result=%u",
         static_cast<unsigned>(length), loginPacketId_, loginStatus_,
         static_cast<unsigned long>(loginTimestamp_), response.inverterSusyId,
         static_cast<unsigned long>(response.inverterSerial), static_cast<unsigned>(result));
    if (result != SmaPhase3::DecodeResult::Ok) {
      fail(loginStatus_ == 0x0100 ? "phase3_login_invalid_password" : "phase3_login_invalid_response");
      return false;
    }
    if (response.inverterSerial != expectedSerial_) {
      fail("phase3_login_serial_mismatch");
      return false;
    }
    loginValid_ = true;
    ++validResponses_;
    lastValidResponseAt_ = millis();
    logMemory("after_login");
    inverterSusyId_ = response.inverterSusyId;
    inverterSerial_ = response.inverterSerial;
    datasetQueryIndex_ = 0;
    if (!sendNextDatasetRequest(inverterSusyId_, inverterSerial_)) {
      fail("phase3_dataset_tx_failed");
      return false;
    }
    return true;
  }
  if (state_ == State::WAIT_DATASET) {
    const uint16_t remainingPackets = length >= 29
        ? static_cast<uint16_t>(l2[25] | (static_cast<uint16_t>(l2[26]) << 8))
        : 0;
    const SmaPhase3::DecodeResult result =
        SmaPhase3::decodeMeasurementResponse(l2, length, datasetPacketId_, measurements_);
    lastDatasetQueryIndex_ = datasetQueryIndex_;
    lastDatasetDecodeResult_ = result;
    if (datasetQueryIndex_ < DATASET_DIAGNOSTIC_COUNT)
      datasetDecodeResults_[datasetQueryIndex_] = result;
    emit("[DATASET] RX index=%u len=%u packet_id=%u remaining=%u result=%u lri=0x%04X type=0x%02X record_size=%u raw=%llu timestamp=%lu",
         static_cast<unsigned>(datasetQueryIndex_), static_cast<unsigned>(length), datasetPacketId_,
         remainingPackets, static_cast<unsigned>(result), measurements_.returnedLri, measurements_.recordType,
         measurements_.recordSize, static_cast<unsigned long long>(measurements_.rawValue),
         static_cast<unsigned long>(measurements_.timestamp));
    if (result == SmaPhase3::DecodeResult::InvalidFcs ||
        result == SmaPhase3::DecodeResult::PacketIdMismatch ||
        result == SmaPhase3::DecodeResult::InvalidStructure) {
      fail("phase3_dataset_invalid_response");
      return false;
    }
    if (result == SmaPhase3::DecodeResult::UnsupportedRecord ||
        result == SmaPhase3::DecodeResult::InvalidValue ||
        result == SmaPhase3::DecodeResult::NoData ||
        result == SmaPhase3::DecodeResult::DeviceError ||
        result == SmaPhase3::DecodeResult::BufferTooSmall) ++datasetUnsupportedResponses_;
    acPowerValid_ = measurements_.acTotalPower.state == SmaPhase3::FieldState::Valid;
    acPowerUnavailable_ = measurements_.acTotalPower.state == SmaPhase3::FieldState::Unavailable;
    acPowerW_ = acPowerValid_ ? static_cast<uint32_t>(measurements_.acTotalPower.raw) : 0;
    returnedLri_ = measurements_.returnedLri;
    returnedRecordType_ = measurements_.recordType;
    returnedRecordSize_ = measurements_.recordSize;
    measurementTimestamp_ = measurements_.acTotalPower.timestamp;
    ++validResponses_;
    lastValidResponseAt_ = millis();
    logMemory("after_dataset_response");
    // A range response may span several Data2+ packets. Offset 25 contains
    // SMA's remaining-packet countdown; keep the same query active until zero.
    if (result == SmaPhase3::DecodeResult::Ok && remainingPackets > 0) {
      deadlineAt_ = millis() + STEP_TIMEOUT_MS;
      return true;
    }
    ++datasetQueryIndex_;
    if (datasetQueryIndex_ >= DATASET_QUERY_COUNT) {
      setState(State::PHASE3_COMPLETE);
      return true;
    }
    if (!sendNextDatasetRequest(inverterSusyId_, inverterSerial_)) {
      fail("phase3_dataset_tx_failed");
      return false;
    }
    return true;
  }
  return false;
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
  if (decodedSerial_ != expectedSerial_) {
    emit("[SMA] identity serial=%lu expected=%lu", static_cast<unsigned long>(decodedSerial_),
         static_cast<unsigned long>(expectedSerial_));
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
