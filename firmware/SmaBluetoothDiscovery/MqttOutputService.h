#pragma once

#include <Preferences.h>
#include <PubSubClient.h>
#include <WiFiClient.h>

#include "SbfspotCompat.h"
#include "ProductConfiguration.h"

class MqttOutputService {
 public:
  MqttOutputService();
  void load(Preferences& preferences);
  bool save(Preferences& preferences, const SbfspotCompat::MqttConfig& config, bool replacePassword);
  void begin();
  void tick(bool wifiConnected, bool otaBusy,
            const SbfspotCompat::InverterSnapshot (&snapshots)[ProductConfig::kInverterCount]);
  void disconnect();

  const SbfspotCompat::MqttConfig& config() const { return config_; }
  bool connected() { return mqtt_.connected(); }
  bool configured() const;
  uint32_t lastConnectAttempt() const { return lastConnectAttempt_; }
  int lastConnectResult() const { return lastConnectResult_; }
  uint32_t reconnectCount() const { return reconnectCount_; }
  uint32_t publishCount() const { return publishCount_; }
  uint32_t publishFailures() const { return publishFailures_; }
  uint32_t lastPublishMs() const { return lastPublishMs_; }
  size_t lastPayloadBytes() const { return lastPayloadBytes_; }
  bool topic(size_t slot, char* output, size_t capacity) const;

 private:
  static constexpr uint32_t kReconnectBackoffMs = 30000;
  static constexpr size_t kPayloadCapacity = 1024;
  WiFiClient network_;
  PubSubClient mqtt_;
  SbfspotCompat::MqttConfig config_;
  uint32_t nextConnectAt_ = 0;
  uint32_t lastConnectAttempt_ = 0;
  int lastConnectResult_ = 0;
  uint32_t reconnectCount_ = 0;
  uint32_t publishCount_ = 0;
  uint32_t publishFailures_ = 0;
  uint32_t lastPublishMs_ = 0;
  size_t nextSlot_ = 0;
  size_t lastPayloadBytes_ = 0;
  char payload_[kPayloadCapacity]{};
};
