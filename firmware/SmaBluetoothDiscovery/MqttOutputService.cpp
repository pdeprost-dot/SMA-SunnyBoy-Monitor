#include "MqttOutputService.h"

#include <Arduino.h>

namespace {
void copyPreference(Preferences& preferences, const char* key, const char* fallback, char* output, size_t capacity) {
  const String value = preferences.getString(key, fallback);
  strlcpy(output, value.c_str(), capacity);
}
}  // namespace

MqttOutputService::MqttOutputService() : mqtt_(network_) {}

void MqttOutputService::load(Preferences& preferences) {
  preferences.begin("sma-monitor", true);
  config_.enabled = preferences.getBool("mqttEn", false);
  copyPreference(preferences, "mqttHost", "", config_.broker, sizeof(config_.broker));
  config_.port = preferences.getUShort("mqttPort", 1883);
  copyPreference(preferences, "mqttUser", "", config_.username, sizeof(config_.username));
  copyPreference(preferences, "mqttPass", "", config_.password, sizeof(config_.password));
  copyPreference(preferences, "mqttPrefix", "", config_.topicPrefix, sizeof(config_.topicPrefix));
  if (!ProductConfig::validTopicPrefix(config_.topicPrefix)) {
    // Backward-compatible migration: old full topics are intentionally not reused.
    strlcpy(config_.topicPrefix, "smaesp", sizeof(config_.topicPrefix));
  }
  config_.publishIntervalSeconds = preferences.getUInt("mqttEvery", 300);
  preferences.end();
  if (!SbfspotCompat::validateMqttConfig(config_)) config_ = SbfspotCompat::MqttConfig{};
}

bool MqttOutputService::save(Preferences& preferences, const SbfspotCompat::MqttConfig& candidate,
                             bool replacePassword) {
  if (!SbfspotCompat::validateMqttConfig(candidate)) return false;
  SbfspotCompat::MqttConfig saved = candidate;
  if (!replacePassword) strlcpy(saved.password, config_.password, sizeof(saved.password));
  preferences.begin("sma-monitor", false);
  preferences.putBool("mqttEn", saved.enabled);
  preferences.putString("mqttHost", saved.broker);
  preferences.putUShort("mqttPort", saved.port);
  preferences.putString("mqttUser", saved.username);
  if (replacePassword) preferences.putString("mqttPass", saved.password);
  preferences.putString("mqttPrefix", saved.topicPrefix);
  preferences.putUInt("mqttEvery", saved.publishIntervalSeconds);
  preferences.end();
  disconnect();
  config_ = saved;
  begin();
  return true;
}

void MqttOutputService::begin() {
  network_.setTimeout(2000);
  mqtt_.setSocketTimeout(2);
  mqtt_.setBufferSize(kPayloadCapacity + 128);
  mqtt_.setServer(config_.broker, config_.port);
  nextConnectAt_ = 0;
}

bool MqttOutputService::configured() const {
  return SbfspotCompat::validateMqttConfig(config_) && config_.broker[0] &&
         ProductConfig::validTopicPrefix(config_.topicPrefix);
}

bool MqttOutputService::topic(size_t slot, char* output, size_t capacity) const {
  return ProductConfig::makeTopic(config_.topicPrefix, slot, output, capacity);
}

void MqttOutputService::disconnect() {
  if (mqtt_.connected()) mqtt_.disconnect();
  network_.stop();
}

void MqttOutputService::tick(bool wifiConnected, bool otaBusy,
                            const SbfspotCompat::InverterSnapshot (&snapshots)[ProductConfig::kInverterCount]) {
  const uint32_t now = millis();
  if (!config_.enabled || !configured() || !wifiConnected || otaBusy) {
    if (mqtt_.connected()) disconnect();
    return;
  }
  if (!mqtt_.connected()) {
    if (static_cast<int32_t>(now - nextConnectAt_) < 0) return;
    lastConnectAttempt_ = now;
    ++reconnectCount_;
    char clientId[32];
    snprintf(clientId, sizeof(clientId), "sma-monitor-%08lX", static_cast<unsigned long>(ESP.getEfuseMac()));
    const bool ok = config_.username[0]
        ? mqtt_.connect(clientId, config_.username, config_.password)
        : mqtt_.connect(clientId);
    lastConnectResult_ = ok ? 0 : mqtt_.state();
    nextConnectAt_ = now + kReconnectBackoffMs;
    if (!ok) return;
  }
  mqtt_.loop();
  const uint32_t interval = config_.publishIntervalSeconds * 1000UL;
  if (!publishPendingMask_ && lastPublishMs_ && now - lastPublishMs_ < interval) return;
  size_t slot = nextSlot_;
  for (size_t checked = 0; checked < ProductConfig::kInverterCount; ++checked) {
    const bool requested = !publishPendingMask_ || (publishPendingMask_ & (1U << slot));
    if (requested && snapshots[slot].acquisitionValid) break;
    if (publishPendingMask_ && requested) publishPendingMask_ &= ~(1U << slot);
    slot = (slot + 1) % ProductConfig::kInverterCount;
  }
  if (!snapshots[slot].acquisitionValid) return;  // Never publish placeholder data.
  size_t bytes = 0;
  if (!SbfspotCompat::serialize(snapshots[slot], payload_, sizeof(payload_), bytes)) {
    publishPendingMask_ &= ~(1U << slot);
    ++publishFailures_;
    lastPayloadBytes_ = 0;
    lastPublishMs_ = now;
    return;
  }
  lastPayloadBytes_ = bytes;
  char publishTopic[64];
  if (!topic(slot, publishTopic, sizeof(publishTopic))) {
    publishPendingMask_ &= ~(1U << slot); ++publishFailures_; return;
  }
  if (mqtt_.publish(publishTopic, reinterpret_cast<const uint8_t*>(payload_), bytes, true)) ++publishCount_;
  else ++publishFailures_;
  publishPendingMask_ &= ~(1U << slot);
  lastPublishMs_ = now;
  nextSlot_ = (slot + 1) % ProductConfig::kInverterCount;
}
