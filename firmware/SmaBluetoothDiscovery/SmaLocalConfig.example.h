#pragma once

#include <Arduino.h>

// Copy this file to SmaLocalConfig.h and replace these fictitious identifiers.
// SmaLocalConfig.h is intentionally ignored by Git.
namespace SmaLocalConfig {
struct Device {
  const char* mac;
  const char* label;
  uint32_t serial;
};

static constexpr Device KNOWN_SMAS[] = {
    {"02:00:00:00:00:01", "SMA #1", 1000000001UL},
    {"02:00:00:00:00:02", "SMA #2", 1000000002UL},
    {"02:00:00:00:00:03", "SMA #3", 1000000003UL},
};

static constexpr const char* TARGET_MAC = "02:00:00:00:00:01";
static constexpr uint32_t TARGET_SERIAL = 1000000001UL;
static constexpr uint8_t TARGET_CONNECT_ADDRESS[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
static constexpr uint8_t TARGET_PROTOCOL_ADDRESS[6] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x02};
}
