#pragma once

#include <stddef.h>
#include <stdint.h>

namespace SmaPhase3 {

enum class UserRole : uint32_t { User = 0x00000007UL, Installer = 0x0000000AUL };
enum class DecodeResult : uint8_t {
  Ok, BufferTooSmall, InvalidStructure, InvalidFcs, PacketIdMismatch,
  UnsupportedRecord, InvalidValue
};

struct Query {
  uint32_t command;
  uint32_t first;
  uint32_t last;
};

struct Measurements {
  bool valid = false;
  uint32_t timestamp = 0;
  uint16_t returnedLri = 0;
  uint8_t recordType = 0;
  uint16_t recordSize = 0;
  uint64_t rawValue = 0;
  bool acPowerValid = false;
  uint32_t acPowerW = 0;
  bool totalEnergyValid = false;
  uint64_t totalEnergyWh = 0;
  bool acVoltageValid = false;
  float acVoltageV = 0;
  bool acCurrentValid = false;
  float acCurrentA = 0;
};

struct LoginResponse {
  uint16_t status = 0xFFFF;
  uint16_t inverterSusyId = 0;
  uint32_t inverterSerial = 0;
};

constexpr size_t kPasswordLength = 12;
constexpr uint16_t kLriAcPower = 0x263F;
constexpr uint16_t kLriTotalEnergy = 0x2601;
constexpr uint16_t kLriAcVoltageL1 = 0x4648;
constexpr uint16_t kLriAcCurrentL1Legacy = 0x4650;
constexpr uint16_t kLriAcCurrentL1 = 0x4653;

constexpr Query kAcPowerQuery{0x51000200UL, 0x00263F00UL, 0x00263FFFUL};
constexpr Query kEnergyQuery{0x54000200UL, 0x00260100UL, 0x002622FFUL};
constexpr Query kAcVoltageCurrentQuery{0x51000200UL, 0x00464800UL, 0x004655FFUL};

bool encodePassword(const char* password, UserRole role, uint8_t output[kPasswordLength]);
size_t buildLoginL2(uint8_t* output, size_t capacity, uint16_t packetId,
                    uint16_t appSusyId, uint32_t appSerial, UserRole role,
                    uint32_t unixTime, const char* password);
size_t buildQueryL2(uint8_t* output, size_t capacity, uint16_t packetId,
                    uint16_t appSusyId, uint32_t appSerial, uint16_t inverterSusyId,
                    uint32_t inverterSerial, const Query& query);
DecodeResult validateLoginResponse(const uint8_t* l2, size_t length,
                                   uint16_t expectedPacketId, uint32_t expectedUnixTime,
                                   LoginResponse& response);
DecodeResult decodeMeasurementResponse(const uint8_t* l2, size_t length,
                                       uint16_t expectedPacketId, Measurements& output);
uint16_t fcs16(const uint8_t* data, size_t length);

}  // namespace SmaPhase3
