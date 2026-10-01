#pragma once

#include <stddef.h>
#include <stdint.h>

namespace SmaPhase3 {

enum class UserRole : uint32_t { User = 0x00000007UL, Installer = 0x0000000AUL };
enum class DecodeResult : uint8_t {
  Ok, BufferTooSmall, InvalidStructure, InvalidFcs, PacketIdMismatch,
  UnsupportedRecord, InvalidValue, NoData, DeviceError
};

enum class FieldState : uint8_t { NotAcquired, Unavailable, Valid };

struct RawField {
  FieldState state = FieldState::NotAcquired;
  uint64_t raw = 0;
  uint32_t timestamp = 0;
  uint8_t recordType = 0;
  uint8_t recordClass = 0;
  uint16_t recordSize = 0;
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
  RawField acTotalPower;
  RawField acPower1;
  RawField acVoltage1;
  RawField acCurrent1;
  RawField gridFrequency;
  RawField dcPower1;
  RawField dcVoltage1;
  RawField dcCurrent1;
  RawField totalEnergy;
  RawField todayEnergy;
  RawField temperature;
  RawField inverterStatus;
  RawField gridRelay;
  RawField operatingTime;
  RawField feedInTime;
  RawField inverterClass;
  RawField inverterType;
  RawField softwareVersion;
  FieldState inverterNameState = FieldState::NotAcquired;
  char inverterName[32]{};
  uint32_t inverterNameTimestamp = 0;
};

struct LoginResponse {
  uint16_t status = 0xFFFF;
  uint16_t inverterSusyId = 0;
  uint32_t inverterSerial = 0;
};

constexpr size_t kPasswordLength = 12;
constexpr uint16_t kLriAcPower = 0x263F;
constexpr uint16_t kLriTotalEnergy = 0x2601;
constexpr uint16_t kLriTodayEnergy = 0x2622;
constexpr uint16_t kLriAcPowerL1 = 0x4640;
constexpr uint16_t kLriAcVoltageL1 = 0x4648;
constexpr uint16_t kLriAcCurrentL1Legacy = 0x4650;
constexpr uint16_t kLriAcCurrentL1 = 0x4653;
constexpr uint16_t kLriGridFrequency = 0x4657;
constexpr uint16_t kLriDcPower = 0x251E;
constexpr uint16_t kLriDcVoltage = 0x451F;
constexpr uint16_t kLriDcCurrent = 0x4521;
constexpr uint16_t kLriTemperature = 0x2377;
constexpr uint16_t kLriInverterStatus = 0x2148;
constexpr uint16_t kLriGridRelay = 0x4164;
constexpr uint16_t kLriOperatingTime = 0x462E;
constexpr uint16_t kLriFeedInTime = 0x462F;
constexpr uint16_t kLriInverterName = 0x821E;
constexpr uint16_t kLriInverterClass = 0x821F;
constexpr uint16_t kLriInverterType = 0x8220;
constexpr uint16_t kLriSoftwareVersion = 0x8234;

constexpr Query kAcPowerQuery{0x51000200UL, 0x00263F00UL, 0x00263FFFUL};
constexpr Query kEnergyQuery{0x54000200UL, 0x00260100UL, 0x002622FFUL};
constexpr Query kAcVoltageCurrentQuery{0x51000200UL, 0x00464800UL, 0x004655FFUL};
constexpr Query kAcPowerL1Query{0x51000200UL, 0x00464000UL, 0x004642FFUL};
constexpr Query kGridFrequencyQuery{0x51000200UL, 0x00465700UL, 0x004657FFUL};
constexpr Query kDcPowerQuery{0x53800200UL, 0x00251E00UL, 0x00251EFFUL};
constexpr Query kDcVoltageCurrentQuery{0x53800200UL, 0x00451F00UL, 0x004521FFUL};
constexpr Query kTemperatureQuery{0x52000200UL, 0x00237700UL, 0x002377FFUL};
constexpr Query kStatusQuery{0x51800200UL, 0x00214800UL, 0x002148FFUL};
constexpr Query kGridRelayQuery{0x51800200UL, 0x00416400UL, 0x004164FFUL};
constexpr Query kOperatingTimeQuery{0x54000200UL, 0x00462E00UL, 0x00462FFFUL};
constexpr Query kTypeLabelQuery{0x58000200UL, 0x00821E00UL, 0x008220FFUL};
constexpr Query kInverterNameQuery{0x58000200UL, 0x00821E00UL, 0x00821EFFUL};
constexpr Query kInverterClassQuery{0x58000200UL, 0x00821F00UL, 0x00821FFFUL};
constexpr Query kInverterTypeQuery{0x58000200UL, 0x00822000UL, 0x008220FFUL};
constexpr Query kSoftwareVersionQuery{0x58000200UL, 0x00823400UL, 0x008234FFUL};

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
