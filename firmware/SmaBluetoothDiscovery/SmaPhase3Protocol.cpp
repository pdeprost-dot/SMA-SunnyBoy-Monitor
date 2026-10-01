#include "SmaPhase3Protocol.h"

#include <string.h>

namespace SmaPhase3 {
namespace {
constexpr uint32_t kSignature = 0x656003FFUL;
constexpr uint32_t kInvalid32A = 0x80000000UL;
constexpr uint32_t kInvalid32B = 0xFFFFFFFFUL;
constexpr uint64_t kInvalid64A = UINT64_C(0x8000000000000000);
constexpr uint64_t kInvalid64B = UINT64_C(0xFFFFFFFFFFFFFFFF);

uint16_t read16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
uint32_t read32(const uint8_t* p) {
  return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
uint64_t read64(const uint8_t* p) { return uint64_t(read32(p)) | (uint64_t(read32(p + 4)) << 32); }
void write16(uint8_t* p, size_t& n, uint16_t v) { p[n++] = uint8_t(v); p[n++] = uint8_t(v >> 8); }
void write32(uint8_t* p, size_t& n, uint32_t v) {
  p[n++] = uint8_t(v); p[n++] = uint8_t(v >> 8); p[n++] = uint8_t(v >> 16); p[n++] = uint8_t(v >> 24);
}
void finish(uint8_t* p, size_t& n) { write16(p, n, fcs16(p + 1, n - 1)); p[n++] = 0x7E; }
void header(uint8_t* p, size_t& n, uint8_t words, uint8_t ctrl, uint16_t ctrl2,
            uint16_t dstSusy, uint32_t dstSerial, uint16_t appSusy, uint32_t appSerial,
            uint16_t packetId) {
  p[n++] = 0x7E; write32(p, n, kSignature); p[n++] = words; p[n++] = ctrl;
  write16(p, n, dstSusy); write32(p, n, dstSerial); write16(p, n, ctrl2);
  write16(p, n, appSusy); write32(p, n, appSerial); write16(p, n, ctrl2);
  write16(p, n, 0); write16(p, n, 0); write16(p, n, packetId | 0x8000U);
}
}  // namespace

uint16_t fcs16(const uint8_t* data, size_t length) {
  uint16_t value = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    value ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) value = (value & 1) ? uint16_t((value >> 1) ^ 0x8408) : uint16_t(value >> 1);
  }
  return value ^ 0xFFFF;
}

bool encodePassword(const char* password, UserRole role, uint8_t output[kPasswordLength]) {
  if (!password || !output) return false;
  const size_t length = strlen(password);
  if (length > kPasswordLength) return false;
  const uint8_t offset = role == UserRole::User ? 0x88 : 0xBB;
  for (size_t i = 0; i < kPasswordLength; ++i)
    output[i] = uint8_t((i < length ? uint8_t(password[i]) : 0U) + offset);
  return true;
}

size_t buildLoginL2(uint8_t* output, size_t capacity, uint16_t packetId,
                    uint16_t appSusyId, uint32_t appSerial, UserRole role,
                    uint32_t unixTime, const char* password) {
  if (!output || capacity < 64) return 0;
  uint8_t encoded[kPasswordLength];
  if (!encodePassword(password, role, encoded)) return 0;
  size_t n = 0;
  header(output, n, 0x0E, 0xA0, 0x0100, 0xFFFF, 0xFFFFFFFFUL, appSusyId, appSerial, packetId);
  write32(output, n, 0xFFFD040CUL); write32(output, n, static_cast<uint32_t>(role));
  write32(output, n, 0x00000384UL); write32(output, n, unixTime); write32(output, n, 0);
  memcpy(output + n, encoded, sizeof(encoded)); n += sizeof(encoded); finish(output, n);
  return n;
}

size_t buildQueryL2(uint8_t* output, size_t capacity, uint16_t packetId,
                    uint16_t appSusyId, uint32_t appSerial, uint16_t inverterSusyId,
                    uint32_t inverterSerial, const Query& query) {
  if (!output || capacity < 48) return 0;
  size_t n = 0;
  header(output, n, 0x09, 0xA0, 0, inverterSusyId, inverterSerial, appSusyId, appSerial, packetId);
  write32(output, n, query.command); write32(output, n, query.first); write32(output, n, query.last);
  finish(output, n); return n;
}

DecodeResult validateLoginResponse(const uint8_t* l2, size_t length,
                                   uint16_t expectedPacketId, uint32_t expectedUnixTime,
                                   LoginResponse& response) {
  if (!l2 || length < 47) return DecodeResult::BufferTooSmall;
  if (l2[0] != 0x7E || read32(l2 + 1) != kSignature || l2[length - 1] != 0x7E)
    return DecodeResult::InvalidStructure;
  if (read16(l2 + length - 3) != fcs16(l2 + 1, length - 4)) return DecodeResult::InvalidFcs;
  if ((read16(l2 + 27) & 0x7FFFU) != (expectedPacketId & 0x7FFFU)) return DecodeResult::PacketIdMismatch;
  if (read32(l2 + 41) != expectedUnixTime) return DecodeResult::InvalidStructure;
  LoginResponse candidate{};
  candidate.status = read16(l2 + 23);  // 0x0100 denotes invalid password.
  candidate.inverterSusyId = read16(l2 + 15);
  candidate.inverterSerial = read32(l2 + 17);
  response = candidate;
  return candidate.status == 0 ? DecodeResult::Ok : DecodeResult::InvalidValue;
}

DecodeResult decodeMeasurementResponse(const uint8_t* l2, size_t length,
                                       uint16_t expectedPacketId, Measurements& output) {
  if (!l2 || length < 29) return DecodeResult::BufferTooSmall;
  if (l2[0] != 0x7E || read32(l2 + 1) != kSignature || l2[length - 1] != 0x7E)
    return DecodeResult::InvalidStructure;
  if (read16(l2 + length - 3) != fcs16(l2 + 1, length - 4)) return DecodeResult::InvalidFcs;
  if ((read16(l2 + 27) & 0x7FFFU) != (expectedPacketId & 0x7FFFU)) return DecodeResult::PacketIdMismatch;
  if (read16(l2 + 23) != 0) return DecodeResult::DeviceError;
  if (length < 44) return DecodeResult::BufferTooSmall;
  if (length <= 44) return DecodeResult::NoData;
  const uint32_t first = read32(l2 + 33), last = read32(l2 + 37);
  const uint32_t count = last >= first ? (last - first + 1U) : 0;
  const uint8_t words = l2[5];
  if (words < 9) return DecodeResult::InvalidStructure;
  const size_t body = 4U * static_cast<size_t>(words - 9U);
  // Complete unescaped Data2+ length: 41-byte header + records + FCS + delimiter.
  if (length != body + 44U) return DecodeResult::InvalidStructure;
  if (count == 0 || body == 0) return DecodeResult::NoData;
  if (body % count != 0) return DecodeResult::InvalidStructure;
  const size_t recordSize = body / count;
  if (recordSize != 16 && recordSize < 20) return DecodeResult::UnsupportedRecord;
  Measurements candidate = output;
  bool recognized = false;
  for (size_t pos = 41; pos + recordSize <= length - 3; pos += recordSize) {
    const uint32_t code = read32(l2 + pos);
    const uint16_t lri = uint16_t((code >> 8) & 0xFFFFU);
    const uint8_t recordClass = uint8_t(code);
    candidate.returnedLri = lri;
    candidate.recordType = uint8_t(code >> 24);
    candidate.recordSize = static_cast<uint16_t>(recordSize);
    candidate.timestamp = read32(l2 + pos + 4);
    RawField* field = nullptr;
    switch (lri) {
      case kLriAcPowerL1: field = &candidate.acPower1; break;
      case kLriAcVoltageL1: field = &candidate.acVoltage1; break;
      case kLriAcCurrentL1Legacy:
      case kLriAcCurrentL1: field = &candidate.acCurrent1; break;
      case kLriGridFrequency: field = &candidate.gridFrequency; break;
      case kLriDcPower: field = &candidate.dcPower1; break;
      case kLriDcVoltage: field = &candidate.dcVoltage1; break;
      case kLriDcCurrent: field = &candidate.dcCurrent1; break;
      case kLriTotalEnergy: field = &candidate.totalEnergy; break;
      case kLriTodayEnergy: field = &candidate.todayEnergy; break;
      case kLriTemperature: field = &candidate.temperature; break;
      case kLriInverterStatus: field = &candidate.inverterStatus; break;
      case kLriGridRelay: field = &candidate.gridRelay; break;
      case kLriOperatingTime: field = &candidate.operatingTime; break;
      case kLriFeedInTime: field = &candidate.feedInTime; break;
      case kLriInverterClass: field = &candidate.inverterClass; break;
      case kLriInverterType: field = &candidate.inverterType; break;
      case kLriSoftwareVersion: field = &candidate.softwareVersion; break;
      default: break;
    }
    if (lri == kLriInverterName && recordSize >= 9) {
      const size_t available = recordSize - 8;
      const size_t copy = available < sizeof(candidate.inverterName) - 1
                            ? available : sizeof(candidate.inverterName) - 1;
      memcpy(candidate.inverterName, l2 + pos + 8, copy);
      candidate.inverterName[copy] = 0;
      candidate.inverterNameState = candidate.inverterName[0] ? FieldState::Valid : FieldState::Unavailable;
      candidate.inverterNameTimestamp = candidate.timestamp;
      recognized = true;
      continue;
    }
    if (!field && lri != kLriAcPower) continue;
    RawField local{};
    local.timestamp = candidate.timestamp;
    local.recordType = candidate.recordType;
    local.recordClass = recordClass;
    local.recordSize = static_cast<uint16_t>(recordSize);
    const bool attribute = lri == kLriInverterStatus || lri == kLriGridRelay ||
                           lri == kLriInverterClass || lri == kLriInverterType;
    if (attribute) {
      local.state = FieldState::Unavailable;
      for (size_t at = 8; at + 4 <= recordSize; at += 4) {
        const uint32_t value = read32(l2 + pos + at);
        const uint32_t tag = value & 0x00FFFFFFUL;
        if (tag == 0x00FFFFFEUL) break;
        if ((value >> 24) == 1) { local.raw = tag; local.state = FieldState::Valid; break; }
      }
    } else if (recordSize == 16) {
      local.raw = read64(l2 + pos + 8);
      local.state = (local.raw == kInvalid64A || local.raw == kInvalid64B)
                      ? FieldState::Unavailable : FieldState::Valid;
    } else if (recordSize >= 20) {
      local.raw = read32(l2 + pos + 16);
      if (lri == kLriSoftwareVersion && recordSize >= 28) local.raw = read32(l2 + pos + 24);
      local.state = (local.raw == kInvalid32A || local.raw == kInvalid32B)
                      ? FieldState::Unavailable : FieldState::Valid;
    } else {
      continue;
    }
    recognized = true;
    candidate.rawValue = local.raw;
    if (lri == kLriAcPower) {
      candidate.acTotalPower = local;
      if (local.state == FieldState::Valid) {
        candidate.acPowerW = static_cast<uint32_t>(local.raw);
        candidate.acPowerValid = true;
      }
    } else if (field && (field->state != FieldState::Valid || recordClass == 1)) {
      *field = local;
    }
    if (lri == kLriTotalEnergy && local.state == FieldState::Valid) {
      candidate.totalEnergyWh = local.raw; candidate.totalEnergyValid = true;
    } else if (lri == kLriAcVoltageL1 && local.state == FieldState::Valid) {
      candidate.acVoltageV = local.raw / 100.0f; candidate.acVoltageValid = true;
    } else if ((lri == kLriAcCurrentL1Legacy || lri == kLriAcCurrentL1) &&
               local.state == FieldState::Valid) {
      candidate.acCurrentA = local.raw / 1000.0f; candidate.acCurrentValid = true;
    }
  }
  candidate.valid = candidate.acPowerValid || candidate.totalEnergyValid || candidate.acVoltageValid ||
                    candidate.acCurrentValid || recognized;
  if (!recognized) return DecodeResult::UnsupportedRecord;
  output = candidate;
  return DecodeResult::Ok;
}

}  // namespace SmaPhase3
