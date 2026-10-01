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
  if (!l2 || length < 60) return DecodeResult::BufferTooSmall;
  if (l2[0] != 0x7E || read32(l2 + 1) != kSignature || l2[length - 1] != 0x7E)
    return DecodeResult::InvalidStructure;
  if (read16(l2 + length - 3) != fcs16(l2 + 1, length - 4)) return DecodeResult::InvalidFcs;
  if ((read16(l2 + 27) & 0x7FFFU) != (expectedPacketId & 0x7FFFU)) return DecodeResult::PacketIdMismatch;
  if (read16(l2 + 23) != 0) return DecodeResult::InvalidStructure;
  const uint32_t first = read32(l2 + 33), last = read32(l2 + 37);
  const uint32_t count = last >= first ? (last - first + 1U) : 0;
  const size_t body = length - 44;  // 41-byte header, FCS and delimiter.
  if (count == 0 || body == 0 || body % count != 0) return DecodeResult::InvalidStructure;
  const size_t recordSize = body / count;
  if (recordSize != 16 && recordSize < 20) return DecodeResult::UnsupportedRecord;
  Measurements candidate{};
  for (size_t pos = 41; pos + recordSize <= length - 3; pos += recordSize) {
    const uint32_t code = read32(l2 + pos);
    const uint16_t lri = uint16_t((code >> 8) & 0xFFFFU);
    candidate.returnedLri = lri;
    candidate.recordType = uint8_t(code >> 24);
    candidate.recordSize = static_cast<uint16_t>(recordSize);
    candidate.timestamp = read32(l2 + pos + 4);
    if (recordSize == 16) {
      const uint64_t raw = read64(l2 + pos + 8);
      if (raw == kInvalid64A || raw == kInvalid64B) return DecodeResult::InvalidValue;
      candidate.rawValue = raw;
      if (lri == kLriTotalEnergy) { candidate.totalEnergyWh = raw; candidate.totalEnergyValid = true; }
    } else {
      const uint32_t raw = read32(l2 + pos + 16);
      if (raw == kInvalid32A || raw == kInvalid32B) return DecodeResult::InvalidValue;
      candidate.rawValue = raw;
      if (lri == kLriAcPower) { candidate.acPowerW = raw; candidate.acPowerValid = true; }
      else if (lri == kLriAcVoltageL1) { candidate.acVoltageV = raw / 100.0f; candidate.acVoltageValid = true; }
      else if (lri == kLriAcCurrentL1Legacy || lri == kLriAcCurrentL1) { candidate.acCurrentA = raw / 1000.0f; candidate.acCurrentValid = true; }
    }
  }
  candidate.valid = candidate.acPowerValid || candidate.totalEnergyValid || candidate.acVoltageValid || candidate.acCurrentValid;
  if (!candidate.valid) return DecodeResult::UnsupportedRecord;
  output = candidate;
  return DecodeResult::Ok;
}

}  // namespace SmaPhase3
