#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../firmware/SmaBluetoothDiscovery/SmaPhase3Protocol.h"

namespace {
int failures = 0;
void check(bool condition, const char* name) {
  std::printf("%s %s\n", condition ? "PASS" : "FAIL", name);
  if (!condition) ++failures;
}
uint32_t r32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }
void w16(uint8_t* p, size_t at, uint16_t v) { p[at] = uint8_t(v); p[at + 1] = uint8_t(v >> 8); }
void w32(uint8_t* p, size_t at, uint32_t v) {
  p[at] = uint8_t(v); p[at + 1] = uint8_t(v >> 8); p[at + 2] = uint8_t(v >> 16); p[at + 3] = uint8_t(v >> 24);
}
size_t synthetic32(uint8_t* frame, uint16_t packet, uint16_t lri, uint32_t raw) {
  std::memset(frame, 0, 64); frame[0] = 0x7E; w32(frame, 1, 0x656003FF); frame[5] = 14;
  w16(frame, 23, 0); w16(frame, 27, packet | 0x8000); w32(frame, 33, 0); w32(frame, 37, 0);
  w32(frame, 41, (uint32_t(0x40) << 24) | (uint32_t(lri) << 8)); w32(frame, 45, 123456);
  w32(frame, 57, raw);
  const size_t length = 64; w16(frame, length - 3, SmaPhase3::fcs16(frame + 1, length - 4)); frame[length - 1] = 0x7E;
  return length;
}
size_t synthetic64(uint8_t* frame, uint16_t packet, uint16_t lri, uint64_t raw) {
  std::memset(frame, 0, 60); frame[0] = 0x7E; w32(frame, 1, 0x656003FF); frame[5] = 13;
  w16(frame, 23, 0); w16(frame, 27, packet | 0x8000); w32(frame, 33, 0); w32(frame, 37, 0);
  w32(frame, 41, uint32_t(lri) << 8); w32(frame, 45, 123456); w32(frame, 49, uint32_t(raw)); w32(frame, 53, uint32_t(raw >> 32));
  const size_t length = 60; w16(frame, length - 3, SmaPhase3::fcs16(frame + 1, length - 4)); frame[length - 1] = 0x7E;
  return length;
}
size_t syntheticString(uint8_t* frame, uint16_t packet, uint16_t lri, const char* value) {
  std::memset(frame, 0, 84); frame[0] = 0x7E; w32(frame, 1, 0x656003FF); frame[5] = 19;
  w16(frame, 23, 0); w16(frame, 27, packet | 0x8000); w32(frame, 33, 0); w32(frame, 37, 0);
  w32(frame, 41, (uint32_t(0x10) << 24) | (uint32_t(lri) << 8)); w32(frame, 45, 123456);
  std::strncpy(reinterpret_cast<char*>(frame + 49), value, 31);
  const size_t length = 84; w16(frame, length - 3, SmaPhase3::fcs16(frame + 1, length - 4)); frame[length - 1] = 0x7E;
  return length;
}
size_t syntheticAttribute(uint8_t* frame, uint16_t packet, uint16_t lri, uint32_t tag) {
  std::memset(frame, 0, 84); frame[0] = 0x7E; w32(frame, 1, 0x656003FF); frame[5] = 19;
  w16(frame, 23, 0); w16(frame, 27, packet | 0x8000); w32(frame, 33, 0); w32(frame, 37, 0);
  w32(frame, 41, (uint32_t(0x08) << 24) | (uint32_t(lri) << 8)); w32(frame, 45, 123456);
  w32(frame, 49, 0x01000000UL | tag); w32(frame, 53, 0x01FFFFFEUL);
  const size_t length = 84; w16(frame, length - 3, SmaPhase3::fcs16(frame + 1, length - 4)); frame[length - 1] = 0x7E;
  return length;
}
}

int main() {
  uint8_t encoded[12]{};
  check(SmaPhase3::encodePassword("0000", SmaPhase3::UserRole::User, encoded) &&
        encoded[0] == 0xB8 && encoded[1] == 0xB8 && encoded[2] == 0xB8 && encoded[3] == 0xB8 &&
        encoded[4] == 0x88 && encoded[11] == 0x88,
        "user password encoding and padding");
  check(!SmaPhase3::encodePassword("1234567890123", SmaPhase3::UserRole::User, encoded),
        "overlong password rejected");

  uint8_t request[96]{};
  constexpr uint32_t testAppSerial = 900000123UL;
  constexpr uint32_t testUnixTime = 1767225600UL;  // 2026-01-01 UTC
  const size_t loginLength = SmaPhase3::buildLoginL2(request, sizeof(request), 9, 125, testAppSerial,
      SmaPhase3::UserRole::User, testUnixTime, "0000");
  check(loginLength == 64 && r32(request + 29) == 0xFFFD040C && r32(request + 33) == 7 &&
        r32(request + 17) >= 900000000UL && r32(request + 17) <= 999999999UL &&
        r32(request + 37) == 900 && r32(request + 41) == testUnixTime,
        "login request fields");
  const size_t queryLength = SmaPhase3::buildQueryL2(request, sizeof(request), 10, 125, 1, 123, 456,
      SmaPhase3::kAcPowerQuery);
  check(queryLength == 44 && r32(request + 29) == 0x51000200 && r32(request + 33) == 0x00263F00 &&
        r32(request + 37) == 0x00263FFF, "measurement query construction");

  uint8_t loginResponse[48]{}; loginResponse[0] = 0x7E; w32(loginResponse, 1, 0x656003FF);
  w16(loginResponse, 15, 123); w32(loginResponse, 17, 456); w16(loginResponse, 23, 0);
  w16(loginResponse, 27, 9 | 0x8000); w32(loginResponse, 41, 0x12345678);
  w16(loginResponse, 45, SmaPhase3::fcs16(loginResponse + 1, 44)); loginResponse[47] = 0x7E;
  SmaPhase3::LoginResponse login{};
  check(SmaPhase3::validateLoginResponse(loginResponse, sizeof(loginResponse), 9, 0x12345678, login) ==
        SmaPhase3::DecodeResult::Ok && login.inverterSusyId == 123 && login.inverterSerial == 456,
        "login response validation");

  uint8_t frame[128]{}; SmaPhase3::Measurements values{};
  size_t length = synthetic32(frame, 11, SmaPhase3::kLriAcVoltageL1, 23045);
  check(SmaPhase3::decodeMeasurementResponse(frame, length, 11, values) == SmaPhase3::DecodeResult::Ok &&
        values.acVoltageValid && std::fabs(values.acVoltageV - 230.45f) < 0.01f,
        "valid response and voltage scaling");
  frame[20] ^= 1;
  check(SmaPhase3::decodeMeasurementResponse(frame, length, 11, values) == SmaPhase3::DecodeResult::InvalidFcs,
        "bad FCS rejected");
  check(SmaPhase3::decodeMeasurementResponse(frame, 20, 11, values) == SmaPhase3::DecodeResult::BufferTooSmall,
        "truncated frame rejected");
  length = synthetic32(frame, 12, SmaPhase3::kLriAcPower, 0xFFFFFFFF);
  check(SmaPhase3::decodeMeasurementResponse(frame, length, 12, values) == SmaPhase3::DecodeResult::Ok &&
        values.acTotalPower.state == SmaPhase3::FieldState::Unavailable && !values.acPowerValid,
        "invalid 32-bit sentinel preserved as unavailable");
  values = {}; length = synthetic32(frame, 12, SmaPhase3::kLriAcPower, 0);
  check(SmaPhase3::decodeMeasurementResponse(frame, length, 12, values) == SmaPhase3::DecodeResult::Ok &&
        values.acTotalPower.state == SmaPhase3::FieldState::Valid && values.acPowerValid && values.acPowerW == 0,
        "genuine numeric zero remains valid");
  length = synthetic64(frame, 13, SmaPhase3::kLriTotalEnergy, UINT64_C(123456789));
  check(SmaPhase3::decodeMeasurementResponse(frame, length, 13, values) == SmaPhase3::DecodeResult::Ok &&
        values.totalEnergyValid && values.totalEnergyWh == UINT64_C(123456789),
        "64-bit total energy decoding");
  values = {}; length = syntheticString(frame, 14, SmaPhase3::kLriInverterName, "Sunny Boy");
  check(SmaPhase3::decodeMeasurementResponse(frame, length, 14, values) == SmaPhase3::DecodeResult::Ok &&
        values.inverterNameState == SmaPhase3::FieldState::Valid && !std::strcmp(values.inverterName, "Sunny Boy"),
        "bounded inverter name decoding");
  values = {}; length = syntheticAttribute(frame, 15, SmaPhase3::kLriInverterStatus, 307);
  check(SmaPhase3::decodeMeasurementResponse(frame, length, 15, values) == SmaPhase3::DecodeResult::Ok &&
        values.inverterStatus.state == SmaPhase3::FieldState::Valid && values.inverterStatus.raw == 307,
        "selected attribute tag decoding");
  return failures ? 1 : 0;
}
