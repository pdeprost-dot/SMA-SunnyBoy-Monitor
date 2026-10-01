#include "ProductConfiguration.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace ProductConfig {
namespace {
constexpr double kPi = 3.14159265358979323846;
double radians(double degrees) { return degrees * kPi / 180.0; }
double degrees(double radiansValue) { return radiansValue * 180.0 / kPi; }
double normalize(double value, double limit) {
  while (value < 0) value += limit;
  while (value >= limit) value -= limit;
  return value;
}
int64_t daysFromCivil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(year - era * 400);
  const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097LL + static_cast<int>(doe) - 719468LL;
}
bool solarEvent(const tm& localDate, double latitude, double longitude, bool rise,
                time_t& result) {
  const int day = localDate.tm_yday + 1;
  const double lngHour = longitude / 15.0;
  const double t = day + ((rise ? 6.0 : 18.0) - lngHour) / 24.0;
  const double meanAnomaly = 0.9856 * t - 3.289;
  double trueLongitude = meanAnomaly + 1.916 * sin(radians(meanAnomaly)) +
                         0.020 * sin(2 * radians(meanAnomaly)) + 282.634;
  trueLongitude = normalize(trueLongitude, 360.0);
  double rightAscension = degrees(atan(0.91764 * tan(radians(trueLongitude))));
  rightAscension = normalize(rightAscension, 360.0);
  rightAscension += floor(trueLongitude / 90.0) * 90.0 - floor(rightAscension / 90.0) * 90.0;
  rightAscension /= 15.0;
  const double sinDec = 0.39782 * sin(radians(trueLongitude));
  const double cosDec = cos(asin(sinDec));
  const double cosHour = (cos(radians(90.833)) - sinDec * sin(radians(latitude))) /
                         (cosDec * cos(radians(latitude)));
  if (cosHour < -1.0 || cosHour > 1.0) return false;
  double hour = rise ? 360.0 - degrees(acos(cosHour)) : degrees(acos(cosHour));
  hour /= 15.0;
  const double utcHours = normalize(hour + rightAscension - 0.06571 * t - 6.622 - lngHour, 24.0);
  const time_t utcMidnight = static_cast<time_t>(daysFromCivil(localDate.tm_year + 1900,
      static_cast<unsigned>(localDate.tm_mon + 1), static_cast<unsigned>(localDate.tm_mday)) * 86400LL);
  result = utcMidnight + static_cast<time_t>(utcHours * 3600.0 + 0.5);
  return true;
}
}

bool validTopicPrefix(const char* value) {
  if (!value) return false;
  const size_t n = strnlen(value, kTopicPrefixCapacity);
  if (n == 0 || n >= kTopicPrefixCapacity || value[0] == '/' || value[n - 1] == '/') return false;
  for (size_t i = 0; i < n; ++i)
    if (value[i] == '+' || value[i] == '#' || value[i] == '/' || static_cast<unsigned char>(value[i]) < 0x20)
      return false;
  return true;
}

bool makeTopic(const char* prefix, size_t slot, char* output, size_t capacity) {
  if (!validTopicPrefix(prefix) || slot >= kInverterCount || !output || capacity == 0) return false;
  const int n = snprintf(output, capacity, "%s/inv%u", prefix, static_cast<unsigned>(slot + 1));
  return n > 0 && static_cast<size_t>(n) < capacity;
}

bool validSerial(uint32_t value) { return value > 0; }

bool validMac(const char* value) {
  if (!value || strlen(value) != 17) return false;
  for (size_t i = 0; i < 17; ++i) {
    if ((i + 1) % 3 == 0) { if (value[i] != ':') return false; }
    else if (!((value[i] >= '0' && value[i] <= '9') || (value[i] >= 'a' && value[i] <= 'f') ||
               (value[i] >= 'A' && value[i] <= 'F'))) return false;
  }
  return true;
}

bool validWpaPassword(const char* value) {
  if (!value) return false;
  const size_t n = strlen(value);
  return n >= 8 && n <= 63;
}
bool validOtaPassword(const char* value) { return value && strlen(value) >= 8 && strlen(value) <= 63; }
bool validLatitude(double value) { return std::isfinite(value) && value >= -90.0 && value <= 90.0; }
bool validLongitude(double value) { return std::isfinite(value) && value >= -180.0 && value <= 180.0; }

bool formatLocalTime(time_t value, char* output, size_t capacity) {
  if (!output || capacity < 20 || value <= 0) return false;
  tm local{};
#if defined(_WIN32)
  if (localtime_s(&local, &value)) return false;
#else
  if (!localtime_r(&value, &local)) return false;
#endif
  return strftime(output, capacity, "%d/%m/%Y %H:%M:%S", &local) == 19;
}

bool calculateSunTimes(time_t now, double latitude, double longitude,
                       time_t& sunrise, time_t& sunset) {
  if (now <= 0 || !validLatitude(latitude) || !validLongitude(longitude)) return false;
  tm date{};
#if defined(_WIN32)
  if (localtime_s(&date, &now)) return false;
#else
  if (!localtime_r(&now, &date)) return false;
#endif
  return solarEvent(date, latitude, longitude, true, sunrise) &&
         solarEvent(date, latitude, longitude, false, sunset);
}

const char* webValueAvailable(bool available) { return available ? "value" : "--"; }
}  // namespace ProductConfig
