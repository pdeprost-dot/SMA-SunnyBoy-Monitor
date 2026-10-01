# Configuration and Web UI V3

The firmware is installable without recompilation after the initial generic
flash. Installation values are stored in the ESP32 `Preferences` NVS namespace
and edited through the framework-free Web UI:

- **Dashboard**: live three-inverter operational view;
- **Inverters**: enable flag, friendly name, Bluetooth MAC, serial and SMA USER
  password per inverter;
- **Network**: Wi-Fi scan/selection, STA credential and configurable fallback
  AP identity;
- **MQTT**: broker, port, username, password and topic prefix;
- **System**: plant name, location, human-readable timezone, maintenance and OTA
  password;
- **Diagnostics**: scheduler, heap, Bluetooth, MQTT, OTA and bounded logs.

Passwords are write-only. APIs expose only `passwordConfigured` booleans. An
empty replacement field explicitly means **keep the configured secret**. Reveal
buttons act only on newly typed browser values.

Existing settings were migrated into NVS before the generic firmware was
installed. The generic build contains no installation identity or SMA
credential and starts unconfigured when NVS is empty.

`Europe/Brussels` maps internally to
`CET-1CEST,M3.5.0,M10.5.0/3`; `UTC` maps to `UTC0`. The System page exposes the
current local time, UTC offset, DST state, NTP state, sunrise and sunset.

Browser firmware upload accepts an ESP32 `.bin` at `/api/firmware`, protected
with HTTP Basic user `admin` and the configured OTA password. It uses the
Arduino Update OTA partition and reboots only after successful finalization.
ArduinoOTA remains available unchanged.

Maintenance suspends planned SMA acquisitions while leaving Wi-Fi, Web, MQTT
and OTA available. Network changes and OTA-password replacements require a
reboot. All SMA scheduler, two-attempt limit, network-quiesce Bluetooth
lifecycle, retained MQTT and zero/null rules remain unchanged.
