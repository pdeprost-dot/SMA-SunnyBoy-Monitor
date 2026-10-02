# Configuration and Web UI V3 (historique de conception)

Le guide utilisateur courant est [`CONFIGURATION.md`](CONFIGURATION.md). Ce
document conserve les décisions de conception V3.

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

The local configuration UI and its secret-bearing API require HTTP Basic
authentication using the OTA administrator credential. The API returns stored
passwords only to that authenticated UI.
Every password input is masked by default and its **Afficher/Masquer** button
only changes the browser presentation. Values can be edited directly and are
persisted to NVS on Save. Secrets are never written to Serial, diagnostics,
MQTT, tracked source files or documentation.

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

The same administrator/OTA password protects three operations: Web UI login,
ArduinoOTA and browser `.bin` installation. The second password field beside
the file upload is an authorization confirmation, not a separate credential.

Maintenance suspends planned SMA acquisitions while leaving Wi-Fi, Web, MQTT
and OTA available. Network changes and OTA-password replacements require a
reboot. All SMA scheduler, two-attempt limit, network-quiesce Bluetooth
lifecycle, retained MQTT and zero/null rules remain unchanged.

## Inverter installation

The Inverters page provides a bounded Bluetooth Classic scan. It temporarily
suspends scheduling, quiesces network services and Wi-Fi, performs one
continuous inquiry, stops Bluetooth, restores Wi-Fi/Web/OTA/MQTT and resumes
the scheduler. Every discovered device is shown; only an exact match with an
already configured address is labelled as configured SMA equipment.

A discovered address can be assigned to INV1, INV2 or INV3 without changing
the slot's enabled flag, friendly name or SMA USER password. MAC and detected
serial/type/software version are read-only technical information. **Tester la
connexion** uses the validated isolated SMA lifecycle to verify identity and
restores normal operation automatically.
