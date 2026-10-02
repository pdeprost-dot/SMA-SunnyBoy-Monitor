# Phase 2.5 — OTA activation diagnostic

Status: **OTA TRANSFER PASS / OTA FINALIZATION UNCONFIRMED-FAILED**

Date: 2026-09-30

## Active baseline

The active firmware remains the known-good public Phase 2 baseline:

- source commit: `8ef42bdbffe4e72cabda55c5584e6b2038ab95b0`;
- source branch: `feature/sma-first-session-v1`;
- golden rollback image: `releases/phase-2-stable/SMA-SunnyBoy-Monitor_phase2-stable_8ef42bdb.bin`;
- golden SHA-256: `C6888A687C6CE3CC952CC1AE6A3ABA8211843DBCFDACEE8AE81B9EE73BC082C8`.

The unvalidated Phase 2.5 work remains preserved on
`feature/sma-memory-alignment-v1`. It must not be presented as a validated
firmware or rollback image.

## Observed OTA result

- Phase 2.5 build: PASS.
- OTA authentication: PASS.
- Client-side transfer: reached 100%.
- Final device acknowledgement `OK`: not confirmed.
- `ArduinoOTA.onEnd()` execution: not observed.
- `Update.end()` success: not confirmed.
- Boot-partition switch: not confirmed.
- Reboot: not observed.
- Device uptime: unchanged.
- Firmware after the attempt: existing Phase 2 baseline still active.

Therefore, 100% transfer must not be interpreted as installation success.
The failure is localized most likely between receipt of the final bytes and
successful OTA finalization, but the available evidence cannot distinguish an
incomplete device-side receive, integrity validation failure, image validation
failure, or boot-partition activation failure.

## Capacity check

Partition scheme: `min_spiffs`.

- `app0` (`ota_0`): `0x1E0000` bytes (`1,966,080` bytes).
- `app1` (`ota_1`): `0x1E0000` bytes (`1,966,080` bytes).
- Golden Phase 2 binary: `1,721,888` bytes.
- Phase 2.5 build binary: `1,724,704` bytes.
- Phase 2.5 image fits the OTA partition: YES.

## Deferred diagnostic

No additional OTA, reboot, USB intervention, firmware change, or SMA
transaction is authorized for this diagnostic now.

During the next otherwise-useful physical intervention on the ESP32, capture
Serial during one controlled OTA and retain the ArduinoOTA/Update evidence for:

- OTA start;
- `Update.begin()` and target partition;
- bytes written/progress;
- MD5/integrity result;
- `Update.end()` result and error text;
- `onEnd()` or `onError()`;
- boot-partition selection and reboot.

This capture is deferred and does not justify a dedicated physical
intervention.
