# Phase 3 preparation — SMA login and initial measurements

Status: code/build preparation only. Nothing in this document has been deployed or validated on the SB2500HF-30.

## Sources and licensing

- Historical note: SBFspot was used as a behavioral/protocol reference under
  CC BY-NC-SA 3.0. The later provenance review does not make the older absolute
  "no source copied" claim; see `../PROTOCOL_PROVENANCE.md` for the evidence-based
  P/N/S/O/U classification.
- ESP32_SMA-Inverter-MQTT is MIT-licensed and corroborates the login fields, query ranges, record layout and scaling.
- ESP32_to_SMA_ESPHome corroborates the login sequence, but has no explicit license file; it was used only as a behavioral reference.

The implementation in `SmaPhase3Protocol.*` is a small, original, fixed-buffer implementation based on protocol facts. Synthetic test frames were created locally.

## Layering

`Bluetooth/RFCOMM -> existing SMA session/framing -> login -> queries -> decoder -> optional logout -> disconnect -> BT off`

The preparation module does not invoke the transport and is not wired into the existing state machine. Phase 2.5 lifecycle changes remain intact.

## Login

- Default role: USER (`0x00000007`); INSTALLER (`0x0000000A`) is implemented but not required by current evidence.
- Password: at most 12 bytes. Each byte is increased by `0x88` for USER or `0xBB` for INSTALLER; unused positions encode NUL with the same offset.
- Data2+ command: `0xFFFD040C`, timeout 900 seconds, Unix timestamp, zero word, encoded 12-byte password.
- Request: longwords `0x0E`, control `0xA0`, control2 `0x0100`, wildcard destination.
- Response must pass structure, FCS, packet-ID and status validation. Timestamp echo and inverter identity should additionally be checked when wired to the live parser.
- Logout command documented by the references is `0xFFFD010E`. Whether it is required before RFCOMM close on this HF inverter remains a hardware question.
- A real password belongs only in ignored `SmaPhase3LocalConfig.h`; the tracked example is fictitious.

## Initial query set

All multibyte numeric fields are little-endian.

| Measurement | Request command/range | LRI | Record/raw type | Scale | Invalid values | SB2500HF-30 confidence |
|---|---|---:|---|---|---|---|
| AC active power total | `51000200`, `00263F00..00263FFF` | `263F` | unsigned 32-bit in numeric record | 1 W/count | `80000000`, `FFFFFFFF` | Medium; generic SMA LRI, not yet observed on HF |
| Total produced energy | `54000200`, `00260100..002622FF` | `2601` | unsigned 64-bit, 16-byte record | 1 Wh/count (divide by 1000 for kWh) | `8000000000000000`, `FFFFFFFFFFFFFFFF` | Medium |
| AC voltage L1 | `51000200`, `00464800..004655FF` | `4648` | unsigned 32-bit in numeric record | 0.01 V/count | 32-bit sentinels above | Medium; single-phase L1 expected |
| AC current L1 | same voltage/current query | `4650` legacy or `4653` alternate | unsigned 32-bit in numeric record | 0.001 A/count | 32-bit sentinels above | Medium-low; both observed SMA variants accepted |

Optional, deliberately not implemented in the first decoder: DC power `251E` (`53800200`), DC voltage `451F` and current `4521`, frequency `4657`, inverter temperature `2377`. Reactive power Q is not assumed available.

The decoder validates buffer bounds, Data2+ signature/delimiters, FCS, response status, packet ID, derived record size and invalid sentinels. It uses no heap allocation. The future API representation is the fixed `Measurements` structure, with per-value validity flags, timestamp, W, Wh, V and A fields.

## SB2500HF-30 caveats

The Phase 2 26-byte level-1 frame `0x0005` is a model-specific observation and must not be used to infer Data2+ record sizes. The login envelope appears generic, but login acceptance, current LRI variant, supported query ranges, record sizes and logout behavior require one carefully logged hardware transaction. TL-family compatibility is not proof of HF compatibility.
