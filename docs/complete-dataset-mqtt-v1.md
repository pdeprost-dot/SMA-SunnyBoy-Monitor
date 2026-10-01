# Complete SMA dataset and MQTT snapshot V1

## Validated acquisition

The ESP32 reads each inverter in an independent Bluetooth lifecycle while the
network services are temporarily stopped. A successful lifecycle includes the
SMA session, USER login and the bounded dataset query sequence. Network
services are restored before MQTT publication. A failed acquisition preserves
the previous valid snapshot and never publishes fabricated replacement data.

The scheduler timing, maximum of two fresh-lifecycle attempts per slot,
maintenance mode and OTA priority remain independent of MQTT success.

## Bluetooth L1 to Data2+ reconstruction

Large SMA responses can span several Bluetooth L1 frames. Reconstruction uses
one fixed 384-byte buffer:

1. detect the Data2+ prefix `7E FF 03 60 65` in the first L1 payload;
2. accumulate subsequent L1 payloads from the selected inverter;
3. unescape `0x7D` sequences while accumulating;
4. finish only on the terminal L1 command `0x0001`;
5. validate the reconstructed Data2+ length, FCS and packet ID before decoding.

Record size is derived from the Data2+ word count and requested LRI count.
DWORD, QWORD, attribute and string records retain distinct decoding rules.
Both `0x4650` and `0x4653` are accepted as the L1 AC-current record. SMA
sentinels remain unavailable values and are never treated as measured zero.

Bounded diagnostics retain fragment counts, raw L1 byte counts, reconstructed
Data2+ packet counts and lengths, and the decode result for each query. Raw
frames are not published over MQTT.

Validated reconstruction sizes:

| Query | L1 fragments | Reconstructed Data2+ |
|---|---:|---:|
| PACTot | 1 | 72 bytes |
| GridFreq | 1 | 72 bytes |
| DC voltage/current | 2 | 100 bytes |
| AC power | 2 | 128 bytes |
| AC voltage/current | 3 | 212 bytes |

The former implementation decoded only the payload of the terminal L1 frame,
so grouped responses appeared artificially short. The validated accumulator
fixes this without changing the request ranges. SBFspot is the attributed
behavioral reference used to confirm reconstruction and record iteration.

## MQTT topics and delivery

One retained JSON document is published after each successful or partial
successful acquisition:

- `<prefix>/inv1`
- `<prefix>/inv2`
- `<prefix>/inv3`

The default prefix is `smaesp`. Publication uses the existing lightweight
PubSubClient QoS behavior and a retained flag. MQTT failure is non-fatal.

The JSON serializer uses a fixed 1024-byte payload buffer. PubSubClient is
configured with a 1152-byte packet buffer. No MAC address, credential or raw
diagnostic frame is included.

## Schema and units

| Field | Unit/source |
|---|---|
| `timestamp` | Host local time at successful acquisition, `DD/MM/YYYY HH:MM:SS` |
| `serial` | Configured and identity-validated SMA serial |
| `name`, `class`, `type`, `sw_version` | SMA identity records |
| `status`, `grid_relay` | SMA attribute records |
| `temperature_c` | degrees Celsius |
| `energy_total_kwh`, `energy_today_kwh` | kWh |
| `ac_power_w`, `ac_power_l1_w` | W |
| `ac_voltage_l1_v` | V |
| `ac_current_l1_a` | A |
| `grid_frequency_hz` | Hz |
| `dc_power_w` | W |
| `dc_voltage_v` | V |
| `dc_current_a` | A |
| `operating_time_h`, `feed_in_time_h` | h |
| `bt_signal_percent` | SMA network value; `null` until acquired |
| `data_valid` | Result of the completed acquisition |

Each measurement keeps an internal state: valid, unavailable, or not yet
acquired. Valid zero is serialized as numeric `0`. Both unavailable states are
serialized as JSON `null`. A complete acquisition failure does not overwrite
the last retained valid snapshot.

## Hardware validation status

- complete AC/DC target dataset: PASS;
- reconstructed length, FCS and packet ID: PASS;
- reconstruction overflow: none;
- autonomous three-inverter scheduler: PASS;
- bounded retries and network restoration: PASS;
- retained broker snapshot: PASS (validated example: 533 bytes);
- Wi-Fi, MQTT and OTA restoration: PASS;
- heap integrity: PASS;
- sunset/night transition: **TEST IN PROGRESS — 01/10/2026**.
