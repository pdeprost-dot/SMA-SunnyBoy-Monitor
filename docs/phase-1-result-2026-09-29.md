# Phase 1 — essai matériel du 2026-09-29

## Configuration

- Carte détectée : ESP32-D0WD révision 1.0, double cœur, Wi-Fi + BT.
- Port lors de l'essai : COM15.
- Flash : 4 MiB.
- Arduino-ESP32 : 3.3.11, ESP-IDF 5.5.5.
- Firmware : `SmaBluetoothDiscovery` au commit initial `82924fd`.
- Inquiry : Bluetooth Classic synchrone, 12 secondes par cycle.

## Build et flash

- Build : PASS.
- Flash : PASS, écriture vérifiée par hash.
- Programme : 1 061 436 octets sur 1 310 720 (80 %).
- RAM statique : 41 932 octets sur 327 680 (12 %).

## Résultat réel

Trois cycles complets ont été capturés :

| Cycle | Appareils Classic | SMA #1 | SMA #2 | SMA #3 |
|---:|---:|---:|---:|---:|
| 1 | 0 | 0 | 0 | 0 |
| 2 | 0 | 0 | 0 | 0 |
| 3 | 0 | 0 | 0 | 0 |

Les trois MAC SMA attendues n'ont pas été détectées. Aucun autre appareil
Bluetooth Classic n'a été signalé dans ces trois fenêtres.

## Mémoire observée

| Jalon | Heap libre | Minimum | Plus grand bloc interne |
|---|---:|---:|---:|
| Boot | 225 580 | 218 292 | 110 580 |
| Bluetooth prêt | 134 172 | 134 004 | 110 580 |
| Après cycle 1 | 133 028 | 126 380 | 110 580 |
| Après cycle 2 | 133 028 | 126 380 | 110 580 |
| Après cycle 3 | 133 028 | 126 380 | 110 580 |

Il n'y a pas de dérive visible sur cette courte séquence. Ce constat ne vaut
pas test d'endurance.

## Verdict

`BUILD PASS — FLASH PASS — HARDWARE DISCOVERY TEST NOT PASSED`

La Phase 1 reste ouverte. Le prochain essai doit être effectué lorsque les
onduleurs sont alimentés/actifs, avec l'ESP32 suffisamment proche et sans
modifier le firmware ni commencer la Phase 2.

