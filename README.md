# SMA SunnyBoy Monitor

Firmware expérimental pour interroger trois onduleurs SMA Sunny Boy
SB2500HF-30 depuis un ESP32 classique en Bluetooth Classic.

Le projet avance par jalons matériels indépendants. Une compilation réussie
est notée `BUILD PASS`. Un jalon n'est noté `TEST PASS` qu'après un essai réel
sur le matériel.

## Périmètre actuel

Phase 1 uniquement : inquiry Bluetooth Classic et détection des trois adresses
connues.

| Onduleur | Adresse Bluetooth | Numéro de série attendu |
|---|---|---:|
| SMA #1 | `02:00:00:00:00:01` | 1000000001 |
| SMA #2 | `02:00:00:00:00:02` | 1000000002 |
| SMA #3 | `02:00:00:00:00:03` | 1000000003 |

La Phase 1 n'établit aucune connexion avec les onduleurs et n'implémente ni
login SMA, ni Data2+, ni mesure, ni Wi-Fi, MQTT, Web, écran ou ProgHard Link.
PZEM et RS485 sont hors périmètre.

## Matériel et environnement

- ESP32-WROOM-32 ou autre ESP32 classique avec Bluetooth BR/EDR ;
- Arduino CLI ;
- cœur `esp32:esp32` 3.3.11 ;
- cible `esp32:esp32:esp32` (ESP32 Dev Module) ;
- moniteur série à 115200 bauds.

Les ESP32-C3, C6, S2 et S3 ne conviennent pas à ce POC Bluetooth Classic.

## Compiler

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/SmaBluetoothDiscovery
```

Ne pas réutiliser un ancien port COM. Vérifier la carte présente avec :

```powershell
arduino-cli board list
```

La commande de téléversement ne doit être lancée qu'après identification de
la carte et du port :

```powershell
arduino-cli upload --fqbn esp32:esp32:esp32 -p COMx firmware/SmaBluetoothDiscovery
```

## Documentation

- [État du projet](docs/PROJECT_STATE.md)
- [Audit SMA de Phase 0](docs/phase-0-audit.md)
- [Protocole de test Phase 1](docs/phase-1-test.md)

