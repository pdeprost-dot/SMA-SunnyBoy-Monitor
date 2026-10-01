# SMA SunnyBoy Monitor

Firmware ESP32 pour lire trois onduleurs SMA Sunny Boy SB 2500HF-30 en
Bluetooth Classic et publier un snapshot MQTT retenu par onduleur.

**État validé : COMPLETE DATASET + MQTT SNAPSHOT V1**

Jalon matériel : `c52523f` — 1 octobre 2026.

Le projet est encore un firmware autonome de validation. Son intégration dans
ProgHard Link et une éventuelle interface M5Stack Core2 ne sont pas réalisées.

## Matériel validé

- ESP32-WROOM-32 / ESP32 classique avec Bluetooth BR/EDR ;
- trois SMA Sunny Boy SB 2500HF-30 ;
- Arduino-ESP32 3.3.11 ;
- cible `esp32:esp32:esp32`, partition `min_spiffs` ;
- Wi-Fi pour Web, MQTT et OTA.

Les ESP32-C3, C6, S2 et S3 ne conviennent pas à ce firmware Bluetooth Classic.

## Architecture

```text
SMA Sunny Boy (x3)
        |
 Bluetooth Classic / RFCOMM / SMA Data2+
        |
 ESP32-WROOM-32
        |
      Wi-Fi
   +----+----+
   |    |    |
  Web  MQTT  OTA
```

La marge mémoire interne de l'ESP32 classique ne permet pas de maintenir tous
les services réseau et Bluetooth Classic avec une marge RFCOMM sûre. Chaque
acquisition utilise donc une fenêtre bornée :

1. arrêt propre de MQTT, OTA, mDNS, Web et Wi-Fi ;
2. démarrage de Bluetooth Classic ;
3. connexion RFCOMM, session SMA et login USER ;
4. lecture du dataset d'un seul onduleur ;
5. fermeture RFCOMM et `BluetoothSerial.end()` ;
6. restauration Wi-Fi, Web, mDNS, OTA et MQTT ;
7. publication MQTT après restauration réseau.

Chaque onduleur utilise un cycle Bluetooth indépendant. Le scheduler décale
les trois slots et autorise au maximum deux tentatives par slot, chacune avec
une pile Bluetooth fraîche. Il n'existe ni troisième essai, ni rattrapage
immédiat, ni dette d'acquisition. Un échec conserve le dernier snapshot valide.
OTA est prioritaire et le mode maintenance suspend les acquisitions.

## Reconstruction SMA L1 vers Data2+

Les réponses groupées peuvent couvrir plusieurs fragments Bluetooth L1. Le
firmware détecte le début Data2+, accumule les fragments dans un buffer fixe de
384 octets, conserve l'état d'échappement `0x7D`, puis décode uniquement après
la commande L1 terminale `0x0001`.

| Requête validée | Fragments L1 | Data2+ reconstruit |
|---|---:|---:|
| PACTot | 1 | 72 octets |
| GridFreq | 1 | 72 octets |
| DC U/I | 2 | 100 octets |
| AC power | 2 | 128 octets |
| AC U/I | 3 | 212 octets |

La longueur utile, le FCS et le packet ID sont contrôlés avant décodage. Les
overflows et échappements incomplets sont fatals. IAC1 accepte les deux LRIs
SMA observés, `0x4650` et `0x4653`.

## Données validées

- identité : série, nom, classe, type et version logicielle ;
- état : statut, relais réseau et température ;
- énergie : ETotal et EToday en kWh ;
- AC : PACTot, PAC1, UAC1, IAC1 et fréquence réseau ;
- DC : PDC1, UDC1 et IDC1 ;
- compteurs : temps de fonctionnement et temps d'injection en heures.

Le pourcentage de signal Bluetooth SMA reste optionnel et non validé. Il est
publié à `null` tant qu'il n'est pas acquis de manière fiable.

## MQTT Snapshot V1

Topics QoS 0 et retenus : `smaesp/inv1`, `smaesp/inv2`, `smaesp/inv3`.
La publication intervient après le retour réseau. Un échec MQTT ne remet pas en
cause l'acquisition SMA.

```json
{
  "timestamp": "01/10/2026 18:09:31",
  "serial": 1000000001,
  "name": "Sunny Boy",
  "class": "Solar Inverters",
  "type": "SB 2500HF-30",
  "sw_version": "02.10.18.R",
  "status": "Ok",
  "grid_relay": "Closed",
  "temperature_c": 37.17,
  "energy_total_kwh": 32723.654,
  "energy_today_kwh": 5.578,
  "ac_power_w": 42,
  "ac_power_l1_w": 41,
  "ac_voltage_l1_v": 238.61,
  "ac_current_l1_a": 0.172,
  "grid_frequency_hz": 49.96,
  "dc_power_w": 92,
  "dc_voltage_v": 356.41,
  "dc_current_a": 0.261,
  "operating_time_h": 62426.541,
  "feed_in_time_h": 59032.349,
  "bt_signal_percent": null,
  "data_valid": true
}
```

Un zéro SMA valide reste `0`. `UNAVAILABLE` et `NOT_YET_ACQUIRED` restent
distincts en mémoire mais deviennent `null` dans le JSON. Une acquisition
échouée ne remplace jamais le dernier snapshot valide. Le payload validé mesure
533 octets ; les buffers fixes JSON et MQTT mesurent 1024 et 1152 octets.

## Validation actuelle

- dataset AC/DC complet, reconstruction multi-fragments, FCS et packet ID : PASS ;
- scheduler multi-onduleurs et retries bornés : PASS ;
- restauration Wi-Fi/Web/MQTT/OTA : PASS ;
- snapshot MQTT retenu reçu depuis le broker : PASS ;
- intégrité heap : PASS ;
- **SUNSET / NIGHT TRANSITION: TEST IN PROGRESS — 01/10/2026**.

L'ESP est volontairement laissé sans interaction pendant cette observation.

## Configuration et compilation

Copier `firmware/SmaBluetoothDiscovery/SmaLocalConfig.example.h` vers
`SmaLocalConfig.h`, puis renseigner les identifiants locaux. Ce fichier et les
credentials restent ignorés par Git.

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs firmware/SmaBluetoothDiscovery
```

## Documentation

- [Dataset complet et MQTT Snapshot V1](docs/complete-dataset-mqtt-v1.md)
- [État du projet](docs/PROJECT_STATE.md)
- [Audit SMA Phase 0](docs/phase-0-audit.md)
- [Résultat Phase 1](docs/phase-1-result-final.md)
- [Premier échange Data2+ Phase 2](docs/phase-2-first-session-result.md)
- [Développement distant](docs/remote-development.md)

## Références et licence

[SBFspot](https://github.com/SBFspot/SBFspot) a servi de référence
comportementale et protocolaire, avec attribution ; son code n'est pas recopié
en bloc. Le dépôt ne contient actuellement aucun fichier de licence globale :
les conditions de redistribution restent à formaliser avant une release finale.
