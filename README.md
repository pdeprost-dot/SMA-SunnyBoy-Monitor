# SMA SunnyBoy Monitor

Firmware autonome pour ESP32 classique qui interroge jusqu'à trois onduleurs
SMA Sunny Boy par Bluetooth Classic, affiche les mesures dans une interface Web
et publie un snapshot MQTT retenu par onduleur.

Version candidate actuelle : **1.0.0-rc.1**. Aucune licence globale n'est
encore attribuée au dépôt ; la provenance du protocole est documentée dans
[`docs/PROTOCOL_PROVENANCE.md`](docs/PROTOCOL_PROVENANCE.md).

## Fonctions

- découverte Bluetooth Classic, affectation et test d'identité des onduleurs ;
- configuration persistante NVS par interface Web ;
- scheduler trois onduleurs, deux tentatives maximum et aucun rattrapage ;
- acquisition SMA Data2+ : identité, état, énergie, mesures AC/DC, température
  et compteurs horaires ;
- snapshots MQTT QoS 0 retenus, avec distinction entre zéro réel et donnée
  indisponible ;
- mise à jour par ArduinoOTA et navigateur ;
- mode maintenance et diagnostics bornés.

L'ESP32 coupe temporairement Wi-Fi et les services réseau pendant chaque cycle
Bluetooth afin de préserver la mémoire interne requise par RFCOMM. Les services
sont restaurés avant la publication MQTT.

## Matériel supporté

Validé sur :

- ESP32-WROOM-32 / ESP32 classique avec Bluetooth BR/EDR ;
- trois SMA Sunny Boy SB 2500HF-30 réels ;
- Arduino-ESP32 3.3.11 ;
- partition `min_spiffs`.

ESP32-C3, C6, S2 et S3 ne fournissent pas le Bluetooth Classic requis. Les
autres familles SMA ne sont pas déclarées compatibles sans validation réelle.
PZEM et ProgHard Link ne font pas partie de cette release candidate.

## État de validation

- scan/affectation/test Bluetooth : **PASS** ;
- acquisition des trois SB 2500HF-30 : **PASS** ;
- Web et configuration persistante : **PASS** ;
- MQTT retenu : **PASS** ;
- ArduinoOTA et OTA navigateur : **PASS** ;
- stabilité nocturne : **PASS** ;
- transition nuit → matin : **PASS** ;
- transition vers la production matinale : **PASS** ;
- récupération réelle par point d'accès de secours : **EN ATTENTE**.

## Installation rapide

Voir [`docs/INSTALLATION.md`](docs/INSTALLATION.md) pour l'environnement exact,
le build propre et le flash USB. Après le premier démarrage, configurer Wi-Fi,
les trois onduleurs, MQTT, la localisation et les secrets depuis le Web.

Documentation :

- [configuration](docs/CONFIGURATION.md) ;
- [MQTT](docs/MQTT.md) ;
- [OTA et rollback](docs/OTA.md) ;
- [sécurité](SECURITY.md) ;
- [validation matérielle](docs/HARDWARE_VALIDATION.md) ;
- [provenance du protocole](docs/PROTOCOL_PROVENANCE.md) ;
- [documents historiques](docs/history/).

## Architecture mémoire

Chaque slot utilise un cycle indépendant :

```text
réseau OFF → Bluetooth begin → RFCOMM → session/login/requêtes SMA
→ RFCOMM close → Bluetooth end → réseau ON → publication MQTT
```

Une acquisition échouée conserve le dernier snapshot valide. Elle ne fabrique
ni zéro ni valeur indisponible et n'augmente pas automatiquement le nombre de
tentatives.

## Limites et sécurité

- interface Web en HTTP Basic sans TLS : utilisation sur LAN de confiance ;
- secrets stockés en NVS et visibles dans l'interface authentifiée ;
- même secret administrateur pour Web, ArduinoOTA et OTA navigateur ;
- firmware OTA non signé cryptographiquement ;
- signal Bluetooth SMA non acquis : `bt_signal_percent` reste `null` ;
- point d'accès de secours encore à valider sur matériel réel.

SMA et Sunny Boy sont des marques de SMA Solar Technology AG. Ce projet n'est
ni affilié, ni approuvé par SMA. SBFspot et NANODE SMA PV MONITOR ont servi de
références historiques et d'interopérabilité ; voir le document de provenance.
