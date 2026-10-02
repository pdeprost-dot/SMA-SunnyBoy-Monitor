# État du projet

## Release candidate

- branche : `feature/sma-memory-alignment-v1` ;
- version firmware : `1.0.0-rc.1` ;
- schéma MQTT : `2` ;
- cible : ESP32-WROOM-32, Arduino-ESP32 3.3.11, `min_spiffs` ;
- matériel de référence : trois SMA Sunny Boy SB 2500HF-30.

## Validé

- scan Bluetooth Classic, affectation et test d'identité ;
- RFCOMM canal 1, session Data2+, login USER et dataset complet ;
- scheduler avec lifecycle Bluetooth indépendant et deux tentatives maximum ;
- restauration Wi-Fi, Web, mDNS, MQTT et OTA ;
- snapshots MQTT retenus et sémantique zéro/null ;
- configuration Web/NVS, ArduinoOTA et OTA navigateur ;
- stabilité nocturne, transition nuit→matin et production matinale ;
- intégrité heap après les cycles réels.

## En attente avant v1.0.0

- décision documentée sur la licence/provenance ;
- test matériel réel de récupération par AP de secours ;
- soak final de release et validation du binaire exact ;
- inventaire final des licences des dépendances binaires.

PZEM, ProgHard Link et M5Stack Core2 sont hors périmètre de rc.1.
