# Inventaire tiers

Cet inventaire décrit les dépendances utilisées ou consultées. Il n'attribue
pas de licence globale à ce dépôt.

## Dépendances de compilation et d'exécution

| Composant | Usage | Licence annoncée en amont |
|---|---|---|
| Arduino-ESP32 3.3.11 | Core ESP32, Wi-Fi, BluetoothSerial, WebServer, ArduinoOTA, Update, NVS | LGPL-2.1-or-later pour le core Arduino ; composants ESP-IDF sous leurs licences respectives, principalement Apache-2.0 |
| PubSubClient 2.8 | Client MQTT | MIT |
| Espressif ESP-IDF inclus par Arduino-ESP32 | Pile radio, FreeRTOS, lwIP et pilotes | Inventaire multi-licences fourni par Espressif |
| Python 3 + paho-mqtt | Outil optionnel `overnight_capture.py` | PSF-2.0 et EPL-2.0 / EDL-1.0 |

Le firmware n'embarque aucun framework Web, CDN, police, image ou bibliothèque
JavaScript externe : HTML, CSS et JavaScript sont propres au projet et intégrés
dans le sketch.

## Références de protocole non embarquées

| Projet | Usage | Licence annoncée en amont |
|---|---|---|
| SBFspot, commit `b77f65047e9f4a136a09677838f6b4e5024862ee` | Référence de comportement et d'interopérabilité SMA | CC BY-NC-SA 3.0 dans la version étudiée |
| NANODE SMA PV MONITOR, Stuart Pittaway | Référence historique antérieure de la lignée du protocole | CC BY-NC-SA 3.0 dans les fichiers étudiés |

Leurs sources ne sont pas vendoriées dans ce dépôt. La comparaison détaillée,
les éléments d'expression potentiellement proches et les incertitudes sont dans
[`PROTOCOL_PROVENANCE.md`](PROTOCOL_PROVENANCE.md). Avant de choisir une licence
de distribution pour ce dépôt, cette provenance doit faire l'objet d'une revue
juridique ou d'une séparation/réécriture documentée des éléments concernés.
