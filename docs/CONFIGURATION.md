# Configuration

La configuration persistante utilise le namespace NVS `sma-monitor`.

## Réseau

Configurer le SSID et le mot de passe STA, puis l'identité et le mot de passe du
point d'accès de secours. Un changement réseau nécessite un redémarrage.

## Onduleurs

Pour chaque slot INV1 à INV3 : activer le slot, choisir un nom convivial,
lancer le scan Bluetooth, affecter le périphérique et saisir le mot de passe
SMA USER. La MAC, la série et l'identité détectée sont des informations
techniques en lecture seule. Le test de connexion ne remplace pas la
configuration enregistrée de manière implicite.

## MQTT et système

Configurer broker, port, utilisateur, mot de passe et préfixe de topic. Le
système accepte un nom d'installation, latitude, longitude et fuseau horaire.
`Europe/Brussels` utilise les règles CET/CEST ; l'écran affiche heure locale,
offset UTC, DST, NTP, lever et coucher.

## Secrets et maintenance

Les champs secret sont masqués par défaut et peuvent être affichés ou modifiés
depuis l'interface authentifiée. Le secret administrateur est partagé par le
Web, ArduinoOTA et l'OTA navigateur. Le mode maintenance suspend le scheduler
SMA mais conserve Wi-Fi, Web, MQTT et OTA.
