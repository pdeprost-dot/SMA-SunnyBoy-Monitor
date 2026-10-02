# Installation et build reproductible

## Environnement validé

- Arduino CLI ;
- core `esp32:esp32` version **3.3.11** ;
- bibliothèque PubSubClient **2.8** ;
- carte ESP32-WROOM-32 / ESP32 classique ;
- FQBN `esp32:esp32:esp32:PartitionScheme=min_spiffs`.

```powershell
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.3.11
arduino-cli lib install "PubSubClient@2.8"
arduino-cli compile --clean `
  --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs `
  firmware/SmaBluetoothDiscovery
```

Le build générique ne contient aucun identifiant d'installation. Les fichiers
`SmaLocalConfig.h` et `SmaPhase3LocalConfig.h` sont locaux et ignorés ; leurs
versions `.example.h` ne contiennent que des valeurs fictives.

## Premier flash USB

Identifier le port avec `arduino-cli board list`, puis :

```powershell
arduino-cli upload -p COMxx `
  --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs `
  firmware/SmaBluetoothDiscovery
```

Configurer ensuite le Wi-Fi, les onduleurs, MQTT, la localisation et les mots
de passe dans l'interface Web. Le scan Bluetooth permet d'affecter chaque SMA à
INV1, INV2 ou INV3 sans recopier manuellement sa MAC.

Le point d'accès de secours existe mais sa récupération réelle reste à valider
avant la release finale.
