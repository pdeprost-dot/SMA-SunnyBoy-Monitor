# Développement distant — Wi-Fi, Web et OTA LAN

Ce socle est un outil de Phase 1. Il n'ajoute aucun protocole SMA.

## Provisioning

Sans configuration valide, l'ESP32 ouvre :

- SSID : `SMA-Monitor-XXXX` ;
- mot de passe : `SMAsetup-XXXX` ;
- page : adresse indiquée par Serial ou par la passerelle du réseau affichée
  par le système d'exploitation.

`XXXX` est affiché sur Serial et dérivé de l'identité de la carte. La page
permet de scanner les SSID, choisir le réseau, saisir le mot de passe et
redémarrer. Dans la version produit actuelle, les secrets enregistrés sont
renvoyés uniquement à l'interface Web authentifiée et ne sont pas écrits dans
les logs.

L'effacement Wi-Fi exige un POST sur `/api/wifi/reset` avec le corps JSON
exact `{"confirm":"ERASE_WIFI"}`. L'appareil redémarre ensuite en AP.

## API

- `GET /api/status`
- `POST /api/bt/scan`
- `GET /api/bt/results`
- `GET /api/log`
- `GET /api/wifi/networks`
- `POST /api/wifi/config` (`ssid` et `password` en formulaire)
- `POST /api/wifi/reset`
- `POST /api/reboot` avec `{"confirm":"REBOOT"}`

Le scan Bluetooth est asynchrone. Un second démarrage pendant `SCANNING`
retourne HTTP 409. L'OTA n'est traitée que hors scan.

## OTA LAN

Le hostname est imprimé sur Serial. Dans RC1, un secret administrateur/OTA
aléatoire est créé en NVS si aucun secret valide n'existe, mais sa valeur n'est
pas imprimée. Cette limite empêche le provisionnement Web autonome d'une NVS
vierge ; voir [INSTALLATION.md](INSTALLATION.md). Une installation existante
doit conserver son secret configuré dans un emplacement privé.

Compiler avec une partition à deux slots OTA :

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs --build-path build/remote firmware/SmaBluetoothDiscovery
```

Envoyer ensuite le binaire avec l'outil du cœur ESP32 :

```powershell
python "$env:LOCALAPPDATA\Arduino15\packages\esp32\hardware\esp32\3.3.11\tools\espota.py" -i ADRESSE_IP -p 3232 -a MOT_DE_PASSE_OTA -f build/remote/SmaBluetoothDiscovery.ino.bin
```

Un `OTA TEST PASS` exige un transfert réel réussi, le redémarrage et la
lecture de la nouvelle version. Une compilation ne valide pas l'OTA.

## État de validation du socle

Le 2026-09-30, la compilation et le flash USB ont réussi sur l'ESP32-D0WD
révision 1.0. Au démarrage simultané de l'AP, du serveur Web, de l'OTA et du
Bluetooth Classic, les mesures étaient :

- heap libre : 37 708 octets ;
- minimum observé : 35 972 octets ;
- plus grand bloc interne : 24 564 octets.

Cette marge reste contrainte. Le Web, les API, un scan déclenché par API et
l'OTA doivent encore être essayés réellement avant tout `TEST PASS`. La
stabilité et la fragmentation devront être suivies pendant des cycles longs.
