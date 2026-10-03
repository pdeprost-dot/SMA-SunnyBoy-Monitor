# Installation, compilation et mises à jour

Ce guide décrit le cycle complet du firmware **SMA-SunnyBoy-Monitor
1.0.0-rc.2** avec Arduino IDE 2.x sous Windows 10/11. La configuration validée
est Arduino-ESP32 3.3.11, PubSubClient 2.8 et le schéma de partitions
`min_spiffs`.

Sur une carte neuve, le compte Web initial est `admin` avec un mot de passe
vide. Définissez un mot de passe administrateur individuel dès la première
configuration. Tant qu'il reste vide, ArduinoOTA et l'OTA navigateur sont
désactivés.

## A. Première installation avec Arduino IDE

### 1. Matériel requis

- un **ESP32 original avec Bluetooth Classic/BR-EDR**, cible validée :
  ESP32-WROOM-32 ;
- un câble USB de données, un PC Windows 10/11 et un LAN de confiance ;
- des onduleurs compatibles (validation actuelle : SMA SB 2500HF-30).

ESP32-S2, S3, C3 et C6 ne sont pas des cibles équivalentes : ils ne fournissent
pas le Bluetooth Classic/SPP requis.

### 2. Installer Arduino IDE

1. Télécharger Arduino IDE 2.x depuis la
   [page officielle Arduino](https://www.arduino.cc/en/software).
2. Installer puis ouvrir l'IDE.

Arduino CLI, VS Code et PlatformIO ne sont pas requis.

### 3. Télécharger et ouvrir le projet

1. Sur GitHub, choisir **Code > Download ZIP**.
2. Extraire complètement l'archive dans un dossier local.
3. Ouvrir exactement
   `firmware/SmaBluetoothDiscovery/SmaBluetoothDiscovery.ino`.

Arduino exige ici que `SmaBluetoothDiscovery.ino` reste dans le dossier
`SmaBluetoothDiscovery`, avec les `.cpp` et `.h` voisins. Ne copiez pas le seul
`.ino` ailleurs. Pour un utilisateur avancé, `git clone` peut remplacer le ZIP.

### 4. Installer le support ESP32 validé

1. Ouvrir **File > Preferences** (*Fichier > Préférences*).
2. Dans **Additional Boards Manager URLs**, ajouter :
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`.
3. Ouvrir **Tools > Board > Boards Manager**.
4. Rechercher `esp32`, choisir **esp32 by Espressif Systems**, sélectionner
   **3.3.11**, puis installer.

La version 3.3.11 est celle utilisée pour valider RC1 ; ne la remplacez pas
silencieusement pendant un diagnostic.

### 5. Installer la bibliothèque externe

Dans **Tools > Manage Libraries**, rechercher **PubSubClient** et installer la
version **2.8**. C'est la seule bibliothèque externe à installer manuellement.
WiFi, BluetoothSerial, WebServer, Preferences, Update et ArduinoOTA sont
fournis par Arduino-ESP32.

### 6. Sélectionner la carte et les partitions

Dans **Tools** :

1. choisir **Board > esp32 > ESP32 Dev Module** ;
2. choisir **Partition Scheme > Minimal SPIFFS (1.9MB APP with OTA/128KB
   SPIFFS)**.

Cela correspond au FQBN
`esp32:esp32:esp32:PartitionScheme=min_spiffs`. Cette disposition fournit deux
emplacements applicatifs OTA. La taille maximale d'une image est
**1 966 080 octets**.

### 7. Compiler et téléverser par USB

1. Brancher l'ESP32 par USB.
2. Choisir **Tools > Port > COMx**.
3. Cliquer **Verify** (✓). Un succès signifie que le sketch et ses dépendances
   ont été compilés et liés.
4. Cliquer **Upload** (→).
5. Ouvrir **Tools > Serial Monitor** à **115200 bauds**.

BOOT n'est normalement pas requis. Si une carte ne passe pas automatiquement
en téléchargement, maintenir BOOT au début de l'envoi puis le relâcher lorsque
l'écriture commence.

## B. Premier démarrage et configuration

### 1. Réseau initial

Le firmware charge la NVS `sma-monitor`. Sans SSID enregistré, il démarre un
point d'accès dont les valeurs par défaut sont :

- SSID `SMA-Monitor-XXXX` ;
- mot de passe `SMAsetup-XXXX`.

`XXXX` correspond aux quatre derniers chiffres hexadécimaux de l'identité de
la carte. Si un STA enregistré ne se connecte pas dans les 18 secondes, le
point d'accès démarre aussi. Le firmware ne fixe pas une adresse IP de portail :
consultez le moniteur série ou la passerelle affichée par Windows, sans
supposer une adresse. Une nouvelle tentative STA a lieu toutes les 60 secondes
et le point d'accès s'arrête après connexion.

Ouvrir l'adresse IP annoncée sur Serial dans un navigateur. Le Web utilise
HTTP sur le port 80.

### 2. Authentification Web

Au premier démarrage sur NVS vierge, la connexion HTTP Basic utilise :

- utilisateur `admin` ;
- mot de passe : laisser le champ **vide**.

Définissez ensuite dans **System** un mot de passe administrateur/OTA. Ce même
secret protégera le Web, ArduinoOTA et l'installation d'un `.bin`. Les champs
secrets sont masqués par défaut et disposent d'**Afficher/Masquer**. Si vous le
laissez volontairement vide, l'accès Web reste `admin` + mot de passe vide,
y compris sur le LAN, et les deux mécanismes OTA restent désactivés.

### 3. Parcours de configuration Web

Navigation : **Dashboard | Inverters | Network | MQTT | System | Diagnostics**.

1. **Network** : cliquer **Scanner**, choisir le Wi-Fi, saisir son mot de passe
   et personnaliser éventuellement le point d'accès de secours. Enregistrer et
   redémarrer lorsque demandé.
2. **Inverters** : activer INV1 à INV3, donner un nom, lancer **Scanner
   Bluetooth**, sélectionner un appareil et l'affecter au slot. Saisir le mot
   de passe SMA USER. MAC, série et identité détectée sont en lecture seule. Le
   test de connexion confirme l'identité sans enregistrer autre chose.
3. **MQTT** : renseigner activation, broker, port, utilisateur, mot de passe,
   préfixe et intervalle.
4. **System** : renseigner installation, latitude, longitude et fuseau
   (`Europe/Brussels` ou `UTC`). Vérifier heure locale, UTC, DST, NTP, lever et
   coucher. Le secret administrateur/OTA se modifie ici et exige un reboot.
5. Enregistrer chaque page ; les valeurs persistent en NVS.

Les fichiers ignorés `SmaLocalConfig.h` et `SmaPhase3LocalConfig.h` sont des
mécanismes locaux historiques, **non requis** pour le produit configurable.
Leurs `.example.h` ne contiennent que des valeurs fictives.

## C. Vérifier le fonctionnement normal

Après reboot, vérifier :

- WiFi connecté et Web accessible ;
- affectations INV1/INV2/INV3 visibles ;
- scheduler `RUNNING` et acquisitions affichées ;
- MQTT connecté si activé ;
- version `1.0.0-rc.2` affichée ;
- Bluetooth revenu à `BT_OFF` entre acquisitions ;
- OTA disponible, sans reboot/panic dans **Diagnostics**.

Pendant une acquisition SMA, Wi-Fi et les services réseau sont volontairement
arrêtés puis restaurés. Une indisponibilité Web/OTA brève est normale.

## D. Modifier et recompiler

1. Mettre à jour ou rouvrir l'arborescence complète.
2. Conserver Arduino-ESP32 3.3.11, PubSubClient 2.8, **ESP32 Dev Module** et
   **Minimal SPIFFS**.
3. Modifier le code puis la version dans
   `firmware/SmaBluetoothDiscovery/Version.h`, source de vérité du nom, de la
   version firmware et du schéma MQTT.
4. Donner à toute variante un identifiant distinct, par exemple
   `1.0.0-rc.2` ou `1.0.1-dev`.
5. Relancer **Verify**.

Ne changez pas le schéma MQTT sans modification volontaire de son contrat.

## E. Téléverser directement par ArduinoOTA

1. Mettre PC et ESP32 sur le même LAN ; vérifier pare-feu, VLAN et mDNS.
2. Attendre que le scheduler soit hors acquisition. Le mode maintenance est
   recommandé pour garder les services réseau disponibles.
3. Dans **Tools > Port**, sélectionner le port réseau
   `sma-sunnyboy-monitor-XXXX`, et non COM.
4. Cliquer **Upload** et saisir le secret administrateur/OTA à la demande.
5. Attendre finalisation, reboot et reconnexion Wi-Fi.
6. Vérifier la nouvelle version dans Dashboard ou Diagnostics.

Si le port réseau n'apparaît pas, redémarrer l'IDE, vérifier mDNS/pare-feu, ou
utiliser l'OTA navigateur. USB reste la récupération. Un transfert à 100 % sans
finalisation ni nouveau boot n'est pas un succès.

## F. Générer le `.bin` applicatif avec Arduino IDE

1. Vérifier carte et partitions.
2. Choisir **Sketch > Export Compiled Binary** (*Croquis > Exporter les
   binaires compilés*, selon la langue).
3. Les fichiers sont exportés dans le dossier du sketch.
4. Pour l'OTA Web, choisir seulement **`SmaBluetoothDiscovery.ino.bin`**.

Ne jamais envoyer les fichiers contenant `bootloader`, `partitions` ou
`merged` via l'OTA Web. L'application doit tenir dans **1 966 080 octets**. La
taille exacte varie avec le code et les outils et doit être contrôlée à chaque
build.

## G. Mise à jour `.bin` depuis le navigateur

Flux : **Arduino IDE > Export Compiled Binary > `.ino.bin` applicatif > Web >
System > Mise à jour firmware (.bin) > Installer**.

1. Se connecter au Web avec `admin` et le secret partagé.
2. Ouvrir **System**, puis **Mise à jour firmware (.bin)**.
3. Confirmer le secret administrateur/OTA et choisir le `.ino.bin`.
4. Cliquer **Installer**, sans couper l'alimentation.
5. Attendre progression, finalisation et reboot automatique.
6. Se reconnecter et vérifier la version active.

Le serveur refuse l'opération pendant Bluetooth/acquisition ou une autre OTA.

## H. Persistance de la configuration

Une OTA normale conserve la NVS : Wi-Fi et AP, affectations/identités/secrets
des onduleurs, MQTT, installation/position/fuseau/maintenance et secret
administrateur/OTA. Un **effacement complet**, un **effacement NVS** ou un
**changement de table de partitions** peut les détruire. Une nouvelle table de
partitions n'est pas une OTA applicative ordinaire et demande une procédure USB.

## I. Sécurité pratique

- Ne pas couper l'alimentation pendant l'écriture.
- Compiler pour ESP32 classique avec `min_spiffs` et vérifier la taille.
- Garder USB comme récupération.
- Ne pas exposer Web/OTA à Internet : HTTP Basic n'est pas chiffré.
- Ne jamais committer secrets ou identifiants d'installation.
- Conserver hors dépôt un `.bin` connu fonctionnel et son SHA-256.

## J. Dépannage

| Problème | Vérifications utiles |
|---|---|
| Aucun COM | Câble USB données, autre port, pilote USB-série de la carte. |
| Cartes ESP32 absentes | URL Espressif, puis paquet `esp32 by Espressif Systems` 3.3.11. |
| Mauvaise famille | **ESP32 Dev Module**, pas S2/S3/C3/C6. |
| `PubSubClient.h` absent | Installer PubSubClient 2.8 dans Library Manager. |
| Compilation échoue | Vérifier versions, sketch complet et première erreur affichée. |
| Firmware trop grand | Choisir **Minimal SPIFFS** ; limite 1 966 080 octets. |
| Envoi USB échoue | Fermer les moniteurs série, vérifier COM, puis essayer BOOT comme ci-dessus. |
| Port OTA invisible | LAN, scheduler/maintenance, mDNS, pare-feu, redémarrage IDE. |
| Authentification refusée | Utiliser le secret admin/OTA, pas Wi-Fi/AP/MQTT/SMA. |
| OTA Web rejetée | `.ino.bin` applicatif, taille, auth et acquisition inactive. |
| Pas de reconnexion | Attendre, consulter Serial 115200/DHCP, puis restaurer par USB. |
| Carte neuve | Utiliser `admin` avec un mot de passe vide, puis définir immédiatement un secret individuel dans **System**. |
| OTA indisponible | Définir un mot de passe administrateur non vide puis redémarrer. |

## Annexe avancée : Arduino CLI

Cette annexe n'est pas requise pour Arduino IDE.

```powershell
arduino-cli core update-index
arduino-cli core install esp32:esp32@3.3.11
arduino-cli lib install "PubSubClient@2.8"
arduino-cli compile --clean `
  --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs `
  firmware/SmaBluetoothDiscovery
```

Premier flash USB :

```powershell
arduino-cli board list
arduino-cli upload -p COMxx `
  --fqbn esp32:esp32:esp32:PartitionScheme=min_spiffs `
  firmware/SmaBluetoothDiscovery
```

Références officielles :

- [installation Arduino IDE](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE) ;
- [installation Arduino-ESP32](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).
