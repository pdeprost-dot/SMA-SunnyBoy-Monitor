# SMA SunnyBoy Monitor — état du projet

## Jalon courant

`COMPLETE DATASET + MQTT SNAPSHOT V1`

- branche : `feature/sma-memory-alignment-v1` ;
- commit firmware validé : `c52523f59f433dfe4eda15d7114c223f9ae13379` ;
- cible : ESP32-WROOM-32 avec Arduino-ESP32 3.3.11 ;
- installation : trois SMA Sunny Boy SB 2500HF-30.

## Fonctions matériellement validées

- découverte Bluetooth Classic des trois onduleurs ;
- RFCOMM canal 1, session SMA Data2+ et login USER ;
- reconstruction Data2+ sur plusieurs fragments Bluetooth L1 ;
- identité, état, énergie, mesures AC/DC, température et compteurs horaires ;
- scheduler avec un lifecycle Bluetooth indépendant par onduleur ;
- maximum deux tentatives fraîches par slot, sans boucle ni rattrapage ;
- arrêt réseau temporaire pour préserver la mémoire RFCOMM ;
- restauration Wi-Fi, Web, mDNS, OTA et MQTT ;
- snapshot JSON MQTT retenu par onduleur ;
- OTA prioritaire et mode maintenance ;
- intégrité heap après les cycles validés.

## Stratégie de disponibilité

L'appareil est un moniteur quotidien, pas un système d'acquisition sans perte.
La disponibilité réseau et OTA prime sur une mesure isolée. Un échec conserve
le dernier snapshot valide et son horodatage. Il ne crée ni zéro artificiel,
ni acquisition de rattrapage, ni augmentation automatique du nombre d'essais.

## Dataset et MQTT

Champs validés : série, nom, classe, type, version, statut, relais réseau,
température, ETotal, EToday, PACTot, PAC1, UAC1, IAC1, fréquence, PDC1, UDC1,
IDC1, temps de fonctionnement et temps d'injection.

Le signal Bluetooth SMA reste optionnel et non validé. Il demeure `null`
jusqu'à une acquisition fiable.

Topics : `smaesp/inv1`, `smaesp/inv2`, `smaesp/inv3`. Les messages sont QoS 0,
retenus et publiés après retour réseau. Le buffer JSON fixe mesure 1024 octets,
le buffer MQTT 1152 octets et l'exemple matériel validé 533 octets.

Un zéro valide est publié comme `0`. Les états internes `UNAVAILABLE` et
`NOT_YET_ACQUIRED` sont distincts mais publiés comme `null`. `data_valid`
représente la validité de l'acquisition sans exiger les champs optionnels.

## Validation en cours

**SUNSET / NIGHT TRANSITION: TEST IN PROGRESS — 01/10/2026**

L'ESP est laissé intact pour observer la transition production, coucher du
soleil et nuit. Aucun résultat de cette observation n'est encore déclaré PASS.

## Hors périmètre actuel

- intégration ProgHard Link ;
- interface Web V2 ;
- interface M5Stack Core2 ;
- PZEM/RS485 ;
- tag ou release golden finale.

## MQTT/Web V2 et transition jour/nuit

Le schema V2 ajoute versions de schema/firmware, nom d'installation optionnel,
temps onduleur, lever/coucher du soleil, puissance DC totale, heures de
reveil/sommeil, `data_complete` et `acquisition_result`. Les tags relais 51 et
311 deviennent `Closed` et `Open`; l'attribut special `0x00FFFFFD` devient
`null`. Le dashboard responsive presente trois cartes et distingue zero,
indisponible et acquisition nocturne partielle.

- sunset avec zero valide : PASS ;
- nuit joignable avec dataset partiel : PASS ;
- reveil naturel du matin : **NOT YET TESTED**.

`bt_signal_percent` reste `null` car il exige une transaction SMA L1 distincte
(`control=0x03`, payload `05 00`, octet 22 x 100/255). Le RSSI ESP n'est pas
utilise comme substitut.

## Configuration/Web V3

Le firmware generique se configure sans recompilation par NVS et interface Web.
La navigation separe Dashboard, Onduleurs, Reseau, MQTT, Systeme et Diagnostics.
Les secrets sont write-only; l'API ne retourne que leur etat configure. Les
identites onduleur, Wi-Fi/AP, MQTT, installation, localisation, fuseau horaire
et mot de passe OTA sont persistants. La mise a jour `.bin` authentifiee depuis
le navigateur complete ArduinoOTA. Voir `configuration-web-v3.md`.
