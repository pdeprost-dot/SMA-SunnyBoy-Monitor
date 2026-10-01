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
