# SMA SunnyBoy Monitor — état du projet

## Dépôt canonique prévu

`pdeprost-dot/SMA-SunnyBoy-Monitor`

## Périmètre

Le projet courant concerne exclusivement les trois onduleurs SMA
SB2500HF-30. PZEM/RS485 reste assuré par l'ESP8266 existant et est hors
périmètre. ProgHard Link, MQTT et un éventuel M5Stack Core2 viendront après la
validation du protocole SMA isolé.

## Jalon clôturé

Phase 1 — Bluetooth Classic Discovery.

Objectif mesurable : détecter réellement au moins une des trois adresses SMA
connues par inquiry BR/EDR depuis un ESP32 classique.

Une compilation seule donne `BUILD PASS — HARDWARE TEST PENDING`.

Premier essai matériel le 2026-09-29 : build et flash PASS sur ESP32-D0WD,
mais trois inquiries ont retourné zéro appareil. Phase 1 non validée ; voir
`phase-1-result-2026-09-29.md`.

La Phase 1 est validée : les trois adresses SMA attendues ont été détectées
réellement avec Wi-Fi et Web actifs. Le cycle validant compte trois appareils,
six callbacks, aucune saturation du tableau et 22 requêtes HTTP servies.
Voir `phase-1-result-final.md`.

Le minimum heap de 11 796 octets et un plus grand bloc ponctuellement réduit à
2 036 octets imposent une surveillance spécifique lors de la future Phase 2.

## Jalons suivants

Aucun jalon suivant ne doit être commencé automatiquement. La Phase 2 portera
sur une seule connexion SMA et devra commencer par un budget mémoire strict.

