# SMA SunnyBoy Monitor — état du projet

## Dépôt canonique prévu

`pdeprost-dot/SMA-SunnyBoy-Monitor`

## Périmètre

Le projet courant concerne exclusivement les trois onduleurs SMA
SB2500HF-30. PZEM/RS485 reste assuré par l'ESP8266 existant et est hors
périmètre. ProgHard Link, MQTT et un éventuel M5Stack Core2 viendront après la
validation du protocole SMA isolé.

## Jalon actif

Phase 1 — Bluetooth Classic Discovery.

Objectif mesurable : détecter réellement au moins une des trois adresses SMA
connues par inquiry BR/EDR depuis un ESP32 classique.

Une compilation seule donne `BUILD PASS — HARDWARE TEST PENDING`.

Premier essai matériel le 2026-09-29 : build et flash PASS sur ESP32-D0WD,
mais trois inquiries ont retourné zéro appareil. Phase 1 non validée ; voir
`phase-1-result-2026-09-29.md`.

## Jalons suivants

Aucun jalon suivant ne doit être commencé automatiquement. La Phase 2 sera
définie après examen du log matériel de Phase 1 et portera sur une seule
connexion SMA.

