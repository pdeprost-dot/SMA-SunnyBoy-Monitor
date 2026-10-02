# Validation matérielle

Plateforme : ESP32-WROOM-32, Arduino-ESP32 3.3.11, trois SMA Sunny Boy
SB 2500HF-30 installés simultanément.

| Validation | Résultat |
|---|---|
| Scan Bluetooth et affectation des trois SMA | PASS |
| RFCOMM, session, login USER et dataset Data2+ | PASS |
| Reconstruction multi-fragments et FCS/packet ID | PASS |
| Scheduler trois onduleurs, maximum deux tentatives | PASS |
| Restauration Wi-Fi/Web/MQTT/OTA et BT_OFF | PASS |
| MQTT retenu, zéro distinct de null | PASS |
| Configuration Web et persistance NVS | PASS |
| ArduinoOTA et OTA navigateur | PASS |
| Stabilité nocturne | PASS |
| Transition nuit → matin | PASS |
| Transition vers production matinale | PASS |
| Récupération réelle par AP de secours | EN ATTENTE |

L'heure exacte du premier réveil solaire n'a pas été capturée, le collecteur PC
s'étant arrêté à 01:36. Le comportement fonctionnel complet a néanmoins été
observé le matin. Les identifiants de l'installation ne sont pas publiés.
