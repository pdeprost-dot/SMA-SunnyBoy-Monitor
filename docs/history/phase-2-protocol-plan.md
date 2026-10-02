# Phase 2 — première transaction SMA

## Cible et arrêt fonctionnel

La couche protocolaire cible exclusivement `02:00:00:00:00:01`, numéro de
série attendu `1000000001`. Le scanner Phase 1 n'est pas utilisé avant la
connexion. Le développement fonctionnel doit s'arrêter dès la première
réponse Data2+ valide et décodée.

## Séquence minimale retenue

1. connexion Bluetooth Classic SPP directe à la MAC connue, PIN Bluetooth
   fixe `0000`, canal RFCOMM `1` mesuré sur le SB2500HF-30 ; cette connexion
   directe évite son interrogation SDP intermittente ;
2. réception de l'annonce SMA niveau 1, commande `0x0002`, et extraction du
   NetID ;
3. réponse niveau 1 `0x0002` contenant `0x00700400`, le NetID et les champs
   d'initialisation ;
4. attente des messages intermédiaires puis de la commande niveau 1 `0x0005` ;
   l'adresse Bluetooth locale est obtenue auprès de la pile ESP32 puis inversée
   pour la représentation SMA (le SB2500HF-30 envoie une trame `0x0005` de
   26 octets, trop courte pour l'offset 26 utilisé par certaines références) ;
5. émission d'une requête Data2+ `0x00000200` dans une enveloppe niveau 1
   `0x0001` ;
6. validation de la longueur et du XOR niveau 1, déséchappement Data2+,
   signature `FF 03 60 65`, FCS CRC-16/X25, identifiant de paquet et décodage
   du numéro de série ;
7. succès uniquement si le numéro décodé vaut `1000000001`.

Le login n'est pas inclus dans ce premier incrément : l'identité Data2+ est
une transaction protocolaire vérifiable ne nécessitant pas d'identifiant SMA.
Il ne sera ajouté que si cette transaction démontre qu'il est nécessaire pour
atteindre le premier résultat demandé.

## Architecture

`SmaBluetoothClient` possède des buffers fixes de 384 octets, un parseur RX
incrémental et une machine à états. La seule API Arduino potentiellement
bloquante, `BluetoothSerial::connect()`, s'exécute dans une tâche FreeRTOS
dédiée afin de laisser Web, Wi-Fi et OTA progresser dans `loop()`.

API de développement :

- `GET /api/sma/status` ;
- `POST /api/sma/connect` ;
- `POST /api/sma/disconnect`.

## Origines et licences

- `darrylb123/ESP32_SMA-Inverter-MQTT` (MIT) a servi de référence principale
  pour l'ordre des commandes, les formats interopérables et les offsets de la
  réponse d'identité ;
- SBFspot (CC BY-NC-SA 3.0) a été consulté uniquement comme référence
  comportementale ; aucun de ses blocs de code n'a été porté ;
- `wimpie007/ESP32_to_SMA_ESPHome` ne contient pas de licence identifiable et
  n'a fourni aucun code au projet.

L'implémentation présente est propre au projet. Les constantes et formats de
trame strictement nécessaires à l'interopérabilité SMA sont documentés ici.
