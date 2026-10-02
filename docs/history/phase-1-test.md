# Phase 1 — protocole de test matériel

## But

Répondre à une seule question : l'ESP32-WROOM-32 détecte-t-il les trois Sunny
Boy en Bluetooth Classic ?

## Avant téléversement

1. Identifier physiquement la carte reliée au PC.
2. Exécuter `arduino-cli board list` et relever le port courant.
3. Vérifier qu'il s'agit bien d'un ESP32 classique, pas d'un C3, C6, S2 ou S3.
4. Compiler avec le FQBN `esp32:esp32:esp32`.

## Essai

1. Téléverser le firmware sans effacement global de la flash.
2. Ouvrir le port série à 115200 bauds.
3. Conserver le log complet depuis le boot.
4. Laisser au moins trois cycles d'inquiry se terminer.
5. Renvoyer le log brut, même en cas d'erreur ou de détection partielle.

Chaque cycle dure 12 secondes et est suivi d'une pause de 5 secondes. Le log
indique MAC, nom, RSSI et classe d'appareil lorsque l'API les fournit. Il
affiche un marqueur `MATCH` pour chaque SMA connu et un résumé cumulatif.

## Critère

- `BUILD PASS` : compilation réussie seulement.
- `TEST PASS` : au moins une MAC SMA connue détectée pendant un essai réel.
- Une détection partielle est conservée comme résultat réel et doit être
  investiguée ; elle n'est pas transformée artificiellement en succès total.

