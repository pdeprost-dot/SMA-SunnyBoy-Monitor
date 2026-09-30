# Phase 1 — correction de la coexistence Web/Bluetooth

## Symptôme

Après une inquiry Bluetooth Classic continue de 12 secondes, le serveur Web
ne répondait plus. La mesure utilisateur après scan indiquait 18 864 octets
libres et un plus grand bloc interne de seulement 3 444 octets.

## Cause démontrée

`BluetoothSerial::discoverAsync()` ne bloque pas la boucle Arduino. La mesure
instrumentée a montré un intervalle maximal de boucle de 9 ms. En revanche :

1. l'inquiry Classic continue monopolise périodiquement la radio 2,4 GHz
   partagée et une requête HTTP a dépassé trois secondes ;
2. la bibliothèque conserve chaque résultat dans un `std::map` dynamique
   interne, en plus du tableau fixe de l'application ; ce conteneur n'était
   libéré qu'au scan suivant et accentuait pression mémoire et fragmentation.

## Correction

- découpage du scan en fenêtres d'inquiry de 1 280 ms ;
- créneau Wi-Fi de 1 000 ms entre les fenêtres ;
- copie dédupliquée dans le tableau fixe de 16 résultats ;
- arrêt puis `discoverClear()` après chaque fenêtre ;
- maintien du traitement HTTP dans la boucle principale ;
- journalisation de l'état Wi-Fi, du nombre de requêtes servies pendant le
  scan, du pire intervalle de boucle et des trois métriques heap.

Cette correction ne redémarre pas l'ESP32 et n'ajoute aucun protocole SMA.

## Validation limitée

Trois scans successifs ont été exécutés. Le Wi-Fi est resté en état connecté,
la racine Web a répondu HTTP 200 après chaque cycle et aucune réinitialisation
ou watchdog n'a été observé. Après stabilisation, les deux premiers cycles
mesurés ont tous deux fini à environ 28 572 octets libres et 14 836 octets
pour le plus grand bloc. Il n'y avait donc pas de dégradation cumulative entre
ces cycles. Le minimum historique instrumenté était 16 716 octets.

Ce test local ne constitue pas une validation de détection SMA : aucun appareil
Bluetooth n'était visible à cet emplacement.

Après passage du créneau Wi-Fi à 1 000 ms, le binaire final a servi 13 appels
`/api/status` pendant un scan sans timeout. La latence maximale était 1 199 ms.
La page racine a répondu HTTP 200 immédiatement après `SCAN COMPLETE`. Le heap
est passé de 32 452 à 28 552 octets libres et le plus grand bloc de 17 396 à
13 300 octets ; les minima transitoires étaient respectivement 24 776 et
10 740 octets. Le Wi-Fi est resté connecté et aucun reset/watchdog n'a eu lieu.
