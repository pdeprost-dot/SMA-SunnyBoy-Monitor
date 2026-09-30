# Phase 2 — première session SMA réelle

## Résultat

La première transaction protocolaire réelle avec SMA #1 est validée.

- cible : `02:00:00:00:00:01` ;
- connexion SPP directe : canal RFCOMM 1 ;
- NetID reçu : 2 ;
- requête Data2+ : `0x00000200` ;
- réponse niveau 1 : 106 octets pour la trame cible ;
- FCS Data2+ reçu et recalculé : `0x700B` ;
- Packet ID : 2 ;
- numéro de série décodé : `1000000001` ;
- compteurs complets du cycle : TX 94 octets, RX 560 octets ;
- réponses valides : 1.

Les 560 octets RX incluent des annonces provenant des autres onduleurs du même
NetID. Elles ont été rejetées par le filtre de source ; seule la réponse de
`02:00:00:00:00:01` a été validée et décodée.

## Coexistence et mémoire

Pendant l'échange validant, les deux API ont continué à répondre : six réponses
HTTP, aucune erreur, latence maximale groupée 1 188 ms. Le Wi-Fi est resté
connecté.

- heap avant connexion : environ 30 ko ;
- minimum heap observé : 12 624 octets ;
- minimum observé du plus grand bloc : 4 852 octets ;
- heap après décodage/pendant la session : 21 144 octets ;
- plus grand bloc après décodage : 4 852 octets.

La marge reste critique et doit être traitée comme une contrainte de la suite,
pas comme une validation de stabilité.

## Compatibilité SB2500HF-30 constatée

Le modèle envoie la séquence niveau 1 `0x0002`, `0x000A`, `0x000C`, `0x0005`.
Sa trame `0x0005` peut ne contenir que 26 octets : l'adresse locale ne doit pas
être lue aveuglément à l'offset 26 comme dans certaines références. Elle est
obtenue auprès de la pile ESP32. L'interrogation SDP s'est également montrée
intermittente ; le canal RFCOMM 1, mesuré lors d'un succès SDP, est désormais
utilisé directement.

## Test de reconnexion

Le diagnostic USB a montré une fermeture propre (`ESP_SPP_CLOSE_EVT`, statut
0), suivie d'un échec lorsque `esp_spp_connect()` était relancé immédiatement :
initialisation client puis fermeture, sans événement d'ouverture. La même
reconnexion réussit après trois secondes. Une garde non bloquante de trois
secondes est donc appliquée après la fermeture SPP ; aucun redémarrage ou
watchdog volontaire n'est utilisé. Trois cycles connect/session/Data2+ puis
disconnect ont ensuite réussi. Les heaps après déconnexion étaient 22 384,
21 624 et 21 632 octets : stabilisation après le premier cycle, sans dérive
entre les cycles 2 et 3. Le minimum heap était 11 468 octets et le plus grand
bloc est descendu ponctuellement à 852 octets ; cette marge reste critique.

## Verdict limité

- `BUILD PASS` ;
- `FLASH PASS` par OTA ;
- `BT CONNECTION PASS` ;
- `SMA SESSION PASS` pour l'initialisation niveau 1/Data2+ ;
- `SMA PROTOCOL RESPONSE PASS` pour `0x00000200` et le numéro de série ;
- reconnect test : `PASS` sur trois cycles avec la garde de trois secondes.

Aucun login, aucune mesure électrique, aucun multi-SMA fonctionnel et aucun
code Phase 3 n'ont été ajoutés.
