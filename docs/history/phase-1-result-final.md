# Phase 1 — résultat matériel final

## Verdict

`SMA DISCOVERY TEST PASS`

Les trois onduleurs SMA Sunny Boy SB2500HF-30 ont été détectés réellement par
Bluetooth Classic pendant que le Wi-Fi et le serveur Web restaient actifs :

| Onduleur | Adresse Bluetooth | Résultat |
|---|---|---|
| SMA #1 | `02:00:00:00:00:01` | détecté |
| SMA #2 | `02:00:00:00:00:02` | détecté |
| SMA #3 | `02:00:00:00:00:03` | détecté |

## Mesures du cycle validant

- `devices=3` ;
- `callbacks=6` ;
- `overflow=false` ;
- Wi-Fi connecté pendant et après le scan ;
- 22 requêtes HTTP traitées pendant le scan ;
- heap final : 29 580 octets ;
- plus grand bloc interne final : 14 836 octets ;
- minimum heap observé : 11 796 octets ;
- plus grand bloc interne ponctuellement descendu à 2 036 octets.

La remontée du heap et du plus grand bloc après l'inquiry confirme que le
nettoyage des résultats internes et le découpage en fenêtres permettent au
Web et au Wi-Fi de rester opérationnels. Toutefois, le creux ponctuel à
2 036 octets montre une pression mémoire sévère lorsque les appareils réels
sont visibles. Ce point est explicitement à surveiller avant et pendant toute
future Phase 2 ; aucune conclusion de stabilité 24/7 ne peut en être déduite.

## Périmètre validé

Ce jalon valide uniquement la découverte Bluetooth Classic simultanée des
trois adresses attendues avec Wi-Fi/Web actifs. Il ne valide ni connexion SMA,
ni session/login, ni Data2+, ni décodage de mesure. Aucun code Phase 2 n'est
inclus dans ce jalon.
