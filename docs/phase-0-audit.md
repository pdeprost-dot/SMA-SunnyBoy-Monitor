# Phase 0 — synthèse de l'audit SMA

L'audit complet initial a été réalisé le 2026-09-29 avant la création de ce
dépôt. Ses conclusions SMA sont conservées ici sous forme synthétique.

- Référence ESP32 principale :
  [`darrylb123/ESP32_SMA-Inverter-MQTT`](https://github.com/darrylb123/ESP32_SMA-Inverter-MQTT),
  licence MIT.
- Référence comportementale :
  [`SBFspot/SBFspot`](https://github.com/SBFspot/SBFspot), licence
  CC BY-NC-SA 3.0. Son code ne doit pas être porté dans ce projet.
- `ESP32_to_SMA_ESPHome` ne présentait pas de licence identifiable lors de
  l'audit : aucune copie de code.
- Le protocole utilise Bluetooth Classic SPP, une enveloppe SMA niveau 1 et
  SMA Data2+ au niveau 2.
- La compatibilité du SB2500HF-30 est probable mais ne sera déclarée qu'après
  une transaction physique valide.
- La puissance réactive Q reste inconnue tant que le modèle réel ne l'a pas
  exposée et que sa valeur n'a pas été décodée.
- L'architecture future séparera transport, framing, session, décodage LRI et
  publication. ProgHard Link conservera son client MQTT unique.
- Le M5Stack Core2 ne sera étudié qu'après validation sur ESP32-WROOM-32.

L'audit détaillé d'origine reste archivé dans le dossier de travail parent.

