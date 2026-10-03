# Sécurité

## Modèle de déploiement

Le firmware est destiné à un réseau local de confiance. Il ne doit pas être
exposé directement à Internet.

- L'interface utilise HTTP Basic sans TLS : identifiant et mot de passe peuvent
  être observés par un tiers présent sur le LAN.
- Le même secret administrateur protège le Web, ArduinoOTA et l'OTA navigateur.
- Les secrets sont stockés dans la NVS de l'ESP32 et ne sont pas chiffrés par ce
  projet. L'interface authentifiée peut les afficher sur demande.
- Les images OTA sont authentifiées par mot de passe mais ne sont pas signées.
- Les diagnostics, Serial et MQTT ne doivent jamais contenir de secret.

Sur NVS vierge, le Web accepte initialement `admin` avec un mot de passe vide
afin de permettre le provisionnement local par le point d'accès de secours.
ArduinoOTA et l'OTA navigateur restent désactivés jusqu'à la définition d'un
mot de passe administrateur non vide. Laisser ce mot de passe vide maintient
l'interface Web sans protection par secret, y compris sur le LAN : définir un
mot de passe individuel avant l'exploitation normale est fortement recommandé.

Utiliser un VLAN ou réseau IoT isolé, un mot de passe unique et fort, et ne pas
réutiliser un credential sensible d'un autre service.

## Signalement

Ne pas ouvrir une issue publique contenant un mot de passe, une MAC réelle, un
numéro de série ou une capture réseau privée. Contacter d'abord le mainteneur du
dépôt par un canal privé GitHub approprié.
