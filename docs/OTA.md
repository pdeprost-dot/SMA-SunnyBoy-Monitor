# OTA et rollback

Deux mécanismes utilisent le même secret administrateur :

- ArduinoOTA sur le LAN ;
- envoi d'un fichier `.bin` depuis la page Système.

Sur un appareil neuf, le mot de passe administrateur est initialement vide et
les deux mécanismes OTA sont désactivés. Ils deviennent disponibles après
définition d'un mot de passe non vide dans **System** puis redémarrage. Cela
évite qu'une première configuration locale rende l'OTA non authentifiée.

Une barre à 100 % ne suffit pas : un test OTA est PASS uniquement après
finalisation, sélection de la partition, redémarrage et identification de la
nouvelle version. Le schéma `min_spiffs` fournit deux partitions applicatives
OTA ; l'image applicative `.bin` doit tenir dans la limite de **1 966 080
octets**. La procédure Arduino IDE et l'identification du bon fichier sont
détaillées dans [INSTALLATION.md](INSTALLATION.md).

Pour chaque release conserver exactement le binaire réellement testé, son
SHA-256, le commit source, le FQBN, les versions d'outils et le résultat matériel.
Le rollback doit reflasher cet artefact sans recompilation.

Les images ne sont pas signées cryptographiquement. Ne pas exposer OTA hors du
LAN de confiance.
