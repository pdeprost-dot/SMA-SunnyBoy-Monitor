# OTA et rollback

Deux mécanismes utilisent le même secret administrateur :

- ArduinoOTA sur le LAN ;
- envoi d'un fichier `.bin` depuis la page Système.

Une barre à 100 % ne suffit pas : un test OTA est PASS uniquement après
finalisation, sélection de la partition, redémarrage et identification de la
nouvelle version. Le schéma `min_spiffs` fournit deux partitions applicatives
OTA ; la taille du binaire doit être vérifiée avant publication.

Pour chaque release conserver exactement le binaire réellement testé, son
SHA-256, le commit source, le FQBN, les versions d'outils et le résultat matériel.
Le rollback doit reflasher cet artefact sans recompilation.

Les images ne sont pas signées cryptographiquement. Ne pas exposer OTA hors du
LAN de confiance.
