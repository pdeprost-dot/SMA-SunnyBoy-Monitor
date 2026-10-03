# Provenance du protocole SMA Bluetooth

Ce document décrit les sources techniques consultées. Il ne constitue ni une
conclusion juridique, ni une déclaration de licence, ni une revendication de
clean-room implementation.

## Lignée historique vérifiée

Le projet **NANODE SMA PV MONITOR** de Stuart Pittaway apparaît publiquement en
2012. Son dépôt contient une implémentation Arduino du transport Bluetooth SMA,
des trames SMA Net1/Net2+, le FCS, le login USER et plusieurs lectures de
mesures. Ses fichiers déclarent Stuart Pittaway comme auteur et indiquent
CC BY-NC-SA 3.0.

SBFspot reconnaît explicitement cette filiation dans son README et dans
`SBFspot.cpp` : S. Pittaway est crédité comme auteur de « NANODE SMA PV
MONITOR », projet « on which this project is based ». La référence étudiée pour
ce firmware est SBFspot commit
`b77f65047e9f4a136a09677838f6b4e5024862ee` (version affichée 3.10.0,
source `V3.9.12-4-gb77f650`).

Notre projet a ensuite utilisé SBFspot comme référence comportementale et de
compatibilité, ainsi que des captures réelles produites par trois SB 2500HF-30.
L'historique de développement montre cette consultation ; le projet ne doit
donc pas être présenté comme clean-room.

Références :

- <https://github.com/stuartpittaway/nanodesmapvmonitor>
- <https://github.com/SBFspot/SBFspot>

## Méthode de classement

- **P** : fait d'interopérabilité imposé par le protocole ou confirmé dans
  plusieurs implémentations indépendantes/captures ;
- **N** : expression ou algorithme déjà clairement présent dans la lignée
  NANODE ;
- **S** : expression/extension apparemment propre à SBFspot ;
- **O** : architecture ou expression développée dans ce projet ;
- **U** : origine non établie avec assez de confiance.

Une revue ultérieure a ajouté la catégorie **M** (« visible dans un dépôt qui
se déclare MIT »). Cette catégorie n'est pas une validation de licence :
ESP32_SMA-Inverter-MQTT contient dans son historique un firmware SMA initial
sous CC BY-NC-SA, et esphome_smabluetooth descend explicitement de ce code.
Voir [`LICENSE_PROVENANCE_REVIEW.md`](LICENSE_PROVENANCE_REVIEW.md).

Un élément peut recevoir plusieurs lettres : une constante peut être un fait
protocolaire P alors que la manière de la parcourir ou de la présenter provient
de N ou S. Les faits et valeurs nécessaires à l'interopérabilité sont séparés
de l'expression source et des choix algorithmiques.

## Comparaison

| Élément | Classe | Éléments factuels observés |
|---|---|---|
| Enveloppe SMA Net1 : `0x7E`, longueur, XOR d'en-tête, adresses, commande | P/N | Présente dans NANODE et SBFspot ; confirmée par les captures ESP. |
| Signature Net2/Data2+ `7E FF 03 60 65` | P/N | Détection explicite dans NANODE puis SBFspot ; nécessaire à l'interopérabilité. |
| Échappement `0x7D`, XOR `0x20`, octets `7D/7E/11/12/13` | P/N | Algorithme visible dans NANODE et SBFspot. Notre boucle est réécrite avec bornes et état inter-fragments, mais la règle est commune. |
| FCS PPP, init/final XOR `0xFFFF`, polynôme réfléchi `0x8408` | P/N | NANODE utilise une table ; SBFspot conserve cette famille. Notre version calcule bit par bit sans reprendre la table. |
| Reconstruction de réponses multi-paquets Net1 vers un Data2+ | P/N/S/O | NANODE concatène déjà les fragments et conserve l'état d'échappement. SBFspot généralise et borne le buffer. Notre automate fixe de 384 octets, ses compteurs et erreurs fatales sont locaux. |
| Construction de l'en-tête Data2+ et packet ID avec bit `0x8000` | P/N/S | Champs et ordre dictés par les trames ; helpers et représentation C++ de notre projet sont locaux. |
| Login `0xFFFD040C`, USER `0x07`, timeout 900 s | P/N/S | Séquence déjà présente sous forme d'octets dans NANODE, structurée explicitement dans SBFspot, confirmée sur les SB2500HF-30. |
| Encodage USER : 12 octets, addition `0x88`, padding `0x88` | P/N/S | NANODE réalise déjà l'addition `0x88`; SBFspot généralise USER/INSTALLER. Notre fonction bornée suit le même protocole. |
| INSTALLER `0x0A` et encodage `0xBB` | P/S | Présent dans SBFspot ; non requis ni validé par notre produit USER. |
| Compteur/packet ID et validation de l'écho | P/N/S/O | Le compteur existe dans NANODE ; SBFspot valide packet ID et timestamp. Notre validation bornée et ses codes d'erreur sont locaux. |
| Commandes et plages énergie, AC/DC, fréquence, état, identité | P/S | LRI historiques partiels dans NANODE ; couverture et regroupements complets repris comme faits d'interopérabilité de SBFspot puis validés par captures. |
| `0x2601` ETotal, `0x2622` EToday, `0x451F` UDC, `0x4521` IDC, `0x821E` nom | P/N | Déjà nommés ou décodés dans NANODE. |
| AC `0x4640`, `0x4648`, courant `0x4650/0x4653`, fréquence `0x4657` | P/S | Les deux familles de courant et les plages groupées sont explicitement gérées par SBFspot ; confirmées sur le matériel. |
| Sentinelles 32/64 bits et records attributs | P/S | Représentation SMA documentée par le comportement SBFspot et vérifiée sur les réponses réelles ; notre état `NotAcquired/Unavailable/Valid` est O. |
| Sémantique des tags relais 51/311 | P/S | Libellés issus des définitions SBFspot, confirmés par les transitions matérielles. |
| BTSignal L1 contrôle `0x03`, payload `05 00`, octet ×100/255 | P/N/S | La requête et le calcul sont déjà commentés dans NANODE et repris par SBFspot. Non exécuté par le firmware courant. |
| Initialisation Bluetooth HC-05 et commandes AT NANODE | N | Spécifique au matériel NANODE ; non utilisée par l'ESP32. |
| Connexion ESP32 `BluetoothSerial`, canal RFCOMM 1 et lifecycle begin/end | P/O | API et canal validés localement ; stratégie Wi-Fi OFF et cycle frais par onduleur sont propres à ce projet. |
| Scheduler trois slots, maximum deux essais, sans dette | O | Politique produit locale, absente des deux références historiques. |
| Gestion mémoire, garde RFCOMM, arrêt réseau/BT et restauration | O | Développée et mesurée sur ESP32-WROOM-32. |
| Web, NVS, scan/affectation, OTA et MQTT retenu | O | Architecture produit locale. |
| JSON zéro/null, `data_valid`, `data_complete`, conservation du snapshot | O | Modèle de validité et publication propre au projet, avec noms de compatibilité inspirés de SBFspot. |
| Calcul lever/coucher | U | Algorithme astronomique courant ; sa source initiale n'est pas encore documentée avec certitude. |

## Commandes et plages utilisées

Les constantes ci-dessous sont traitées comme faits d'interopérabilité, mais
leur regroupement a été vérifié contre SBFspot et sur le matériel réel :

| Données | Commande | Plage |
|---|---:|---:|
| PACTot | `0x51000200` | `0x00263F00..0x00263FFF` |
| ETotal / EToday | `0x54000200` | `0x00260100..0x002622FF` |
| PAC L1 | `0x51000200` | `0x00464000..0x004642FF` |
| UAC / IAC | `0x51000200` | `0x00464800..0x004655FF` |
| Fréquence | `0x51000200` | `0x00465700..0x004657FF` |
| PDC | `0x53800200` | `0x00251E00..0x00251EFF` |
| UDC / IDC | `0x53800200` | `0x00451F00..0x004521FF` |
| Température | `0x52000200` | `0x00237700..0x002377FF` |
| Statut / relais | `0x51800200` | `0x00214800` et `0x00416400` |
| Temps fonctionnement/injection | `0x54000200` | `0x00462E00..0x00462FFF` |
| Nom/classe/type/version | `0x58000200` | `0x00821E00..0x008234FF` |

## Éléments originaux validés matériellement

- automate non bloquant adapté à Arduino-ESP32 ;
- accumulation multi-fragments dans des buffers fixes avec contrôles de bornes ;
- stratégie mémoire Wi-Fi OFF → Bluetooth → Wi-Fi ON ;
- cycle Bluetooth indépendant pour chaque onduleur ;
- scheduler borné et politique de disponibilité ;
- décodage avec validité par champ, conservation du zéro réel et des sentinelles ;
- configuration NVS, découverte/affectation, Web, MQTT et OTA ;
- instrumentation heap, breadcrumbs de crash et restauration réseau.

## Limites de cette étude

La présence antérieure d'une séquence dans NANODE montre que SBFspot n'en est
pas nécessairement l'origine. Elle ne détermine pas à elle seule si cette
séquence est protégeable, si notre expression est dérivée, ou quelles conditions
de redistribution s'appliquent. Ces questions restent séparées de l'analyse
technique de provenance.

La comparaison source approfondie des deux implémentations ESP32 déclarées MIT
et le plan de réduction du risque d'expression sont documentés séparément dans
[`LICENSE_PROVENANCE_REVIEW.md`](LICENSE_PROVENANCE_REVIEW.md). Sa conclusion
technique est qu'une licence MIT globale est plausible pour l'architecture
produit, mais pas encore suffisamment étayée pour le noyau protocolaire sans
réécriture indépendante ou revue juridique.
