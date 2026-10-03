# Revue technique de provenance en vue d'une licence MIT

Date de la revue : 3 octobre 2026. Cette revue est une analyse technique des
sources et historiques publics ; ce n'est pas un avis juridique et elle ne
donne aucune licence au présent dépôt.

Sources primaires :

- <https://github.com/stuartpittaway/nanodesmapvmonitor/tree/b9be6ed209a7c5b5836400766d9edd91c33fae1b>
- <https://github.com/SBFspot/SBFspot/tree/b77f65047e9f4a136a09677838f6b4e5024862ee>
- <https://github.com/darrylb123/ESP32_SMA-Inverter-MQTT/tree/342911836417b2032033e8feb82f9e65b449845c>
- <https://github.com/keerekeerweere/esphome_smabluetooth/tree/bb0658dfe3109c1ff45fc44b88419ccc2b434561>

## Périmètre vérifié

| Projet | Révision étudiée | Licence affichée dans la révision | Filiation observée |
|---|---|---|---|
| SMA-SunnyBoy-Monitor | `22ef8251088a8c7a844257765afb74f16131c372` | aucune licence globale | Code produit local ; SBFspot et NANODE ont été consultés pendant le développement du protocole. |
| NANODE SMA PV MONITOR | `b9be6ed209a7c5b5836400766d9edd91c33fae1b` | CC BY-NC-SA 3.0 dans le README et les en-têtes source | Projet public de Stuart Pittaway, premier commit retrouvé le 31 janvier 2012. |
| SBFspot | `b77f65047e9f4a136a09677838f6b4e5024862ee` | CC BY-NC-SA 3.0 (`license.md` et README) | Premier commit GitHub retrouvé le 12 juillet 2013 ; le README et `SBFspot.cpp` créditent NANODE comme projet « on which this project is based ». |
| ESP32_SMA-Inverter-MQTT | `342911836417b2032033e8feb82f9e65b449845c` | MIT dans `LICENSE` et les sources actuelles | Fork GitHub de `Lupo135/ESP32_SMA-Inverter`. Son historique contient un arbre racine `b4f1476` dont le firmware SMA porte CC BY-NC-SA 3.0, fusionné avec un arbre racine ne contenant qu'un `LICENSE` MIT par `511b571`. |
| esphome_smabluetooth | `bb0658dfe3109c1ff45fc44b88419ccc2b434561` | MIT dans `LICENSE` et les sources | Le protocole est ajouté par `4677bf5` avec les copyrights Lupo135/Darryl Bond et une structure directement issue d'ESP32_SMA-Inverter-MQTT ; il n'est donc pas une seconde implémentation indépendante. |

Les déclarations MIT de ces deux derniers dépôts sont des faits vérifiés. Leur
présence ne démontre toutefois pas que les auteurs disposaient du droit de
relicencier toute expression antérieure. Aucun document d'autorisation ou de
double licence expliquant le passage de l'arbre CC à MIT n'a été trouvé dans
les historiques examinés.

## Faits, inférences et questions ouvertes

### Faits vérifiés

- NANODE contient déjà Net1, l'enveloppe Data2+, l'échappement `0x7D`, le FCS,
  le compteur de paquets, le login USER avec transformation `+0x88`, des LRI et
  la reconstruction de paquets.
- SBFspot reprend cette lignée, la généralise et la crédite explicitement.
- ESP32_SMA-Inverter-MQTT possède deux commits racines. Le premier contient le
  firmware SMA sous CC BY-NC-SA ; le second contient seulement un fichier MIT.
  Le merge initial remplace les en-têtes du firmware par MIT sans justification
  de provenance visible dans Git.
- esphome_smabluetooth importe ensuite le module `SMA_Inverter` de cette lignée
  en conservant les copyrights Lupo135 et Darryl Bond.
- Notre historique et nos documents montrent que SBFspot a servi directement
  de référence comportementale, de source de constantes/plages et de guide de
  décodage. Une revendication clean-room serait donc inexacte.
- Notre FCS est une implémentation bit par bit du CRC PPP (`0x8408`), et non la
  table de 256 entrées commune à NANODE, SBFspot et aux deux dépôts MIT.
- Notre transport est un automate borné et non bloquant, sans reprise des noms,
  classes globales ou boucles bloquantes des références.

### Inférences techniques

- La présence des mêmes valeurs dans quatre lignées est une preuve forte de
  fait d'interopérabilité, mais pas une autorisation de reprendre une expression
  source particulière.
- D et E confirment le protocole sur ESP32, mais ne constituent pas une preuve
  d'invention indépendante des algorithmes : leur propre historique relie le
  code au même arbre CC/SBFspot.
- L'expression de notre architecture produit est nettement distincte. Le risque
  résiduel se concentre dans le petit noyau qui construit et interprète les
  trames SMA, créé après consultation de SBFspot.

### Questions non résolues

- Les contributeurs de l'arbre ESP32 initial avaient-ils une autorisation ou
  une double licence non publiée ?
- Quelles parties de SBFspot sont des créations propres, plutôt que des faits
  ou expressions déjà présents dans NANODE ou une documentation SMA ?
- Les séquences et choix de découpage restants dans notre module atteignent-ils
  juridiquement le seuil d'une expression dérivée ? Seule une revue juridique
  peut répondre à cette question.

## Comparaison protocolaire P/M/N/S/O/U

`P` = fait protocolaire ; `M` = visible dans les dépôts déclarés MIT ; `N` =
lignée NANODE ; `S` = ajout/expression SBFspot ; `O` = expression/architecture
locale ; `U` = origine ou droit non résolu. `M` ne signifie pas « preuve de
relicenciabilité », car D et E partagent la lignée décrite ci-dessus.

| Élément | Valeurs identiques | Algorithme comparable | Expression source comparable | Classe | Risque pour notre expression |
|---|---|---|---|---|---|
| Net1 : `0x7E`, longueur, XOR, adresses, commande | Oui, toutes les lignées | Oui | Notre writer borné est distinct, mais l'ordre des champs est imposé | P/M/N/O | faible |
| Data2+ : signature, header et bit packet ID | Oui | Oui | D/E restent très proches de `SBFNet.cpp`; notre builder compact a une structure propre | P/M/N/S/O | moyen |
| Échappement `7D/7E/11/12/13`, XOR `0x20` | Oui | Oui | Boucle inévitablement courte ; notre gestion inter-fragments et bornes est locale | P/M/N/O | faible |
| FCS PPP init/final `FFFF`, polynôme `8408` | Oui | D/E/N/S utilisent la même table | Notre calcul bit par bit ne reprend ni table ni structure | P/M/N/O | faible |
| Reconstruction multipart | Oui sur le résultat attendu | N/S/D/E utilisent packet-count/buffers ; notre automate accumule les fragments Net1 et conserve l'échappement | Structure locale sensiblement différente | P/M/N/S/O | faible à moyen |
| Initialisation NetID et identité | Oui | Même ordre fonctionnel | Les trames sont dictées par l'équipement ; notre machine d'état est propre | P/M/N/S/O | moyen |
| Login USER, `0xFFFD040C`, rôle `0x07`, timeout 900 | Oui | Oui | Notre builder est plus compact mais suit l'ordre SBFspot étudié | P/M/N/S/O/U | moyen |
| Mot de passe 12 octets, `+0x88`, padding `0x88` | Oui | Oui | Notre boucle unique diffère des deux boucles historiques | P/M/N/S/O | faible à moyen |
| Compteur, bit `0x8000`, validation ID/timestamp | Oui | Oui | Validation avec enum d'erreurs et sortie transactionnelle locale | P/M/N/S/O | moyen |
| Commandes, LRI et plages AC/DC/énergie/identité | Oui pour la couverture commune | Tables/switches comparables | Constantes nécessaires à l'interopérabilité ; regroupements étendus via SBFspot | P/M/N/S | faible |
| Sentinelles 32/64 bits | Oui dans SBFspot et E | Oui | Notre état `NotAcquired/Unavailable/Valid` et propagation null/zero sont locaux | P/M/S/O | faible |
| Itération des records, attributs et types | Valeurs/offsets communs | SBFspot et E décodent les mêmes familles | Notre calcul borné de taille et affectation restent influencés par la sémantique SBFspot | P/M/S/O/U | moyen |
| Timestamps de records | Oui | Oui | Notre formatage hôte et sélection wake/sleep sont séparés du parseur | P/M/S/O | faible à moyen |

## Audit fichier par fichier de notre noyau

| Notre fichier/fonction | But | Équivalent NANODE le plus proche | Équivalent SBFspot | Équivalent déclaré MIT | Classe | Risque | Action recommandée |
|---|---|---|---|---|---|---|---|
| `SmaBluetoothClient::receiveBytes/processFrame` | Parser Net1 et pilotage de session | `waitForPacket`, `writePacketHeader` | `Bluetooth.cpp`, réception dans `SBFspot.cpp` | `getPacket` dans `SMA_Inverter.cpp` | P/M/N/S/O | faible | Conserver l'automate ; formaliser les offsets dans une spécification locale. |
| `appendL2Payload/wrapL2` | Déséchappement, échappement et multipart | helpers `writeSMANET2*` et accumulation | `writeByte`/buffer Data2+ | mêmes helpers dans D/E | P/M/N/S/O | faible à moyen | Réimplémentation isolée souhaitable seulement dans une démarche de réduction maximale du risque. |
| `sendLevel1Configuration/buildL2Identity` | Handshake NetID/identité | séquences Net1/N2 NANODE | initialisation SBFspot | `initialiseSMAConnection` D/E | P/M/N/S/O/U | moyen | Candidat de réécriture depuis captures et description de champs. |
| `SmaPhase3::header/buildLoginL2/buildQueryL2` | Construction Data2+, login et requêtes | builder Net2+, login NANODE | `writePacket`, `logonSMAInverter`, `getInverterData` | fonctions quasi homonymes D/E | P/M/N/S/O/U | moyen | Candidat principal à une réécriture indépendante. |
| `SmaPhase3::fcs16` et endian helpers | CRC PPP et scalaires little-endian | table FCS/helpers | table FCS/helpers | même table/helpers D/E | P/M/N/O | faible | Conserver ; documenter paramètres CRC avec vecteurs indépendants. |
| `encodePassword/validateLoginResponse` | Encodage et contrôle de réponse | `+0x88` NANODE | logon et offsets SBFspot | même séquence D/E | P/M/N/S/O/U | moyen | Réécrire depuis un contrat de trame et des vecteurs gelés. |
| `decodeMeasurementResponse` | Taille/itération/attributs/sentinelles | décodage limité | `getInverterData`, records `Rec40*` | switch de `SMA_Inverter.cpp` | P/M/S/O/U | moyen | Candidat principal : parseur par curseur typé, construit sans consulter les implémentations. |
| constantes `Query` et LRI | Catalogue des données | sous-ensemble NANODE | catalogue étendu SBFspot | mêmes valeurs D/E | P/M/N/S | faible | Conserver comme données d'interopérabilité, avec source/capture par entrée. |
| `SbfspotCompat.*` | Modèle de validité et JSON MQTT borné | aucun | noms de champs MQTT comparables | sérialisation différente | O, avec nomenclature S | faible | Conserver ; éventuellement renommer le namespace pour éviter de suggérer une dérivation de code. |
| lifecycle, scheduler, NVS, Web, MQTT, OTA | Produit ESP32 | aucun équivalent global | aucun | aucun | O | faible | Aucun besoin de réécriture de provenance identifié. |

Une recherche source n'a trouvé ni table FCS tierce, ni fonctions SBFspot
copiées textuellement, ni classe `ESP32_SMA_Inverter`, ni blocs substantiels
identiques dans notre dépôt. Le risque « moyen » vient du processus documenté
de consultation et de la proximité fonctionnelle du noyau, pas d'une preuve de
copie littérale constatée par cette revue.

Un contrôle mécanique complémentaire, après suppression des espaces et des
commentaires, n'a trouvé aucune longue suite de lignes identiques entre notre
noyau et NANODE/SBFspot. Le maximum trouvé contre le dépôt ESP32 déclaré MIT
est trois lignes triviales de déclaration ; ce contrôle ne remplace pas
l'analyse qualitative ci-dessus.

## Plan de réimplémentation indépendante si retenu

Le reste du produit peut rester inchangé. Une frontière propre serait :

```text
SmaWireCodec
  encodeL1 / decodeL1
  encodeData2 / feedData2Fragment
  crc16Ppp

SmaSessionCodec
  makeIdentity / makeUserLogin / parseLogin
  makeQuery

SmaRecordDecoder
  decodeRecords -> FieldValue{state, raw, timestamp, type}
```

Pour chaque candidat :

| Candidat | Comportement immuable | Spécification/test d'entrée | Sources à ne pas consulter pendant la réécriture | Tests requis |
|---|---|---|---|---|
| Codec Net1/Data2+ | octets sur le fil, escaping, limites et FCS | captures matérielles expurgées + format de champs neutre + vecteurs actuels | NANODE, SBFspot, D et E | golden TX/RX, chaque caractère échappé, fragments coupés à chaque position, overflow/FCS négatifs |
| Session/identité/login | ordre des messages, USER, password, IDs et timestamps | captures validées et tableau de champs/offsets | fonctions login/init des quatre références et notre ancienne implémentation | identité, login valide/invalide, packet-ID/timestamp erronés, mot de passe 0/12/13 octets |
| Requêtes/records | commandes/plages identiques, deux LRI courant, sentinelles et attributs | corpus RX borné déjà testé + catalogue factuel LRI | parseurs `getInverterData`, `Rec40*`, D/E et notre parseur actuel | tous types/taille, multi-records, legacy/new current, null versus zéro, troncature/NO_DATA |

La personne qui écrit la nouvelle implémentation ne devrait recevoir que cette
spécification neutre, les captures expurgées et les résultats attendus. Une
autre personne peut extraire ces vecteurs. L'ancien module doit rester hors de
vue pendant l'écriture, puis servir uniquement à un test différentiel final.

## Évaluation technique des options de licence

### MIT pour tout le dépôt

**TECHNIQUEMENT PLAUSIBLE MAIS RISQUE DE PROVENANCE RÉSIDUEL.**

L'architecture produit et la majeure partie du code sont clairement originaux,
et aucun copier-coller substantiel n'a été identifié. Cependant, le noyau de
session/construction/décodage a été écrit après étude directe de SBFspot. Les
deux dépôts MIT n'apportent pas une provenance indépendante fiable : D contient
un ancêtre CC visible et E descend de D. Ajouter aujourd'hui MIT à tout le dépôt
n'est donc pas techniquement suffisamment étayé sans revue juridique ou
réimplémentation indépendante des zones « moyen ».

### Apache-2.0 pour tout le dépôt

Même obstacle de provenance que MIT, avec en plus des conditions et une licence
de brevets différentes. Aucun élément technique de cette revue ne rend
Apache-2.0 plus défendable pour les expressions potentiellement dérivées.

### CC BY-NC-SA 3.0 par héritage

C'est l'option conservatrice si le noyau est considéré comme une adaptation de
NANODE/SBFspot, mais la pertinence de Creative Commons pour du logiciel et la
portée exacte du ShareAlike nécessitent une revue juridique. Cette revue ne
conclut pas que tout le dépôt doit obligatoirement l'adopter.

### Licence séparée

Techniquement possible : garder les composants produit originaux séparés d'un
module protocolaire sous conditions compatibles avec sa provenance. Cela exige
une frontière de fichiers nette et une détermination juridique de la licence
du module ; le simple ajout d'en-têtes différents ne résout pas une dérivation.

### Réécriture indépendante

C'est la voie technique la plus solide vers MIT : remplacer les trois zones
« moyen » depuis une spécification et des captures neutres, conserver les
tests matériels et documenter qui a fourni la spécification et qui a écrit le
nouveau code. Une revue juridique reste recommandée avant publication sous une
licence globale.

## Conclusion opérationnelle

Ne pas ajouter de licence MIT maintenant. Les constantes de protocole, LRIs et
CRC peuvent être conservés comme faits d'interopérabilité documentés. Pour
réduire le risque résiduel, réimplémenter indépendamment le builder de
session/login, le codec Net1/Data2+ si l'objectif est maximal, et surtout le
parseur de records, puis refaire les tests hôte et matériels existants.

