# MQTT

Un snapshot JSON QoS 0 retenu est publié après chaque acquisition utilisable :

```text
<prefix>/inv1
<prefix>/inv2
<prefix>/inv3
```

Le préfixe par défaut est `smaesp`. Les unités sont explicites dans les noms :
watts, volts, ampères, hertz, degrés Celsius, kWh et heures.

Champs principaux : versions de schéma/firmware, timestamp local, identité SMA,
statut/relais, énergie, puissances AC/DC, tensions/courants, fréquence,
température et compteurs horaires.

- une mesure SMA zéro valide est publiée comme nombre `0` ;
- une valeur indisponible ou non acquise est publiée `null` ;
- `data_valid` indique une acquisition exploitable ;
- `data_complete` distingue un dataset complet d'un dataset nocturne partiel ;
- `acquisition_result` vaut `success` ou `partial` ;
- un échec complet ne remplace jamais le snapshot retenu précédent.

Le schéma MQTT reste en version **2**. Le signal SMA Bluetooth nécessite une
transaction distincte non validée et reste `null` ; le RSSI ESP n'est pas
utilisé comme substitut.
