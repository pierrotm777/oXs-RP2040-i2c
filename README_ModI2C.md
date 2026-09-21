# README_ModI2C.md — Adaptations I²C oXs / RP2040-Zero

**Compte rendu des travaux RadioLink → HITEC → Spektrum X-Bus**  
**Projet :** oXs sur RP2040-Zero, Pico SDK 2.1.0, télémétrie vers TX16S / EdgeTX.  
**État au 20 septembre 2026 :** RadioLink et HITEC ont fait l'objet de correctifs et d'essais ; Spektrum X-Bus fonctionne sur AR6610T et son démarrage simultané avec le RP2040 semble désormais reproductible.

> **Portée du document.** Il décrit les corrections, ajouts, fichiers sources concernés et résultats des essais. Ce n'est **pas** un diff garanti du dossier final sur le PC : vérifier que les derniers fichiers effectivement validés regroupent toutes les corrections indiquées. Les essais abandonnés sont distingués de la solution retenue.

## 1. RadioLink — `PROTOCOL=R`

### 1.1. Adaptation initiale en esclave I²C RP2040

- Création/adaptation du transport RadioLink autour de `src/rlink.cpp`, `src/rlink.h` et du pilote Pico SDK existant `src/i2c_slave.cpp/.h` ; intégration dans `main.cpp` par `setupRlink()` et `handleRlink()`.
- Utilisation de **I²C0 en mode esclave, adresse 7 bits `0x04`**. Correspondance de configuration : **`TLM = SDA0`**, **`PRI = SCL0`**, avec `PRI = TLM + 1` sur une paire valide. Exemples : GP0/GP1 ou GP8/GP9 (ce dernier couple est celui du montage documenté pour HITEC/X-Bus).
- Deux trames RadioLink de **16 octets**, de préfixes `89 CD` et `89 AB` ; données reconstruites à partir des champs oXs `fields[]`. Le premier paquet envoyé est **`89 CD`**, puis alternance avec `89 AB`.

### 1.2. Correction du blocage I²C et cohérence des trames

**Fichier corrigé :** `src/rlink.cpp`. Le premier essai de transport ne garantissait pas une réponse correcte à chaque demande de lecture. Corrections apportées :

- **Un octet écrit par événement `I2C_SLAVE_REQUEST`** avec `i2c_write_byte_raw()`, au lieu de tenter de remplir les 16 octets en une seule fois dans l'interruption.
- **Aucune attente bloquante dans l'IRQ** ; les données sont préparées hors interruption dans `handleRlink()`.
- Copie figée des 16 octets au commencement de chaque lecture : le maître ne reçoit pas une trame mélangeant deux actualisations.
- Alternance `CD`/`AB` **à la fin de la transaction** (`I2C_SLAVE_FINISH`), pas à chaque octet ; comptage des lectures complètes ou partielles.
- Vidage des éventuelles écritures reçues, diagnostic du port/adresse au lancement et compteurs avec `DEBUGTLM=Y`.

La réponse `0x00` au-delà de 16 octets est une mesure de protection du pilote de cet essai ; la lecture normale attendue reste **16 octets**.

### 1.3. Corrections RadioLink GPS

**Fichier corrigé :** `src/rlink.cpp`. Corrections apportées :

- L'ordre des coordonnées dans la trame `89 CD` : **longitude avant latitude**.
- Le diagnostic USB des coordonnées : division en virgule flottante par `10000000.0`, au lieu d'une division entière avant conversion en `float`.

**Attention :** vérifier que le `src/rlink.cpp` conservé possède **à la fois** les deux corrections GPS et le mécanisme IRQ non bloquant.

### 1.4. Correction des paramètres pour les trois protocoles I²C

Dans `src/param.cpp` :

- Rectification du texte d'aide : **PRI = SCL0** (`1, 5, 9, 13`) et **TLM = SDA0** (`0, 4, 8, 12`).
- Correction d'une condition utilisant `||` au lieu de `&&` : les restrictions de broches UART de `PRI`/`TLM` ne doivent **pas** exclure par erreur les protocoles `R`, `X`, `T`.
- Validation générale adaptée à ces trois protocoles, avec vérification de la paire I²C0 : `PRI == TLM + 1` lorsque les broches sont définies.
- Garde supplémentaire dans `setupRlink()` pour refuser une paire incorrecte.

Ce correctif concerne la **configuration commune** ; il a été introduit pendant les essais RadioLink puis réutilisé pour HITEC et Spektrum X-Bus.

### 1.5. Sauvegarde en mémoire Flash — correction commune issue des essais RadioLink

**Fichier corrigé :** `src/param.cpp`. Cette modification n'est **pas spécifique au protocole RadioLink** : elle protège `SAVE`, les séquenceurs et le mélangeur gyroscopique pour tout oXs.

**Problème :** le décalage historique `FLASH_CONFIG_OFFSET = 256 * 1024` risquait de tomber dans l'image du firmware devenue plus volumineuse. Écrire `SAVE` à cet endroit pouvait effacer/programmer une portion du programme et provoquer un échec de démarrage.

**Nouvelle implantation, pour une RP2040-Zero disposant de 2 Mio de Flash :** les **trois derniers secteurs de 4 Kio** sont réservés. Ils sont consécutifs et distincts :

| Élément sauvegardé | Décalage depuis le début de la Flash | Taille réservée |
|---|---:|---:|
| Configuration `config` | `0x1FD000` | 4 Kio |
| Séquenceurs `seq` | `0x1FE000` | 4 Kio |
| Mélangeur gyroscopique `gyroMixer` | `0x1FF000` | 4 Kio |

Formule de base du code :

```cpp
#define OXS_FLASH_BYTES (2u * 1024u * 1024u)
#define FLASH_CONFIG_OFFSET (OXS_FLASH_BYTES - 3u * FLASH_SECTOR_SIZE)
```

**Autres protections ajoutées :**

- `static_assert(sizeof(config) <= FLASH_PAGE_SIZE)` : la configuration doit tenir dans la page de **256 octets** prévue pour son écriture.
- Pour `seq` et `gyroMixer`, taille programmée **arrondie au multiple de 256 octets**, tampon rempli avec `0xFF`, puis copie des données utiles.
- Vérification à la compilation que chaque bloc tient dans son secteur de **4 Kio** ; `flash_range_program()` reçoit une longueur conforme aux contraintes des pages Flash.
- Conservation des précautions existantes autour de l'écriture : mise en sécurité du cœur 1, interruptions suspendues pendant l'effacement/la programmation, puis restauration.

**Consignes impératives après cette modification :** installer d'abord le **nouvel UF2 en mode BOOTSEL**, puis lancer `SAVE` seulement avec ce firmware. Les anciennes sauvegardes placées à `256 Kio` ne sont **pas migrées automatiquement** vers les nouvelles adresses : reconfigurer et sauvegarder si nécessaire. Cette implantation suppose une carte **2 Mio** et une image firmware qui **n'empiète pas sur les 12 derniers Kio** ; à recontrôler en cas d'augmentation importante du programme.

## 2. HITEC — `PROTOCOL=T`

### 2.1. Émulation de l'ensemble HTS-SS

**Fichiers :** `src/hitec.cpp` et `src/hitec.h`, appels `setupHitec()` / `handleHitec()` dans `main.cpp`.

- **I²C0 matériel, esclave `0x08`, 100 kHz**, sur **GP8 = SDA / TLM=8** et **GP9 = SCL / PRI=9** pour notre carte.
- L'**I²C1** oXs destiné aux capteurs physiques reste distinct (par exemple GP10/GP11 selon configuration) ; pas de remplacement des pilotes de ces capteurs.
- Constitution de trames HITEC de **7 octets** et prise en charge des familles `0x11` à `0x1B` pertinentes.
- Préparation périodique des trames à partir de `fields[]` (environ **100 ms**) ; double banque actif/inactif pour que l'IRQ lise une version stable.
- Deux modes prévus par `HITEC_TEST_VALUES` : **`1`** pour les valeurs fixes de banc, **`0`** pour les mesures oXs et les valeurs simulées `FVP`/`FVN`. L'en-tête de la version finale distribuée a `HITEC_TEST_VALUES=0` par défaut.

### 2.2. Mise au point des transactions sur Optima 7

**Fichier corrigé :** `src/hitec.cpp`. Les captures de l'ensemble HTS-SS d'origine et les essais sur l'Optima 7 ont permis de corriger plusieurs tentatives intermédiaires :

- Ajout de la trame d'identification/état **`0x11`** et reproduction, en mode de test exact, du cycle capturé : `11,12,13,14,15,16,17,18,12,13,14,12,13,14` (hexadécimal).
- Analyse des lectures présentant une **impulsion/horloge supplémentaire** après les sept octets utiles. Les essais de réinitialisation différée de l'esclave et différentes réponses au huitième octet ont conduit à conserver la solution validée : **précharger sept octets + `0xFF` dans la FIFO TX dès la demande**.
- Écriture directe dans `DATA_CMD` **seulement si la FIFO offre de la place**, sans attente, `printf`, ni réinitialisation I²C dans l'IRQ ; prise en compte des transactions incomplètes et du départ d'une nouvelle interrogation lorsque le STOP n'est pas signalé comme attendu.
- Les compteurs de diagnostic accessibles par `DEBUGTLM=Y` restent hors IRQ ; la variante finale retire les impressions temporaires de mesure du temps d'interruption.
- **Pull-up externes 1 kΩ vers 3,3 V sur SDA et SCL** : nécessaires dans le montage Optima 7 testé. Ne pas en déduire qu'il faut les réutiliser sur Spektrum.

Les tentatives de « reset après 5 ms », d'octet `FF` sans reset et d'autres ajustements de l'impulsion supplémentaire **ne sont pas des changements à cumuler**. La version retenue est celle de `src/hitec.cpp` avec FIFO préchargée et sans instrumentation temporelle temporaire.

### 2.3. Conversion des données et corrections EdgeTX

**Fichier corrigé :** `src/hitec.cpp`.

- Coordonnées GPS oXs signées en **degrés × 10⁷** converties en format HITEC en **arithmétique entière** pour ne pas perdre de précision sur le RP2040.
- Corrections des octets de **date `YY/MM/DD`** dans la trame `0x16` ; l'**heure/minute** y sont également envoyées, tandis que les **secondes GPS** appartiennent à `0x12`, conformément au décodage HITEC EdgeTX.
- Mise en forme des températures, vitesse sol/air, RPM, cap, satellites, tension, courant et altitude selon les trames utilisées ; notamment décalage HITEC de tension et unité du courant vérifiés lors des essais.
- Pour l'altimètre barométrique `0x1B`, **arrondi symétrique des centimètres en mètres**, y compris en négatif ; pas d'invention d'un champ VSPEED inexistant dans cette trame de sept octets.

### 2.4. Simulations FVP/FVN communes

**Fichier corrigé :** `src/tools.cpp`.

Une ancienne branche de `fillFields(1)` (`FVP`) n'injectait qu'un **sous-ensemble de champs**, notamment GPS : les autres capteurs HITEC/X-Bus ne reflétaient donc pas toujours les valeurs simulées. Le correctif parcourt **tous les `posFieldValues[]`**, comme `FVN` parcourt ses valeurs négatives ; les champs sont marqués disponibles et réappliqués pendant la simulation. Cela évite notamment que des mesures physiques (par exemple V3/V4) écrasent durablement les températures simulées.

Ce changement concerne **les valeurs de test partagées par les protocoles** ; il n'est pas une modification du bus I²C.

## 3. Spektrum X-Bus — `PROTOCOL=X`

### 3.1. Nouveau transport I²C mult-adresses, distinct de SRXL2

**Fichiers ajoutés ou adaptés :** `src/xbus.cpp`, `src/xbus.h`, `src/i2c_multi.c`, `src/i2c_multi.h`, `src/i2c_multi.pio`, `src/main.cpp` et `CMakeLists.txt`. Récepteur testé : **Spektrum AR6610T**.

- Réutilisation des **identifiants, structures et formats de capteurs existants dans `srxl_sensors.h`** pour construire la charge utile Spektrum.
- **SRXL2 reste le transport UART half-duplex distinct** : X-Bus répond, lui, aux lectures du récepteur sur plusieurs adresses I²C.
- Adoption du pilote **MSRC `i2c_multi` sur PIO0**, adapté au projet oXs : dépendances d'en-têtes allégées, liaison C/C++, et **tampon TX statique** car le pilote continue de lire les octets après le retour du callback.
- Format de capteur : **16 octets**, avec identifiant/adresse, identifiant secondaire et données ; copies protégées vis-à-vis des IRQ et données calculées en dehors des interruptions.
- Fonction `handleXbusSpektrum()` **sans adresse en argument** : le callback du pilote détermine lui-même l'adresse interrogée et sélectionne la trame.

**Intégration CMake nécessaire** (à conserver dans le projet où le pilote est réellement installé) :

```cmake
pico_generate_pio_header(oXs ${CMAKE_CURRENT_LIST_DIR}/src/i2c_multi.pio)
target_sources(oXs PRIVATE src/i2c_multi.c)
```

`hardware_pio` est déjà lié dans la base consultée. **Attention : les quatre machines d'état de PIO0** sont utilisées par ce pilote ; la cohabitation avec SBUS OUT, réception ESC, PWM/PIO0 ou d'autres consommateurs de PIO0 n'est **pas automatiquement garantie**. Le prototype refuse explicitement certains conflits (`SBUS_OUT`, `ESC`), pas tous.

### 3.2. Adresses mises en service

| Adresse 7 bits | Capteur présenté au récepteur |
|---|---|
| `0x11` | Vitesse air |
| `0x16` | Position GPS |
| `0x17` | État GPS, heure, satellites |
| `0x18` | Courant, tension, capacité récepteur |
| `0x20` | ESC : RPM, tension, courant, températures, etc. |
| `0x40` | Variomètre / altitude |
| `0x7E` | RPM, tension, température |

**Adresse `0x34` non ajoutée** dans cette version. Les données de chaque capteur sont préparées depuis `fields[]` toutes les **100 ms** environ. `DEBUGTLM=Y` affiche, toutes les **~2 s**, le nombre de demandes reçues pour chacune des sept adresses.

### 3.3. Corrections électriques et validation du bus

- Après analyse des captures Logic, **correction d'une confusion SDA/SCL** : les fils doivent suivre les **signaux physiques**, non les étiquettes initiales du décodeur ; pour le code validé : **GP8/TLM = SDA** et **GP9/PRI = SCL**.
- Sur ce récepteur, les lignes mesurées au repos étaient autour de **3,3 V sans les deux résistances externes de 1 kΩ**. Ces résistances ont été retirées pendant les essais Spektrum ; le RP2040 a alors pu démarrer normalement après le récepteur. Les 1 kΩ exigées par l'Optima 7 ne sont donc **pas une règle universelle**.
- Des essais récepteur alimenté et RP2040 éteint avaient montré **environ 1,7 V sur le rail 3,3 V du RP2040** : éviter les montages provoquant cette alimentation parasite ; ne jamais appliquer le 5 V sur un GPIO RP2040.
- Lorsque **RP2040 démarré avant AR6610T**, télémétrie reçue et `FVP` modifiant réellement les valeurs affichées. Les compteurs des adresses `0x11`, `0x16`, `0x17`, `0x18` et `0x20` augmentaient ensemble ; `0x40` était vu une fois, `0x7E` pas interrogé dans les relevés fournis. Ce dernier point ne signifie pas à lui seul que ces capteurs sont défectueux.

### 3.4. Résolution du démarrage simultané récepteur / RP2040

**Problème initial :** dans le montage habituel, **le récepteur fournit le 5 V au RP2040**. Le récepteur terminait apparemment sa découverte X-Bus avant que l'esclave I²C du RP2040 soit initialisé. Démarrer artificiellement le RP2040 avant le récepteur fonctionnait ; démarrer ensemble ne fonctionnait pas.

**Fichier corrigé :** `src/main.cpp`. **Solution retenue :** démarrage anticipé conditionné exclusivement à X-Bus :

1. Exécuter `setupConfig()` plus tôt dans `setup()` pour connaître le protocole **avant** la pause USB de `DEBUG`.
2. **Uniquement pour `config.protocol == 'X'`**, sauter l'attente de connexion USB (jusqu'à ~1 s) et les **2 secondes** supplémentaires du mode `DEBUG`. Les autres protocoles conservent cette temporisation.
3. Préparer `fields[]` et les deux queues nécessaires, puis appeler **`setupXbusSpektrum()` avant `multicore_launch_core1(core1_main)`**, donc avant la longue découverte des capteurs sur le cœur 1.
4. Retirer l'ancien appel X-Bus situé après l'attente de fin du cœur 1 : **une seule initialisation X-Bus**. La boucle conserve `handleXbusSpektrum()` sous `PROTOCOL=X`.

Extrait du nouvel emplacement :

```cpp
// Après initialisation de fields[] et des queues, AVANT le cœur 1 :
if (config.protocol == 'X') {
    setupXbusSpektrum();
}
multicore_launch_core1(core1_main);
```

**Résultat communiqué :** après remplacement du `main.cpp`, la télémétrie **semble fonctionner à chaque mise sous tension conjointe**, récepteur alimentant le RP2040. C'est une validation d'usage rapportée par Pierrot ; il n'a pas été fourni de campagne chiffrée de centaines de démarrages.

**Le 2N2222 de retenue de SCL et le forçage logiciel de GP9 à LOW ne sont PAS intégrés** à la solution retenue. Aucun changement n'a été requis dans les trames X-Bus ou `i2c_multi` pour résoudre cette séquence de démarrage.

### 3.5. Date/heure GPS EdgeTX et limites documentées

- L'analyse du `spektrum.cpp` EdgeTX fourni a montré que la trame GPS `0x17` fournit **l'heure UTC**, tandis que le capteur `UNIT_DATETIME` construit sa **date à partir de l'horloge interne de la radio**. La date `2000-01-01` provenait du réglage de date de la TX16S, **pas d'un défaut de trame X-Bus** ; après réglage de la date de la radio, le résultat correspondait.
- La capacité encodée sur `0x18` reste limitée à environ **3276,6 mAh** dans le champ de 0,1 mAh du prototype ; une simulation `FVP` de **34 321 mAh** est **saturée et non transmise intégralement**. L'extension `highCharge` ou un autre type de capteur reste à étudier si nécessaire.
- Pour `0x40`, seule la valeur de variation liée à l'intervalle **1 seconde** est remplie avec `VSPEED` ; les autres deltas temporels non calculés restent indisponibles.

## 4. Référence rapide — fichiers sources concernés et précautions

| Fichier / groupe | Rôle |
|---|---|
| `src/param.cpp` | Aide et vérification `PRI/TLM` pour R/X/T ; **implantation Flash/SAVE 2 Mio**. |
| `src/main.cpp` | Sélection du protocole ; **démarrage anticipé conditionné à X-Bus**. |
| `src/rlink.cpp`, `src/rlink.h` | RadioLink esclave I²C `0x04`, alternance 16 octets ; vérifier que les correctifs GPS et IRQ sont réunis. |
| `src/hitec.cpp`, `src/hitec.h` | HITEC `0x08`, FIFO `7 + FF`, GPS/date/altitude, mode test/live. |
| `src/xbus.cpp`, `src/xbus.h` | Capteurs Spektrum multiples, formats partagés avec SRXL2, données depuis `fields[]`. |
| `src/i2c_multi.c/.h/.pio` | Esclave I²C multi-adresses sur **PIO0** pour X-Bus. |
| `src/tools.cpp` | `FVP` appliqué à tous les champs de test, cohérent avec `FVN`. |
| `CMakeLists.txt` | Génération du header PIO `i2c_multi.pio` et compilation de `i2c_multi.c`. |

### Configuration et câblage retenus pour HITEC / X-Bus

```text
PROTOCOL=T   # HITEC Optima 7      (ou X pour Spektrum AR6610T)
TLM=8        # GP8  = SDA
PRI=9        # GP9  = SCL
# Masse commune ; vérifier le niveau 3,3 V des lignes I²C.
```

**Ne pas généraliser le même montage de pull-up** : l'Optima 7 a fonctionné avec **1 kΩ sur chaque ligne vers 3,3 V** ; sur l'AR6610T les résistances externes ont été retirées pendant la mise au point. Pour Spektrum, conserver le **démarrage anticipé dans `main.cpp`**, pas un pont GPIO–5 V ni une retenue SCL non testée.

**Conserver systématiquement le dernier projet complet et l'UF2 dont le démarrage et les capteurs ont été vérifiés, avant toute nouvelle modification.**
