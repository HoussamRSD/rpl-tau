# Documentation des Nouvelles Fonctions OF-TAU - Version2-NoRL (`rpl-dag.c`)

Ce document explique le rôle de chaque nouvelle fonction ajoutée dans et pour le fichier `rpl-dag.c` pour la mise en œuvre du protocole de routage **OF-TAU** dans sa variante sans agent dynamique (Version "NoRL" à poids constants). L'idée est de collecter, normaliser et évaluer de multiples métriques de qualité d'un nœud et d'un lien physique pour la prise de décision.

## 1. Métriques Locales Normalisées (Scores de 0 à 1000)

Ces fonctions normalisent les différentes valeurs physiques afin de les amener sur une échelle comparative de `0` (score pire/nul) à `1000` (score excellent).

*   `degree_norm(void)` : **Degré de Connectivité**
    *   **Rôle :** Évalue le nombre de parents et voisins valides visibles. L'objectif est de repérer et privilégier les nœuds avec une forte redondance radio (zones très connectées).
    *   **Plage :** de 0 (nœud isolé) à 1000 (zone de haute densité, bridé mathématiquement dès 20 voisins).

*   `npc_norm(void)` : **Nombre de Changements de Parent (NPC)**
    *   **Rôle :** Estime l'instabilité relative du nœud. Plus un nœud change souvent de parent, plus il est pénalisé pour éviter d'attirer ses propres enfants dans une instabilité structurelle (réduit l'effet "Parent Lock-in" et l'oscillation).
    *   **Plage :** 0 (extrêmement stable) à 1000 (instable, atteint dès 25 changements).
    *   *Note : Dans les calculs d'incorporation globaux, on utilisera `1000 - NPC` pour que "fort" signifie "bon/stable".*

*   `rpl_etx_norm(rpl_parent_t *p)` : **Signal de Qualité de Lien MAC (ETX)**
    *   **Rôle :** Transforme la métrique standard locale ETX (Expected Transmission Count, nombre de retransmissions théorique). L'ETX brut original est inversé pour en devenir un "score d'excellence".
    *   **Plage :** 1000 (lien parfait, 1 tentative requise, ETX = 1.0) jusqu'à 0 (lien inexploitable, ETX ≥ 8.0).

*   `rpl_rssi_norm(rpl_parent_t *p)` : **Puissance Transceiver Entrante (RSSI)**
    *   **Rôle :** Interroge les paquets réseaux interceptés pour lire la puissance du signal radio d'un voisin. Favorise la stabilité spatiale et la proximité (essentiel face à la mobilité).
    *   **Plage :** 0 (faible connexion ≤ -100 dBm) à 1000 (connexion propre ≥ -40 dBm).

## 2. Le Moteur Mathématique OF-TAU de Décision

*   `rpl_tau_compute_cand(uint16_t ETX_n, uint16_t RSSI_n, uint16_t tau_parent)`
    *   **Rôle :** Fonction arithmétique déterminant le score d'adoption d'un parent spécifique (qui de ses voisins sera le `preferred_parent`). 
    *   **Mécanique :** Combine le score reçu par DIO en "propagations en cascade" (`tau_parent`) et ajoute l'évaluation physique locale des sauts (`ETX_n` et `RSSI_n`). Dans la version NoRL, ses poids coefficients (`W_ETX`, `W_RSSI`, `W_TAU`) sont constants.
    *   **Résultat :** Donne un score `tau_cand` entre 0 et 1000 pour classer les parents.

## 3. Le Processus d'Amnésie et de Modération (Timers)

La pénalisation (NPC) nécessite un mécanisme permettant aux nœuds qui s'immobilisent plus tard de redevenir valides après avoir cessé leurs mouvements.

*   `rpl_pe_npc_reset(void)`
    *   **Rôle :** Purge pure et simple du compteur d'instabilités (Remet `parent_switches` à 0).

*   `handle_npc_reset_timer(void *ptr)`
    *   **Rôle :** Sous-routine callback activée par un `ctimer` logiciel matériel. Appelle l'amnésie par `rpl_pe_npc_reset()` et se répète perpétuellement via une nouvelle planification `ctimer_set`.

*   `rpl_pe_on_parent_switch(void)`
    *   **Rôle :** Rattaché à l’évènement de `rpl_set_preferred_parent`. S'exécute à chaque saut de route dans l'arborescence, incrémente les punitions `parent_switches`, et lance la boucle de minuterie la toute *première* fois que cela arrive.

## 4. Pré-Publication de Signalisation : Le "Piggybacking" DIO

*   `rpl_pe_update_local(rpl_instance_t *instance)`
    *   **Rôle :** Cette fonction encapsule la synthèse de l'état du nœud juste avant de diffuser les paquets informatifs `DIO` au reste du réseau IoT.
    *   **Mécanique :** Combine les scores locaux structurels (Degré `deg_n` et l'inverse instabilité `1000 - npc_n`) avec le coût du chemin du `preferred_parent` au travers de son algorithme de pondérations constantes (`W_PATH`, `W_DEG`, `W_NPC`). L'unique score calculé est écrasé dans la variable globale `rpl_pe_Tau` qui sera ensuite poussée via le contrôleur d'envoi. Si le nœud est le Root, il envoie constamment 1000. ("Document Title":"IoT" OR "Internet of Things" OR "LLN" OR "Low-Power and Lossy Networks" OR "Constrained Networks" OR "Abstract":"IoT" OR "Internet of Things" OR "LLN" OR "Low-Power and Lossy Networks" OR "Constrained Networks" OR "Author Keywords":"IoT" OR "Internet of Things" OR "LLN" OR "Low-Power and Lossy Networks" OR "Constrained Networks") AND ("Document Title":"multicast" OR "MPL" OR "multicast routing" OR "multicast protocol" OR "Abstract":"multicast" OR "MPL" OR "multicast routing" OR "multicast protocol" OR "Author Keywords":"multicast" OR "MPL" OR "multicast routing" OR "multicast protocol") 
