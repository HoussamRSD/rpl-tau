# OF-TAU : Moteur d'Apprentissage par Renforcement (RL) - Gradient Bandit

Ce dossier contient l'implémentation `Version4-RL` du protocole RPL intégrant un algorithme d'Apprentissage par Renforcement (RL) appelé **Gradient Bandit**. 
Au lieu d'avoir des poids figés dans le marbre (comme dans des fichiers de configuration de type `project-conf.h`), chaque nœud va désormais "apprendre" de son environnement et ajuster l'importance de chaque métrique en temps réel.

Voici l'analyse bloc par bloc de ce moteur d'intelligence artificielle optimisé pour microcontrôleur, tel qu'implémenté dans cette version.

## 1. Le Frein Temporel (Throttling)
```c
  rl_update_counter++;
  if(rl_update_counter < RL_UPDATE_PERIOD) return;
  rl_update_counter = 0;
```
L'apprentissage machine consomme du CPU et peut rendre un réseau instable s'il réagit à la moindre microseconde. Ce bloc agit comme un frein. Le nœud observe son environnement en continu, mais il ne met à jour ses poids mathématiques que tous les `RL_UPDATE_PERIOD` cycles. Cela laisse le temps à la topologie de se stabiliser avant de juger si le choix du parent était bon ou mauvais.

## 2. Le Vecteur d'État (Features)
```c
  feats[0] = (int32_t)clamp1000(pref->pe_RE);
  feats[1] = (int32_t)(1000 - clamp1000(pref->pe_QL));
  // ...
```
Le nœud construit sa perception du parent sous forme d'un tableau `feats`. 
**Remarque très importante :** L'opération `1000 - ...` a été appliquée sur des métriques comme QL (Queue Load) et NPC. Cela signifie que les données sont harmonisées : pour toutes les variables, une valeur proche de 1000 signifie désormais "C'est excellent". C'est une condition absolue pour que l'algorithme du Gradient fonctionne sans s'emmêler les pinceaux.

## 3. La Récompense et le Filtre Anti-Bruit (Deadband)
```c
  reward = curr_etx_norm - rl_prev_etx_norm;
  abs_reward = reward < 0 ? -reward : reward;
  if(abs_reward < RL_REWARD_DEADBAND) { return; }
```
La récompense est basée sur l'évolution de la qualité physique du lien (ETX).
Le **DEADBAND** est une sécurité vitale (un filtre de bruit). Les ondes radio fluctuent naturellement à cause de l'air ou des obstacles mineurs. Si l'ETX varie juste un tout petit peu (dans la zone morte du deadband), l'agent ignore cette variation pour ne pas ruiner son apprentissage avec du bruit aléatoire.

*(Note : Dans `rpl_etx_norm`, l'ETX est inversé en amont via `1000 - ...` pour qu'un score de 1000 représente un lien parfait. Ainsi, quand le lien radio (ETX brut) s'améliore dans la vraie vie en diminuant, `curr_etx_norm` augmente et le calcul `curr - prev` donne bien un résultat positif récompensant ce bon comportement).*

## 4. Le Masque de Gel (Frozen Mask)
```c
  if(!((RL_FROZEN_MASK >> i) & 1u)) {
    mean_learn += feats[i];
    learn_count++;
  }
```
C'est une partie très élégante du modèle. Dans l'IoT, certaines valeurs peuvent être constantes (par exemple, si le nœud racine est branché sur secteur, son énergie RE sera toujours à 1000). Si l'agent essayait d'apprendre sur une constante, cela fausserait les mathématiques. Le `RL_FROZEN_MASK` permet, bit par bit, de "geler" l'apprentissage sur certaines variables (marquées d'un `*` dans les logs) et de calculer la moyenne `mean_learn` uniquement sur les variables qui bougent vraiment.

## 5. Le Cœur de l'Apprentissage (Gradient Update)
```c
  delta = reward * (feats[i] - mean_learn) / RL_ALPHA_INV;
  rl_w[i] += delta;
```
C'est ici que la magie opère. C'est l'implémentation en arithmétique entière (sans variables flottantes, ce qui est parfait pour l'embarqué) de la formule de mise à jour du Gradient : 

`Δw_i = (R * (x_i - mean_x)) / α_inv`

- Si la récompense $R$ est positive (le réseau va mieux)...
- ...et que la caractéristique $i$ (`feats[i]`) est supérieure à la moyenne des autres caractéristiques (`mean_learn`)...
- ...alors le Delta est positif et le poids de cette caractéristique augmente. 

L'agent se dit : *"Ce paramètre était fort quand j'ai eu ma récompense, je dois lui faire plus confiance à l'avenir !"*.
