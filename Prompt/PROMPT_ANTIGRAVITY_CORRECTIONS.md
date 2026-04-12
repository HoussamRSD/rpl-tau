# PROMPT TECHNIQUE — Corrections OF-TAU pour RPL/Contiki-Cooja
## Contexte et Résultats Actuels
L'implémentation actuelle donne:
- **Topology 1** (dynamique) : PDR = **46.13%** ± 2.85 | Parent Changes = **94.7** ± 17.4
- **Topology 2** (stable)   : PDR = **93.30%** ± 0.26 | Parent Changes = **0.0**

La Topology 2 fonctionne bien car le réseau est statique — la OF-TAU est correcte au fond. Le problème principal est l'**instabilité en Topology 1** : 94 changements de parent en moyenne signifie qu'un paquet sur deux est perdu pendant les reconvergences DODAG.

---

## DIAGNOSTIC : 5 Bugs Critiques Identifiés

### Bug #1 — CRITIQUE : Absence de lissage (EMA) sur `tau_cand`
**Fichier : `rpl-dag.c`**

`tau_cand` est recalculé à chaque DIO reçu avec les valeurs BRUTES instantanées de QL, ETX, RSSI. Ces métriques fluctuent très rapidement (QL change à chaque paquet routé, ETX change à chaque transmission). Résultat : `tau_cand` oscille, le seuil d'hystérésis est franchi répétitivement → ping-pong de parents.

### Bug #2 — CRITIQUE : L'hystérésis compare le `tau_cand` ACTUEL, pas le tau de référence
**Fichier : `rpl-of-tau.c`**

Code actuel dans `best_parent()`:
```c
if(t2 <= (uint16_t)(t1 + RPL_OF_TAU_SWITCH_THRESHOLD)) return p1;
```
`t1` est le tau COURANT du parent préféré (qui a déjà chuté à cause de la surcharge). Exemple concret du ping-pong :
- Parent A sélectionné avec tau=650
- A se surcharge → tau_cand(A) chute à 150
- Challenger B : tau=480 > 150+300=450 → switch vers B
- B se surcharge → tau_cand(B) chute à 150
- A récupère → tau_cand(A)=600 > 150+300=450 → retour vers A
- **Cycle infini** malgré THRESHOLD=300

La vraie protection anti ping-pong doit comparer contre le **tau au moment de la sélection**, pas le tau courant.

### Bug #3 — IMPORTANT : W_TAU=5 trop élevé → Effet cascade
**Fichier : `project-conf.h`**

Avec W_TAU=5/21 ≈ 24% du score, quand un nœud change de parent, son propre `tau_cand` chute. Ses enfants reçoivent le DIO avec un `pe_Tau` réduit → leur `tau_cand` pour ce parent chute → ils switchent aussi → **cascade en arbre** qui peut dépeupler une branche entière du DODAG.

### Bug #4 — IMPORTANT : Boucle de rétroaction QL ↔ Children
**Fichier : `rpl-dag.c`** — fonction `queue_load_norm()`

La QL mesure `uip_ds6_route_num_routes()` avec max=10 routes. Donc chaque enfant qui s'enregistre ajoute 100 points de QL. Avec W_QL=2 et wsum=21 : delta_tau = -2*100/21 ≈ -9.5 par enfant. À 8 enfants, QL=800, contribution tau = 2*(1000-800)/21 ≈ 19. Le nœud devient peu attractif → les enfants partent → QL chute → tau remonte → les enfants reviennent. **Oscillation classique**.

### Bug #5 — MODÉRÉ : RPL_TAU_NPC_PERIOD trop long (300 secondes)
**Fichier : `rpl-dag.c`**

Un nœud instable reste pénalisé 5 minutes. Dans une topologie dynamique, c'est trop long. Le nœud peut s'être stabilisé après 30 secondes mais reste blacklisté. Les enfants qui l'avaient quitté ne peuvent pas y revenir.

---

## MODIFICATIONS REQUISES — Instructions précises

### MODIFICATION 1 : `rpl.h` — Ajouter les champs de lissage dans la structure parent

Dans la struct `rpl_parent` (après la ligne `uint16_t tau_cand;`), ajouter :

```c
  /* τ_cand = F(PE_u, τ_u, P_lien(i,u))  — computed locally */
  uint16_t tau_cand;

  /* ---- Anti-oscillation fields (NEW) ---- */
  uint16_t tau_at_selection;    /* tau_cand at the moment this parent was chosen → hysteresis baseline */
  clock_time_t tau_last_switch; /* clock_time when we last switched TO a new preferred parent */
```

---

### MODIFICATION 2 : `rpl-dag.c` — Appliquer l'EMA sur `tau_cand`

Trouver les deux endroits où `tau_cand` est calculé (lignes ~1041 et ~1956). Remplacer les blocs :

**AVANT (les deux occurrences) :**
```c
p->tau_cand = rpl_tau_compute_cand(
    p->pe_RE, p->pe_QL, p->pe_Deg, p->pe_NPC,
    rpl_etx_norm(p), rpl_rssi_norm(p), p->pe_Tau);
```

**APRÈS :**
```c
{
  uint16_t new_tau = rpl_tau_compute_cand(
      p->pe_RE, p->pe_QL, p->pe_Deg, p->pe_NPC,
      rpl_etx_norm(p), rpl_rssi_norm(p), p->pe_Tau);
  /* EMA lissage : alpha=3/8 (favorise passé).
   * Empêche les oscillations de tau_cand dues aux fluctuations rapides de QL/ETX. */
  if(p->tau_cand == 0) {
    p->tau_cand = new_tau;
  } else {
    p->tau_cand = (uint16_t)((3UL * new_tau + 5UL * (uint32_t)p->tau_cand) / 8UL);
  }
}
```

---

### MODIFICATION 3 : `rpl-dag.c` — Lissage du tau propagé dans les DIO

Dans `rpl_pe_update_local()`, le nœud diffuse son propre `tau_cand` comme `rpl_pe_Tau`. Ajouter un EMA global pour stabiliser la valeur diffusée. Remplacer la section complète :

**AVANT :**
```c
  rpl_pe_Tau = 1000;
  ...
  if(dag != NULL && dag->rank != ROOT_RANK(instance)
     && dag->preferred_parent != NULL) {
    rpl_pe_Tau = dag->preferred_parent->tau_cand;
    if(rpl_pe_Tau > 1000) rpl_pe_Tau = 1000;
    ...
  }
```

**APRÈS :**
```c
  /* rpl_pe_Tau_smooth: variable statique pour le lissage du tau diffusé */
  static uint16_t rpl_pe_Tau_smooth = 0;

  if(dag == NULL || dag->rank == ROOT_RANK(instance)) {
    rpl_pe_Tau_smooth = 1000;
  } else if(dag->preferred_parent != NULL) {
    uint16_t raw_tau = dag->preferred_parent->tau_cand;
    if(raw_tau > 1000) raw_tau = 1000;
    /* EMA alpha=1/4 : très conservateur pour le tau diffusé, évite la cascade */
    if(rpl_pe_Tau_smooth == 0) {
      rpl_pe_Tau_smooth = raw_tau;
    } else {
      rpl_pe_Tau_smooth = (uint16_t)((2UL * raw_tau + 6UL * (uint32_t)rpl_pe_Tau_smooth) / 8UL);
    }
    current_etx  = rpl_etx_norm(dag->preferred_parent);
    current_rssi = rpl_rssi_norm(dag->preferred_parent);
  }
  rpl_pe_Tau = rpl_pe_Tau_smooth;
```

---

### MODIFICATION 4 : `rpl-dag.c` — Réduire NPC reset period

**AVANT :**
```c
#ifndef RPL_TAU_NPC_PERIOD
#define RPL_TAU_NPC_PERIOD (300 * CLOCK_SECOND)
#endif
```

**APRÈS :**
```c
#ifndef RPL_TAU_NPC_PERIOD
#define RPL_TAU_NPC_PERIOD (60 * CLOCK_SECOND)  /* 1 minute au lieu de 5 */
#endif
```

---

### MODIFICATION 5 : `rpl-of-tau.c` — Corriger l'hystérésis (comparer contre tau_at_selection)

**AVANT** (dans `best_parent()`) :
```c
  if(dag != NULL && (p1 == dag->preferred_parent || p2 == dag->preferred_parent)) {
    rpl_parent_t *pref = dag->preferred_parent;
    if(pref == p1) {
      if(t2 <= (uint16_t)(t1 + RPL_OF_TAU_SWITCH_THRESHOLD)) {
        return p1;
      }
    } else if(pref == p2) {
      if(t1 <= (uint16_t)(t2 + RPL_OF_TAU_SWITCH_THRESHOLD)) {
        return p2;
      }
    }
  }
```

**APRÈS :**
```c
  if(dag != NULL && (p1 == dag->preferred_parent || p2 == dag->preferred_parent)) {
    rpl_parent_t *pref = dag->preferred_parent;
    rpl_parent_t *challenger = (pref == p1) ? p2 : p1;

    /* Cooldown anti-retour : ne pas switcher si on vient de switcher il y a moins de
     * RPL_OF_TAU_COOLDOWN_SEC secondes. Brise les cycles d'oscillation rapide. */
#define RPL_OF_TAU_COOLDOWN_SEC  30
    clock_time_t now = clock_time();
    clock_time_t elapsed = now - pref->tau_last_switch;
    if(elapsed < (clock_time_t)(RPL_OF_TAU_COOLDOWN_SEC * CLOCK_SECOND)) {
      return pref; /* Cooldown actif : garder le parent actuel inconditionnellement */
    }

    /* Comparer contre tau_at_selection (pas tau courant) pour l'hystérésis.
     * Cela empêche qu'un parent dégradé perde sa protection hystérésis. */
    uint16_t ref_tau = pref->tau_at_selection;
    uint16_t chall_tau = (challenger == p1) ? t1 : t2;

    if(chall_tau <= (uint16_t)(ref_tau + RPL_OF_TAU_SWITCH_THRESHOLD)) {
      return pref; /* Le challenger n'est pas assez meilleur que la référence */
    }
    /* Le challenger gagne. Enregistrer les métadonnées anti-oscillation. */
    challenger->tau_at_selection = chall_tau;
    challenger->tau_last_switch  = now;
    return challenger;
  }
```

**Attention** : Ce bloc remplace entièrement l'ancienne vérification d'hystérésis. La variable `challenger` est définie localement ; `pref` existe déjà dans le bloc. Bien supprimer l'ancien `if(pref==p1)... else if(pref==p2)...` au complet.

---

### MODIFICATION 6 : `rpl-of-tau.c` — Ajouter inclure clock.h si absent

En haut du fichier, après les autres includes :
```c
#include "sys/clock.h"   /* clock_time(), CLOCK_SECOND */
```

---

### MODIFICATION 7 : `project-conf.h` — Nouveaux poids calibrés

Remplacer entièrement la section des poids :

**AVANT :**
```c
#define W_RE    4
#define W_QL    2
#define W_DEG   1
#define W_NPC   1
#define W_ETX   5
#define W_RSSI  3
#define W_TAU   5
```

**APRÈS :**
```c
/* ═══════════════════════════════════════════════════════════════════════════
 *  POIDS V2 — Calibrés contre l'oscillation de Topology 1
 *
 *  Changements principaux vs V1 :
 *   - W_ETX  7 (+2) : ETX est la métrique la plus stable et fiable. Priorité max.
 *   - W_TAU  2 (-3) : Réduit la cascade. Le tau parent influence moins les enfants.
 *   - W_QL   3 (+1) : Légère hausse pour mieux équilibrer la charge.
 *   - W_NPC  2 (+1) : Pénalise plus les nœuds instables.
 *   - W_RE   3 (-1) : Reste important pour la durée de vie mais cède du poids à ETX.
 *   - W_RSSI 2 (-1) : RSSI redondant avec ETX dans la plupart des cas.
 *   - W_DEG  1 (=)  : Inchangé.
 *  Total wsum = 7+3+1+2+2+2+3 = 20 (simplifie la normalisation)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define W_RE    3   /* a: RE — Énergie résiduelle */
#define W_QL    3   /* b: QL — Charge de routage (anti surcharge) */
#define W_DEG   1   /* c: Deg — Degré de connexion */
#define W_NPC   2   /* d: NPC — Pénalité d'instabilité */
#define W_ETX   7   /* e: ETX — Qualité physique du lien (PRIORITÉ MAX) */
#define W_RSSI  2   /* f: RSSI — Signal radio */
#define W_TAU   2   /* g: Tau parent — Qualité récursive du chemin (réduit cascade) */

/* Hystérésis réduite : avec le tau_at_selection, 75 suffit pour la stabilité
 * sans bloquer les switches légitimes vers de meilleurs parents. */
#undef  RPL_OF_TAU_SWITCH_THRESHOLD
#define RPL_OF_TAU_SWITCH_THRESHOLD  75
```

---

### MODIFICATION 8 : `rpl-dag.c` — Initialiser les nouveaux champs lors de l'ajout d'un parent

Dans la fonction `rpl_add_parent()` (vers la fin de la fonction, après que `tau_cand` est calculé la première fois), ajouter l'initialisation :

```c
  /* Initialiser les champs anti-oscillation */
  p->tau_at_selection = 0;   /* sera mis à jour au premier best_parent() */
  p->tau_last_switch  = 0;
```

---

## RÉSUMÉ DES FICHIERS MODIFIÉS

| Fichier           | Nature du changement                                     |
|-------------------|----------------------------------------------------------|
| `rpl.h`           | +2 champs dans struct rpl_parent (tau_at_selection, tau_last_switch) |
| `rpl-dag.c`       | EMA sur tau_cand (×2 endroits), EMA sur tau diffusé, NPC period 300s→60s, init nouveaux champs |
| `rpl-of-tau.c`    | Hystérésis corrigée (tau_at_selection + cooldown 30s), include clock.h |
| `project-conf.h`  | Nouveaux poids W_*, SWITCH_THRESHOLD 300→75              |

---

## EXPLICATION DE L'IMPACT ATTENDU

| Problème                              | Avant     | Après (estimation)  |
|---------------------------------------|-----------|---------------------|
| Oscillation tau_cand                  | Forte     | Très réduite (EMA)  |
| Ping-pong parent                      | 94.7 /sim | < 20 /sim           |
| PDR Topology 1                        | 46%       | > 70%               |
| Cascade sur changement de parent      | Forte     | Faible (W_TAU=2)    |
| Rétention d'un parent dégradé         | Oui (bug) | Non (tau_at_sel.)   |
| Blacklist NPC trop longue             | 300s      | 60s                 |

---

## NOTE IMPORTANTE SUR LA VALIDITÉ DODAG (Loop-Free)
Les modifications ci-dessus ne touchent **pas** à `rank_via_parent()` ni à la condition de rang dans `best_parent()`. La propriété loop-free du DODAG reste garantie : `Rank(enfant) > Rank(parent)` est vérifiée avant tout choix de parent. Le cooldown de 30 secondes dans le bloc hystérésis est appliqué **après** les filtres de rang, donc aucun risque de boucle.
