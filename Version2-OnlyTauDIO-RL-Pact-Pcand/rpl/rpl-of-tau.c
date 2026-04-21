/*---------------------------------------------------------------------------*/
/* rpl-of-tau.c
 *
 * Objective Function TAU for RPL + Q-Learning Gatekeeper.
 *
 * ARCHITECTURE EN DEUX ETAPES :
 *
 * ETAPE 1 — Filtre MRHOF + TAU (INTOUCHABLE, PDR=98%) :
 *   best_parent() evalue tous les candidats et elu le meilleur "Champion".
 *   tau_cand = F(ETX_norm, RSSI_norm, tau_parent)
 *   Logique : filtres usable-link + rank constraint + path-cost ETX + hysteresis TAU.
 *
 * ETAPE 2 — Juge Q-Learning "Gatekeeper" (SURCOUCHE FINALE) :
 *   S'active UNIQUEMENT si Champion != Parent_Actuel.
 *   L'agent prend l'etat (Parent_Actuel, Champion) et decide :
 *     Action 0 (RESTER)  => on retourne Parent_Actuel (veto du switch).
 *     Action 1 (CHANGER) => on retourne Champion (autorisation du handoff).
 *
 * MEMOIRE RL (Q-Table LRU statique) :
 *   Cache LRU de 20 entrees max, keye par (addr_curr, addr_cand).
 *   20 x 10 bytes = 200 bytes de RAM. O(1) garanti.
 *
 * REWARD PREDICTIF (calcule periodiquement) :
 *   Action 1 (CHANGE)  : R = tau_cand(champ) - tau_cand(ancien_parent)
 *   Action 0 (STAY)    : R = time_with_parent * 5
 *                        Penalite si delta_ETX < -2.0 : R -= 100
 *                        Penalite si ETX >= 5.0       : R -= 150
 *                        Malus rupture si ETX >= 8.0  : R = -500
 *
 * BELLMAN entier : Q(s,a) += (R - Q(s,a)) / ALPHA_INV  [ALPHA_INV=10 => alpha=0.1]
 * EPSILON-GREEDY : 10% d'exploration aleatoire.
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/link-stats.h"
#include "lib/random.h"
#include <limits.h>
#include <string.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/* Fallback if not defined by platform */
#ifndef LINK_STATS_ETX_DIVISOR
#define LINK_STATS_ETX_DIVISOR 128
#endif

/* ==============================================================================
   PARAMETRES OF-TAU (INCHANGES)
   ============================================================================== */

#ifndef RPL_OF_TAU_MAX_ETX
#define RPL_OF_TAU_MAX_ETX (8 * LINK_STATS_ETX_DIVISOR)      /* ETX 8.0 */
#endif
#ifndef RPL_OF_TAU_INIT_ETX
#define RPL_OF_TAU_INIT_ETX (2 * LINK_STATS_ETX_DIVISOR)
#endif
#ifndef RPL_OF_TAU_SWITCH_THRESHOLD
#define RPL_OF_TAU_SWITCH_THRESHOLD 75
#endif
#ifndef RPL_OF_TAU_MIN_TAU
#define RPL_OF_TAU_MIN_TAU 1
#endif
#ifndef RPL_OF_TAU_PANIC_THRESHOLD
#define RPL_OF_TAU_PANIC_THRESHOLD 200
#endif
#ifndef MRHOF_PC_THRESHOLD
#define MRHOF_PC_THRESHOLD 96                                 /* eq ETX 0.75 */
#endif

/* ==============================================================================
   PARAMETRES DU GATEKEEPER Q-LEARNING
   ============================================================================== */

/* Cache LRU */
#define RL_CACHE_SIZE        20   /* Nombre maximal d'entrees LRU        */

/* Actions */
#define RL_ACTION_STAY        0   /* 0 = rester avec parent_actuel        */
#define RL_ACTION_CHANGE      1   /* 1 = autoriser le handoff             */

/* Apprentissage Bellman entier : alpha = 1/ALPHA_INV = 0.1 */
#define RL_ALPHA_INV         10

/* Seuils de reward (ETX brut, LINK_STATS_ETX_DIVISOR = 128) */
#define RL_ETX_FAST_DROP    256   /* delta_ETX < -2.0  => -2 * 128       */
#define RL_ETX_DANGER       640   /* ETX >= 5.0        =>  5 * 128       */
#define RL_ETX_RUPTURE     1024   /* ETX >= 8.0        =>  8 * 128       */

/* Time_with_parent bonus : R += time_with_parent * RL_STAY_BONUS_SCALE */
#define RL_STAY_BONUS_SCALE   5

/* Q-value init et clamp.
 * ATTENTION : init q_change > q_stay pour casser le biais STAY permanent
 * quand les deux valeurs sont a 0 (q_change > q_stay est faux => toujours STAY).
 * On initialise q_change a une petite valeur positive pour forcer l'exploration
 * initiale du changement, puis l'apprentissage prendra le relai. */
#define RL_QVAL_INIT_STAY     0
#define RL_QVAL_INIT_CHANGE  10   /* Biais initial : encourage l'exploration CHANGE */
#define RL_QVAL_MAX        2000
#define RL_QVAL_MIN       (-2000)

/* Epsilon-greedy : 10% exploration => rand % 10 == 0 */
#define RL_EPSILON_DIV       10

/* ==============================================================================
   STRUCTURES ET VARIABLES STATIQUES DU GATEKEEPER Q-LEARNING  (~230 bytes RAM)
   ============================================================================== */

/*
 * Entree du cache LRU.
 * La "cle d'etat" est la paire (addr_curr, addr_cand) — 2 bytes chacune
 * (dernier octet de l'adresse MAC, suffisant pour distinguer les noeuds
 *  dans une simulation Contiki/Cooja).
 */
typedef struct {
  uint8_t  addr_curr;      /* Dernier octet lladdr du parent actuel   */
  uint8_t  addr_cand;      /* Dernier octet lladdr du champion        */
  int16_t  q_stay;         /* Q(s, RESTER)                            */
  int16_t  q_change;       /* Q(s, CHANGER)                           */
  uint16_t last_used;      /* Temps logique LRU (plus grand = recent) */
} rl_lru_entry_t;

/* Cache LRU statique : 20 * 10 bytes = 200 bytes */
static rl_lru_entry_t rl_cache[RL_CACHE_SIZE];
static uint8_t        rl_cache_used = 0;    /* Entrees occupees        */
static uint16_t       rl_lru_clock  = 0;   /* Horloge logique LRU     */

/* Memoire de l'episode precedent pour le reward periodique */
static uint8_t  rl_last_action      = RL_ACTION_STAY;
static uint8_t  rl_last_curr_addr   = 0;   /* Addr du parent au moment de la decision */
static uint8_t  rl_last_cand_addr   = 0;   /* Addr du champion au moment de la decision */
static uint16_t rl_last_etx_raw     = 0;   /* ETX brut du parent au moment de la decision */
static uint16_t rl_time_with_parent = 0;   /* Nb de cycles periodiques avec meme parent */
static uint16_t rl_last_curr_tau    = 0;   /* tau_cand de l'ancien parent (pour reward CHANGE) */
static uint16_t rl_last_cand_tau    = 0;   /* tau_cand du champion (pour reward CHANGE) */

/* ==============================================================================
   DECLARATIONS ANTICIPEES
   ============================================================================== */

static struct ctimer panic_monitor_timer;
static int panic_timer_started = 0;
static void handle_panic_monitor(void *ptr);

/* ==============================================================================
   FONCTIONS UTILITAIRES OF-TAU (INCHANGEES)
   ============================================================================== */

/*---------------------------------------------------------------------------*/
static uint16_t
clamp_tau(uint16_t tau)
{
  if(tau > 1000) return 1000;
  return tau;
}
/*---------------------------------------------------------------------------*/
static uint16_t
get_etx_or_default(rpl_parent_t *p)
{
  const struct link_stats *ls = rpl_get_parent_link_stats(p);
  if(ls != NULL && ls->etx > 0) {
    return ls->etx;
  }
  return RPL_OF_TAU_INIT_ETX;
}
/*---------------------------------------------------------------------------*/
static void
reset(rpl_dag_t *dag)
{
  (void)dag;
  printf("RPL: Reset OF-TAU+RL\n");
}
/*---------------------------------------------------------------------------*/
static uint16_t
parent_link_metric(rpl_parent_t *p)
{
  return get_etx_or_default(p);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Calcule le score tau_cand frais pour un parent candidat.
 * NE tient PAS compte de RE ni QL (etat local du noeud, pas du parent).
 */
static uint16_t
calculate_candidate_score(rpl_parent_t *p)
{
  if(p == NULL) return 0;

  uint16_t fresh_etx  = rpl_etx_norm(p);
  uint16_t fresh_rssi = rpl_rssi_norm(p);

  if(fresh_etx == 0 || fresh_rssi == 0) {
    return 0; /* Lien catastrophique */
  }

  return rpl_tau_compute_cand(fresh_etx, fresh_rssi, p->pe_Tau);
}

/* ==============================================================================
   GATEKEEPER Q-LEARNING — FONCTIONS INTERNES
   ============================================================================== */

/*---------------------------------------------------------------------------*/
/**
 * \brief Extrait un identifiant uint8_t compact d'un parent RPL.
 *
 * Utilise le dernier octet du pointeur parent (unique dans la RAM du Z1).
 * Evite d'inclure nbr-table.h ou de manipuler des lladdr 8-bytes.
 * Suffisant pour distinguer les noeuds dans Cooja (max 50 noeuds).
 */
static uint8_t
parent_to_addr(rpl_parent_t *p)
{
  /* L'adresse memoire d'un rpl_parent_t est unique et stable pendant
   * toute la duree de vie de l'entree dans la neighbor table. */
  return (uint8_t)((uintptr_t)p & 0xFF);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Cherche ou cree une entree dans le cache LRU.
 *
 * Si (addr_curr, addr_cand) existe => retourne son index.
 * Si non => cree une nouvelle entree (en ecrasant la plus ancienne si plein).
 * Met toujours a jour last_used pour implementer l'eviction LRU.
 *
 * \return index dans rl_cache[], toujours valide.
 */
static uint8_t
rl_lru_get_or_create(uint8_t addr_curr, uint8_t addr_cand)
{
  uint8_t  found_idx  = 0xFF;
  uint8_t  lru_idx    = 0;
  uint16_t lru_time   = 0xFFFF;
  uint8_t  i;

  for(i = 0; i < rl_cache_used; i++) {
    if(rl_cache[i].addr_curr == addr_curr &&
       rl_cache[i].addr_cand == addr_cand) {
      found_idx = i;
      break;
    }
    /* Chercher le candidat LRU (plus petit last_used) */
    if(rl_cache[i].last_used < lru_time) {
      lru_time = rl_cache[i].last_used;
      lru_idx  = i;
    }
  }

  if(found_idx != 0xFF) {
    /* Entree existante : mettre a jour le timestamp */
    rl_lru_clock++;
    rl_cache[found_idx].last_used = rl_lru_clock;
    return found_idx;
  }

  /* Entree inexistante : creer */
  uint8_t new_idx;
  if(rl_cache_used < RL_CACHE_SIZE) {
    /* Cache non plein : utiliser le prochain slot libre */
    new_idx = rl_cache_used;
    rl_cache_used++;
  } else {
    /* Cache plein : ecraser la ligne la plus ancienne (LRU) */
    new_idx = lru_idx;
    printf("[RL-LRU] Cache full, evicting entry (%u,%u)\n",
           rl_cache[lru_idx].addr_curr, rl_cache[lru_idx].addr_cand);
  }

  rl_lru_clock++;
  rl_cache[new_idx].addr_curr  = addr_curr;
  rl_cache[new_idx].addr_cand  = addr_cand;
  rl_cache[new_idx].q_stay     = RL_QVAL_INIT_STAY;
  rl_cache[new_idx].q_change   = RL_QVAL_INIT_CHANGE; /* Biais initial pro-CHANGE */
  rl_cache[new_idx].last_used  = rl_lru_clock;

  return new_idx;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Clamp une Q-value dans [RL_QVAL_MIN, RL_QVAL_MAX].
 */
static int16_t
rl_clamp_qval(int32_t v)
{
  if(v >  RL_QVAL_MAX) return  (int16_t)RL_QVAL_MAX;
  if(v <  RL_QVAL_MIN) return  (int16_t)RL_QVAL_MIN;
  return (int16_t)v;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Evalue la recompense de l'action precedente et met a jour la Q-table.
 *
 * Appele periodiquement (~5s) par le Panic Monitor.
 * Calcule le reward prédictif pour l'action choisie au dernier appel de
 * best_parent(), puis applique la mise a jour Bellman entiere.
 *
 * Reward : voir specification en tete de fichier.
 */
static void
rl_evaluate_past_action(rpl_dag_t *dag)
{
  if(dag == NULL || dag->preferred_parent == NULL) {
    /* Pas de parent => rupture totale. Punir severement l'action passee. */
    uint8_t idx = rl_lru_get_or_create(rl_last_curr_addr, rl_last_cand_addr);
    int32_t reward = -500;

    if(rl_last_action == RL_ACTION_STAY) {
      int32_t old_q = rl_cache[idx].q_stay;
      rl_cache[idx].q_stay = rl_clamp_qval(old_q + (reward - old_q) / RL_ALPHA_INV);
    } else {
      int32_t old_q = rl_cache[idx].q_change;
      rl_cache[idx].q_change = rl_clamp_qval(old_q + (reward - old_q) / RL_ALPHA_INV);
    }

    rl_time_with_parent = 0;
    printf("[RL-REWARD] RUPTURE (no parent) action=%s R=-500\n",
           rl_last_action == RL_ACTION_STAY ? "STAY" : "CHANGE");
    return;
  }

  uint16_t cur_etx = get_etx_or_default(dag->preferred_parent);
  uint8_t  idx     = rl_lru_get_or_create(rl_last_curr_addr, rl_last_cand_addr);
  int32_t  reward  = 0;

  if(rl_last_action == RL_ACTION_CHANGE) {
    /* ----------------------------------------------------------------
     * Reward pour action CHANGE :
     *   R = tau_cand(Nouveau_Parent_actuel) - tau_cand(Ancien_Parent)
     *
     * IMPORTANT : rl_last_curr_tau = tau de l'ANCIEN parent AVANT le switch
     *             (enregistre au moment de la decision dans best_parent).
     *             dag->preferred_parent EST DEJA le nouveau parent.
     * ---------------------------------------------------------------- */
    uint16_t new_tau = clamp_tau(calculate_candidate_score(dag->preferred_parent));
    /* rl_last_cand_tau = tau du champion au moment de la decision */
    /* On compare le tau actuel du nouveau parent vs le tau memorise de l'ancien */
    reward = (int32_t)new_tau - (int32_t)rl_last_curr_tau;
    rl_time_with_parent = 1; /* Compte un premier cycle avec le nouveau parent */

    printf("[RL-REWARD] CHANGE: tau_new=%u tau_old_pref=%u => R=%ld\n",
           new_tau, rl_last_curr_tau, (long)reward);

    int32_t old_q = rl_cache[idx].q_change;
    rl_cache[idx].q_change = rl_clamp_qval(old_q + (reward - old_q) / RL_ALPHA_INV);

  } else {
    /* ----------------------------------------------------------------
     * Reward pour action STAY :
     *   Base : R = time_with_parent * 5
     *   Penalite delta_ETX < -2.0 (chute rapide) : R -= 100
     *   Penalite zone danger ETX >= 5.0           : R -= 150
     *   Rupture ETX >= 8.0                        : R = -500
     * ---------------------------------------------------------------- */
    rl_time_with_parent++;

    if(cur_etx >= RL_ETX_RUPTURE) {
      /* Rupture imminente : overwrite total */
      reward = -500;
      rl_time_with_parent = 0;
      printf("[RL-REWARD] STAY: RUPTURE etx=%u => R=-500\n", cur_etx);
    } else {
      /* Base : stabilite recompensee */
      reward = (int32_t)rl_time_with_parent * RL_STAY_BONUS_SCALE;

      /* Penalite chute rapide delta_ETX */
      if(rl_last_etx_raw > 0 && rl_last_etx_raw != 0xFFFF) {
        /* delta_etx = ETX_t-1 - ETX_t : negatif si ETX monte (dégradation) */
        int32_t delta = (int32_t)rl_last_etx_raw - (int32_t)cur_etx;
        if(delta < -(int32_t)RL_ETX_FAST_DROP) {
          reward -= 100;
          printf("[RL-REWARD] STAY: fast ETX drop delta=%ld => -100\n", (long)delta);
        }
      }

      /* Penalite zone danger */
      if(cur_etx >= RL_ETX_DANGER) {
        reward -= 150;
        rl_time_with_parent = 0;
        printf("[RL-REWARD] STAY: danger zone etx=%u => -150\n", cur_etx);
      }

      printf("[RL-REWARD] STAY: time=%u base=%ld => R=%ld\n",
             rl_time_with_parent,
             (long)((int32_t)rl_time_with_parent * RL_STAY_BONUS_SCALE),
             (long)reward);
    }

    int32_t old_q = rl_cache[idx].q_stay;
    rl_cache[idx].q_stay = rl_clamp_qval(old_q + (reward - old_q) / RL_ALPHA_INV);
  }

  /* Mettre a jour l'ETX de reference pour le prochain cycle */
  rl_last_etx_raw = cur_etx;

  printf("[RL-TABLE]  LRU#%u (%u,%u): Q[STAY]=%d Q[CHANGE]=%d\n",
         idx, rl_cache[idx].addr_curr, rl_cache[idx].addr_cand,
         rl_cache[idx].q_stay, rl_cache[idx].q_change);
}

/* ==============================================================================
   PANIC MONITOR (Supervision reactive + declenchement reward periodique)
   ============================================================================== */

/*---------------------------------------------------------------------------*/
static void
handle_panic_monitor(void *ptr)
{
  rpl_instance_t *default_instance = rpl_get_default_instance();
  if(default_instance != NULL && default_instance->current_dag != NULL) {
    rpl_dag_t *dag = default_instance->current_dag;
    rpl_parent_t *p = dag->preferred_parent;

    /* Supervision reactive : parent en danger => forcer reevaluation */
    if(p != NULL) {
      uint16_t live_tau = calculate_candidate_score(p);
      if(live_tau < RPL_OF_TAU_PANIC_THRESHOLD) {
        printf("RPL: OF-TAU PANIC! Parent tau=%u < %u. Forcing switch!\n",
               live_tau, RPL_OF_TAU_PANIC_THRESHOLD);
        rpl_select_parent(dag);
      }
    }

    /* Apprentissage periodique : evaluer le reward de l'action passée */
    rl_evaluate_past_action(dag);
  }

  ctimer_set(&panic_monitor_timer, 5 * CLOCK_SECOND, handle_panic_monitor, NULL);
}

/* ==============================================================================
   FONCTIONS OF STANDARD (INCHANGEES)
   ============================================================================== */

/*---------------------------------------------------------------------------*/
/**
 * \brief Verifie si un lien parent est physiquement exploitable.
 */
static int
parent_has_usable_link(rpl_parent_t *p)
{
  if(p == NULL) return 0;

  /* Demarrage silencieux du Panic monitor (Lazy Init) */
  if(!panic_timer_started) {
    ctimer_set(&panic_monitor_timer, 5 * CLOCK_SECOND, handle_panic_monitor, NULL);
    panic_timer_started = 1;
  }

  uint16_t tau = clamp_tau(calculate_candidate_score(p));
  uint16_t etx = get_etx_or_default(p);

  if(tau < RPL_OF_TAU_MIN_TAU) return 0;
  if(etx == 0xFFFF || etx == 0) return 0;
  if(etx > RPL_OF_TAU_MAX_ETX) return 0;

  return 1;
}
/*---------------------------------------------------------------------------*/
static uint16_t
parent_path_cost(rpl_parent_t *p)
{
  if(p == NULL) return 0xFFFF;
  uint16_t tau = clamp_tau(calculate_candidate_score(p));
  return (uint16_t)(1000 - tau);
}
/*---------------------------------------------------------------------------*/
static rpl_rank_t
rank_via_parent(rpl_parent_t *p)
{
  if(p == NULL || p->dag == NULL || p->dag->instance == NULL) {
    return INFINITE_RANK;
  }

  rpl_instance_t *instance = p->dag->instance;
  uint16_t etx = get_etx_or_default(p);

  if(etx == 0xFFFF || etx == 0 || etx > RPL_OF_TAU_MAX_ETX) {
    return INFINITE_RANK;
  }

  uint16_t min_hoprankinc = instance->min_hoprankinc;
  uint16_t path_cost = p->rank + etx;

  return MAX(MIN((uint32_t)p->rank + min_hoprankinc, 0xFFFF), path_cost);
}

/* ==============================================================================
   BEST_PARENT
   ETAPE 1 : Filtre MRHOF + TAU (INTOUCHABLE) => elu le "Champion"
   ETAPE 2 : Gatekeeper Q-Learning => veto si Champion != Parent_Actuel
   ============================================================================== */

/*---------------------------------------------------------------------------*/
/**
 * \brief Selectionne le meilleur parent.
 *
 * ETAPE 1 — Algorithme classique MRHOF + TAU (absolument intact) :
 *   Filtre 1 : rejeter les liens inutilisables.
 *   Filtre 2 : garantir loop-free DODAG (rank constraint).
 *   Filtre 3 : path-cost ETX cumule (MRHOF) + score TAU + hysteresis.
 *   Tie-breaks : path-cost, rank, ETX direct.
 *   => Resultat : le "Champion" algorithmique.
 *
 * ETAPE 2 — Gatekeeper Q-Learning (surcouche finale) :
 *   S'active UNIQUEMENT si Champion != dag->preferred_parent.
 *   Interroge le cache LRU pour (current_parent, champion).
 *   Politique epsilon-greedy (epsilon=0.1).
 *   Action 0 (STAY)   => retourne dag->preferred_parent (veto).
 *   Action 1 (CHANGE) => retourne champion (autorisation).
 *   Enregistre l'action pour le prochain calcul de reward.
 */
static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  if(p1 == NULL) return p2;
  if(p2 == NULL) return p1;

  /* ================================================================
   * ETAPE 1 — FILTRE MRHOF + TAU (INTOUCHABLE)
   * ================================================================ */

  /* Filtre 1 : Liens inutilisables */
  if(!parent_has_usable_link(p1)) return p2;
  if(!parent_has_usable_link(p2)) return p1;

  /* Filtre 2 : Rank constraint (loop-free DODAG) */
  rpl_rank_t r1 = rank_via_parent(p1);
  rpl_rank_t r2 = rank_via_parent(p2);
  if(r1 == INFINITE_RANK && r2 == INFINITE_RANK) return p1;
  if(r1 == INFINITE_RANK) return p2;
  if(r2 == INFINITE_RANK) return p1;

  rpl_dag_t *dag = p1->dag;

  /* Path-cost ETX cumule (MRHOF, RFC6719) */
  uint16_t pc1 = (uint16_t)MIN((uint32_t)p1->rank + get_etx_or_default(p1), 0xffff);
  uint16_t pc2 = (uint16_t)MIN((uint32_t)p2->rank + get_etx_or_default(p2), 0xffff);

  /* Score TAU candidat */
  uint16_t t1 = clamp_tau(calculate_candidate_score(p1));
  uint16_t t2 = clamp_tau(calculate_candidate_score(p2));

  printf("RPL: OF-TAU comparing parents: tau1=%u (pc=%u) vs tau2=%u (pc=%u)\n",
         t1, pc1, t2, pc2);

  /* Champion algorithmique (resultat de l'ETAPE 1) */
  rpl_parent_t *champion;

  if(dag != NULL && (p1 == dag->preferred_parent || p2 == dag->preferred_parent)) {
    /*
     * CAS A : un des candidats est le preferred_parent courant.
     * Hysteresis combinee MRHOF + TAU.
     */
    rpl_parent_t *pref   = dag->preferred_parent;
    rpl_parent_t *cand   = (pref == p1) ? p2 : p1;
    uint16_t      t_pref = (pref == p1) ? t1 : t2;
    uint16_t      t_cand = (pref == p1) ? t2 : t1;
    uint16_t      pc_pref= (pref == p1) ? pc1 : pc2;
    uint16_t      pc_cand= (pref == p1) ? pc2 : pc1;

    int pc_close = (pc_pref <= pc_cand + MRHOF_PC_THRESHOLD) &&
                   (pc_cand <= pc_pref + MRHOF_PC_THRESHOLD);

    if(pc_close) {
      /* Chemins ETX equivalents => TAU avec hysteresis */
      if(t_cand <= (uint16_t)(t_pref + RPL_OF_TAU_SWITCH_THRESHOLD)) {
        champion = pref;
      } else {
        champion = cand;
      }
    } else if(pc_pref <= pc_cand) {
      /* preferred a le meilleur path-cost ETX */
      champion = (t_cand <= (uint16_t)(t_pref + RPL_OF_TAU_SWITCH_THRESHOLD))
                 ? pref : cand;
    } else {
      /* candidat a le meilleur path-cost ETX */
      champion = (t_pref <= (uint16_t)(t_cand + RPL_OF_TAU_SWITCH_THRESHOLD))
                 ? cand : pref;
    }

    /* ================================================================
     * ETAPE 2 — GATEKEEPER Q-LEARNING
     * S'active UNIQUEMENT si le Champion est different du parent actuel.
     * ================================================================ */
    if(champion != pref) {
      /*
       * L'algorithme classique veut changer de parent.
       * Le Gatekeeper RL decide si on autorise ou bloque ce handoff.
       */
      uint8_t  addr_curr  = parent_to_addr(pref);
      uint8_t  addr_cand  = parent_to_addr(cand);
      uint8_t  idx        = rl_lru_get_or_create(addr_curr, addr_cand);

      /* CAPTURE des taus AVANT toute decision (necessaire pour le reward CHANGE) */
      uint16_t tau_pref_now = clamp_tau(calculate_candidate_score(pref));
      uint16_t tau_cand_now = clamp_tau(calculate_candidate_score(cand));

      int16_t q_stay   = rl_cache[idx].q_stay;
      int16_t q_change = rl_cache[idx].q_change;

      /* Politique epsilon-greedy : 10% d'exploration aleatoire */
      uint8_t action;
      if((random_rand() % RL_EPSILON_DIV) == 0) {
        /* EXPLORATION : action aleatoire */
        action = (uint8_t)(random_rand() % 2);
        printf("[RL-GATE] EXPLORE action=%s Q[STAY]=%d Q[CHANGE]=%d\n",
               action == RL_ACTION_STAY ? "STAY  " : "CHANGE",
               q_stay, q_change);
      } else {
        /* EXPLOITATION : action greedy (q_change > q_stay => CHANGE) */
        action = (q_change > q_stay) ? RL_ACTION_CHANGE : RL_ACTION_STAY;
        printf("[RL-GATE] EXPLOIT action=%s Q[STAY]=%d Q[CHANGE]=%d\n",
               action == RL_ACTION_STAY ? "STAY  " : "CHANGE",
               q_stay, q_change);
      }

      /* Enregistrer la decision et les VALEURS DE L'ANCIEN PARENT
       * rl_last_curr_tau = tau du parent actuel (AVANT le switch).
       * rl_last_etx_raw  = ETX du parent actuel (AVANT le switch).
       * Ces valeurs servent de reference pour le calcul du reward au prochain cycle. */
      rl_last_action    = action;
      rl_last_curr_addr = addr_curr;
      rl_last_cand_addr = addr_cand;
      rl_last_etx_raw   = get_etx_or_default(pref);  /* ETX de l'ANCIEN parent */
      rl_last_curr_tau  = tau_pref_now;               /* tau de l'ANCIEN parent */
      rl_last_cand_tau  = tau_cand_now;

      if(action == RL_ACTION_STAY) {
        /* VETO : on bloque le switch */
        printf("[RL-GATE] VETO => pref tau=%u, cand tau=%u blocked\n",
               tau_pref_now, tau_cand_now);
        return pref;
      } else {
        /* AUTORISATION : handoff vers le champion */
        printf("[RL-GATE] ALLOW => switching cand tau=%u (was pref tau=%u)\n",
               tau_cand_now, tau_pref_now);
        /* NE PAS reset rl_time_with_parent ici : fait dans rl_evaluate_past_action */
        return cand;
      }
    }

    /* Champion == pref : l'algorithme classique confirme le parent actuel.
     * On met a jour la memoire pour le reward du prochain cycle STAY. */
    rl_last_action    = RL_ACTION_STAY;
    rl_last_curr_addr = parent_to_addr(pref);
    rl_last_cand_addr = parent_to_addr(cand);
    rl_last_etx_raw   = get_etx_or_default(pref);  /* ETX reference pour delta_ETX */
    rl_last_curr_tau  = clamp_tau(calculate_candidate_score(pref));
    return pref;
  }

  /* ================================================================
   * CAS B : Pas de preferred_parent => selection directe MRHOF + TAU.
   * Le Gatekeeper RL ne s'applique pas (pas de contexte de stabilite).
   * ================================================================ */
  {
    int pc_close = (pc1 <= pc2 + MRHOF_PC_THRESHOLD) &&
                   (pc2 <= pc1 + MRHOF_PC_THRESHOLD);
    if(pc_close) {
      if(t2 > t1) return p2;
      if(t1 > t2) return p1;
    } else if(pc1 < pc2) {
      return (t2 > (uint16_t)(t1 + RPL_OF_TAU_SWITCH_THRESHOLD)) ? p2 : p1;
    } else {
      return (t1 > (uint16_t)(t2 + RPL_OF_TAU_SWITCH_THRESHOLD)) ? p1 : p2;
    }
  }

  /* Tie-break 1: path-cost ETX (MRHOF) */
  if(pc2 < pc1) return p2;
  if(pc1 < pc2) return p1;

  /* Tie-break 2: rank */
  if(r2 < r1) return p2;
  if(r1 < r2) return p1;

  /* Tie-break 3: ETX direct */
  return (parent_link_metric(p2) < parent_link_metric(p1)) ? p2 : p1;
}

/* ==============================================================================
   BEST_DAG & UPDATE_METRIC_CONTAINER (INCHANGES)
   ============================================================================== */

/*---------------------------------------------------------------------------*/
static rpl_dag_t *
best_dag(rpl_dag_t *d1, rpl_dag_t *d2)
{
  if(d1 == NULL) return d2;
  if(d2 == NULL) return d1;

  if(d1->grounded != d2->grounded) {
    return d1->grounded ? d1 : d2;
  }
  if(d1->preference != d2->preference) {
    return d1->preference > d2->preference ? d1 : d2;
  }
  return d1->rank <= d2->rank ? d1 : d2;
}
/*---------------------------------------------------------------------------*/
static void
update_metric_container(rpl_instance_t *instance)
{
  /* OF-TAU propagates state via custom PE option, no standard MC needed */
  instance->mc.type = RPL_DAG_MC_NONE;
}
/*---------------------------------------------------------------------------*/
/* Export OF */
rpl_of_t rpl_of_tau = {
  .reset = reset,
#if RPL_WITH_DAO_ACK
  .dao_ack_callback = NULL,
#endif
  .parent_link_metric = parent_link_metric,
  .parent_has_usable_link = parent_has_usable_link,
  .parent_path_cost = parent_path_cost,
  .rank_via_parent = rank_via_parent,
  .best_parent = best_parent,
  .best_dag = best_dag,
  .update_metric_container = update_metric_container,
  .ocp = RPL_OCP_TAU
};
