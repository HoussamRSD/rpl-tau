/*---------------------------------------------------------------------------*/
/* rpl-of-tau.c
 *
 * Objective Function TAU for RPL.
 *
 * tau_cand = F(ETX_norm, RSSI_norm, tau_parent)  -- choix du parent (inchange)
 * tau_local = F(tau_cand_path, Deg, NPC, RE, QL) -- etat local diffuse dans DIO
 *
 * best_parent() = logique combinee MRHOF + TAU :
 *   1. Rejeter liens inutilisables (ETX > MAX, TAU < MIN).
 *   2. Garantir loop-free DODAG (rank constraint).
 *   3. Si |path-cost ETX| proches (< MRHOF_PC_THRESHOLD = 96) :
 *        => chemins equivalents => TAU tranche (avec hysteresis TAU).
 *   4. Si ecart ETX significatif :
 *        => MRHOF impose le meilleur chemin,
 *           SAUF si le perdant a un TAU nettement superieur.
 *
 * rank_via_parent() = ETX-based (guarantees DODAG loop-free property)
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/link-stats.h"
#include <limits.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/* Fallback if not defined by platform */
#ifndef LINK_STATS_ETX_DIVISOR
#define LINK_STATS_ETX_DIVISOR 128
#endif

/* ==============================================================================
   PARAMETRES DE CONTROLE DE MOBILITE & STABILITE RESEAU
   ============================================================================== */

/*
 * ETX Tolerance (Proactive Rejection).
 */
#ifndef RPL_OF_TAU_MAX_ETX
#define RPL_OF_TAU_MAX_ETX (8 * LINK_STATS_ETX_DIVISOR)
#endif

#ifndef RPL_OF_TAU_INIT_ETX
#define RPL_OF_TAU_INIT_ETX (2 * LINK_STATS_ETX_DIVISOR)
#endif

/*
 * Switch Hysteresis Threshold (Ping-Pong Prevention).
 * Un noeud ne switche que si tau_cand depasse le parent actuel de cette valeur.
 */
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
#define MRHOF_PC_THRESHOLD  96
#endif

static struct ctimer panic_monitor_timer;
static int panic_timer_started = 0;

static void handle_panic_monitor(void *ptr);

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
  printf("RPL: Reset OF-TAU\n");
}
/*---------------------------------------------------------------------------*/
static uint16_t
parent_link_metric(rpl_parent_t *p)
{
  /* ETX kept for debug / rank_via_parent.
   * Parent decision occurs via tau_cand in best_parent(). */
  return get_etx_or_default(p);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Calcule le score tau_cand frais pour un parent candidat.
 *
 * Utilise ETX et RSSI mesures localement + le tau_parent recu dans le DIO.
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
/*---------------------------------------------------------------------------*/
/**
 * \brief Callback periodique pour evaluer la sante en temps reel (Panic Monitor)
 *
 * Per RL.md Section 7: only triggers RL re-selection when the link is actually
 * degraded (RSSI too low OR ETX too high). Avoids running RL on every tick.
 */
static void
handle_panic_monitor(void *ptr)
{
  rpl_instance_t *default_instance = rpl_get_default_instance();
  if(default_instance != NULL && default_instance->current_dag != NULL) {
    rpl_dag_t *dag = default_instance->current_dag;
    rpl_parent_t *p = dag->preferred_parent;

    if(p != NULL) {
      const struct link_stats *ls = rpl_get_parent_link_stats(p);
      uint16_t live_etx  = get_etx_or_default(p);
      int16_t  live_rssi = (ls != NULL) ? ls->rssi : (int16_t)(-100);

      /* Link is healthy: RSSI good AND ETX acceptable — do nothing */
      if(live_rssi > (int16_t)RL_RSSI_THRESHOLD &&
         live_etx  < (uint16_t)RL_ETX_THRESHOLD) {
        /* No action needed */
      } else {
        /* Link degraded → let the RL agent decide the new parent */
        printf("RPL: OF-TAU PANIC! rssi=%d etx=%u. Triggering RL.\n",
               (int)live_rssi, live_etx);
        rpl_rl_trigger(dag);
      }
    }
  }

  ctimer_set(&panic_monitor_timer, 5 * CLOCK_SECOND, handle_panic_monitor, NULL);
}
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
/*---------------------------------------------------------------------------*/
static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  if(p1 == NULL) return p2;
  if(p2 == NULL) return p1;

  /* --- Filtre 1 : Liens inutilisables --- */
  if(!parent_has_usable_link(p1)) return p2;
  if(!parent_has_usable_link(p2)) return p1;

  /* --- Filtre 2 : Rank constraint (loop-free DODAG) --- */
  rpl_rank_t r1 = rank_via_parent(p1);
  rpl_rank_t r2 = rank_via_parent(p2);
  if(r1 == INFINITE_RANK && r2 == INFINITE_RANK) return p1;
  if(r1 == INFINITE_RANK) return p2;
  if(r2 == INFINITE_RANK) return p1;

  rpl_dag_t *dag = p1->dag;

  /* --- Path-cost ETX  ---
   * pc = rank_parent + ETX_direct_link
   * Permet de comparer le cout global du chemin, pas juste le saut direct. */
  uint16_t pc1 = (uint16_t)MIN((uint32_t)p1->rank + get_etx_or_default(p1), 0xffff);
  uint16_t pc2 = (uint16_t)MIN((uint32_t)p2->rank + get_etx_or_default(p2), 0xffff);

  /* --- Score TAU candidat (qualite globale du chemin OF-TAU) --- */
  uint16_t t1 = clamp_tau(calculate_candidate_score(p1));
  uint16_t t2 = clamp_tau(calculate_candidate_score(p2));

  printf("RPL: OF-TAU comparing parents: tau1=%u (pc=%u) vs tau2=%u (pc=%u)\n",
         t1, pc1, t2, pc2);

  if(dag != NULL && (p1 == dag->preferred_parent || p2 == dag->preferred_parent)) {
    /*
     * CAS : un des candidats est le preferred_parent courant.
     * Appliquer l'hysteresis 
     */
    rpl_parent_t *pref = dag->preferred_parent;
    rpl_parent_t *cand = (pref == p1) ? p2 : p1;
    uint16_t t_pref    = (pref == p1) ? t1 : t2;
    uint16_t t_cand    = (pref == p1) ? t2 : t1;
    uint16_t pc_pref   = (pref == p1) ? pc1 : pc2;
    uint16_t pc_cand   = (pref == p1) ? pc2 : pc1;

    /* Les deux chemins ETX sont-ils proches ? */
    int pc_close = (pc_pref <= pc_cand + MRHOF_PC_THRESHOLD) &&
                   (pc_cand <= pc_pref + MRHOF_PC_THRESHOLD);

    if(pc_close) {
      /* Chemins ETX equivalents => TAU avec hysteresis TAU */
      if(t_cand <= (uint16_t)(t_pref + RPL_OF_TAU_SWITCH_THRESHOLD)) {
        return pref; /* Pas assez meilleur en TAU => garder preferred */
      }
      return cand;   /* Nettement meilleur en TAU => switcher */
    } else {
      /* Ecart ETX significatif */
      if(pc_pref <= pc_cand) {
        /* preferred a le meilleur path-cost ETX */
        if(t_cand <= (uint16_t)(t_pref + RPL_OF_TAU_SWITCH_THRESHOLD)) {
          return pref; /* ETX pref meilleur ET TAU cand pas assez superieur */
        }
        return cand;   /* TAU du candidat compense vraiment */
      } else {
        /* candidat a le meilleur path-cost ETX */
        if(t_pref <= (uint16_t)(t_cand + RPL_OF_TAU_SWITCH_THRESHOLD)) {
          return cand; /* Meilleur ETX pour le candidat, TAU pref pas nettement sup. */
        }
        return pref;   /* preferred a un TAU nettement superieur => garder */
      }
    }
  }

  /* --- CAS : pas de preferred_parent => selection directe --- */
  {
    int pc_close = (pc1 <= pc2 + MRHOF_PC_THRESHOLD) &&
                   (pc2 <= pc1 + MRHOF_PC_THRESHOLD);
    if(pc_close) {
      /* ETX equivalents => TAU decide */
      if(t2 > t1) return p2;
      if(t1 > t2) return p1;
    } else if(pc1 < pc2) {
      /* p1 meilleur ETX ; p2 compense seulement si TAU nettement superieur */
      return (t2 > (uint16_t)(t1 + RPL_OF_TAU_SWITCH_THRESHOLD)) ? p2 : p1;
    } else {
      return (t1 > (uint16_t)(t2 + RPL_OF_TAU_SWITCH_THRESHOLD)) ? p1 : p2;
    }
  }

  /* Tie-break 1: prefer lower path cost  */
  if(pc2 < pc1) return p2;
  if(pc1 < pc2) return p1;

  /* Tie-break 2: prefer lower rank (fewer hops to root) */
  if(r2 < r1) return p2;
  if(r1 < r2) return p1;

  /* Tie-break 3: prefer better direct link ETX */
  return (parent_link_metric(p2) < parent_link_metric(p1)) ? p2 : p1;
}
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
