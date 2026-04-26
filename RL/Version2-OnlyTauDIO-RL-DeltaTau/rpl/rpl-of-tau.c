/*---------------------------------------------------------------------------*/
/* rpl-of-tau.c
 *
 * Objective Function TAU for RPL — with Q-Learning Proactive Supervisor.
 *
 * ARCHITECTURE EN DEUX COUCHES :
 *
 * Couche 1 — Logique de base MRHOF + TAU (INTOUCHABLE, PDR=98%) :
 *   tau_cand = F(ETX_norm, RSSI_norm, tau_parent)
 *   best_parent() = argmax via path-cost ETX (MRHOF) + tau_cand (TAU)
 *                   avec hysteresis RPL_OF_TAU_SWITCH_THRESHOLD
 *
 * Couche 2 — Superviseur Q-Learning (SURCOUCHE PROACTIVE) :
 *   Greffe non-destructive. Ne remplace PAS la logique de base.
 *   Peut uniquement BYPASSER L'HYSTERESIS si :
 *     (a) Q-table suffisamment apprise (rl_total_updates >= RL_MIN_UPDATES),
 *     (b) Q(s, action) depasse l'autre de RL_OVERRIDE_THRESHOLD.
 *   Si QL est incertain => retombe sur Couche 1.
 *
 * LOGS ACTIFS (essentiels pour analyze.py + comprehension) :
 *   [RL-STATE]   => distribution etats + delta_tau (1 par appel best_parent avec pref)
 *   [RL-UPDATE] REWARD TOTAL => recompense hybride par cycle (~5s)
 *   [RL-UPDATE] REWARD: RUPTURE => rupture de lien detectee
 *   [RL-TABLE]   => snapshot Q-table apres chaque mise a jour
 *   [LAYER1] Classic decision => STAY/SWITCH decision de base
 *   [LAYER2] BOOTSTRAP / PROACTIVE SWITCH / PROTECTIVE STAY / QL UNCERTAIN
 *   [OF-TAU] FINAL => decision finale si QL a override
 *   OF-TAU PANIC! => declenchement du panic monitor
 *
 * LOGS COMMENTES (trop verbeux, ralentissent la simulation) :
 *   - Details internes de calculate_candidate_score (appel tres frequent)
 *   - REJECT dans parent_has_usable_link (appel tres frequent)
 *   - Details ETX/TAU dans LAYER1 (garder seulement la decision finale)
 *   - Details Q-values dans LAYER2 (garder seulement les overrides)
 *   - Details Bellman step-by-step dans rl_qlearning_update
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/link-stats.h"
#include <limits.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

#ifndef LINK_STATS_ETX_DIVISOR
#define LINK_STATS_ETX_DIVISOR 128
#endif

/* ==============================================================================
   PARAMETRES DE BASE OF-TAU
   ============================================================================== */

#ifndef RPL_OF_TAU_MAX_ETX
#define RPL_OF_TAU_MAX_ETX (8 * LINK_STATS_ETX_DIVISOR)
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
#define MRHOF_PC_THRESHOLD  96
#endif

/* ==============================================================================
   PARAMETRES DU SUPERVISEUR Q-LEARNING
   ============================================================================== */

#define RL_STATE_URGENCY  0
#define RL_STATE_MINOR    1
#define RL_STATE_MAJOR    2

#define RL_ACTION_STAY    0
#define RL_ACTION_SWITCH  1

#define RL_ALPHA_SHIFT          3   /* alpha = 1/8 */
#define RL_GAMMA_NUM            7
#define RL_GAMMA_DEN            8   /* gamma = 7/8 */

#define RL_REWARD_STABILITY_STEP  5
#define RL_REWARD_STABILITY_MAX  50
#define RL_REWARD_RUPTURE       (-50)
#define RL_ETX_REWARD_SCALE      16

#define RL_MIN_UPDATES_TO_TRUST   8
#define RL_OVERRIDE_THRESHOLD    20

#define RL_QVAL_MAX   500
#define RL_QVAL_MIN  (-500)

/* Q-Table [state][action] — ~22 bytes total RAM */
static int16_t  q_table[3][2];
static uint8_t  rl_last_state        = RL_STATE_URGENCY;
static uint8_t  rl_last_action       = RL_ACTION_STAY;
static uint16_t rl_last_etx_raw      = 0;
static int16_t  rl_stability_streak  = 0;
static uint16_t rl_total_updates     = 0;

static struct ctimer panic_monitor_timer;
static int panic_timer_started = 0;

static void handle_panic_monitor(void *ptr);

/* ==============================================================================
   FONCTIONS UTILITAIRES DE BASE
   ============================================================================== */

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
  return get_etx_or_default(p);
}
/*---------------------------------------------------------------------------*/
/**
 * Calcule le score tau_cand = F(ETX_norm, RSSI_norm, tau_parent_recu_DIO).
 * Appele tres frequemment => tous les printf internes sont commentes.
 */
static uint16_t
calculate_candidate_score(rpl_parent_t *p)
{
  if(p == NULL) return 0;

  uint16_t fresh_etx  = rpl_etx_norm(p);
  uint16_t fresh_rssi = rpl_rssi_norm(p);

  if(fresh_etx == 0 || fresh_rssi == 0) {
    /* COMMENTE : trop frequent, ralentit la simulation
    printf("[OF-TAU] BAD LINK: ETX_n=%u RSSI_n=%u => tau_cand=0\n",
           fresh_etx, fresh_rssi);
    */
    return 0;
  }

  uint16_t score = rpl_tau_compute_cand(fresh_etx, fresh_rssi, p->pe_Tau);

  /* COMMENTE : appele a chaque comparaison de parent, tres verbeux
  printf("[OF-TAU] Score: ETX_n=%u RSSI_n=%u tau_parent=%u => tau_cand=%u\n",
         fresh_etx, fresh_rssi, p->pe_Tau, score);
  */
  return score;
}

/* ==============================================================================
   SUPERVISEUR Q-LEARNING — FONCTIONS INTERNES
   ============================================================================== */

/*---------------------------------------------------------------------------*/
/**
 * \brief Determine l'etat abstrait RL.
 *
 * Garde le printf [RL-STATE] : 1 seul par appel best_parent() avec pref,
 * necessaire pour analyze.py (distribution des etats).
 */
static uint8_t
compute_rl_state(uint16_t tau_pref, uint16_t tau_cand)
{
  int32_t delta = (int32_t)tau_cand - (int32_t)tau_pref;
  uint8_t state;
  const char *name;

  if(tau_pref < 200) {
    state = RL_STATE_URGENCY;
    name  = "URGENCY";
  } else if(delta > 80) {
    state = RL_STATE_MAJOR;
    name  = "MAJOR  ";
  } else {
    state = RL_STATE_MINOR;
    name  = "MINOR  ";
  }

  /* GARDE : necessaire pour analyze.py (distribution etats + delta_tau) */
  printf("[RL-STATE] %s (%u): tau_pref=%u tau_cand=%u delta=%ld\n",
         name, state, tau_pref, tau_cand, (long)delta);
  return state;
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Mise a jour Q-Learning periodique (~5s).
 *
 * Logs gardes :
 *   [RL-UPDATE] REWARD: RUPTURE  => evenement important (rupture lien)
 *   [RL-UPDATE] REWARD TOTAL:    => necessaire pour analyze.py
 *   [RL-TABLE]                   => necessaire pour analyze.py (convergence)
 *
 * Logs commentes : details Bellman step-by-step (informatif mais verbeux)
 */
static void
rl_qlearning_update(rpl_dag_t *dag)
{
  /* COMMENTE : en-tete verbeux, pas utile pour analyze.py
  printf("[RL-UPDATE] === Bellman Update #%u ===\n", rl_total_updates + 1);
  printf("[RL-UPDATE] Prev decision: state=%u  action=%s  etx_raw=%u  streak=%d\n",
         rl_last_state,
         rl_last_action == RL_ACTION_STAY ? "STAY  " : "SWITCH",
         rl_last_etx_raw, (int)rl_stability_streak);
  */

  if(dag == NULL || dag->preferred_parent == NULL) {
    /* COMMENTE : informatif mais pas essentiel
    printf("[RL-UPDATE] WARNING: No preferred parent => implicit rupture!\n");
    */
    rl_stability_streak = 0;
    if(rl_total_updates < 0xFFFF) rl_total_updates++;
    int16_t td = RL_REWARD_RUPTURE - q_table[rl_last_state][rl_last_action];
    q_table[rl_last_state][rl_last_action] += (int16_t)(td >> RL_ALPHA_SHIFT);
    if(q_table[rl_last_state][rl_last_action] > RL_QVAL_MAX)
      q_table[rl_last_state][rl_last_action] = RL_QVAL_MAX;
    if(q_table[rl_last_state][rl_last_action] < RL_QVAL_MIN)
      q_table[rl_last_state][rl_last_action] = RL_QVAL_MIN;
    /* GARDE : rupture => evenement important */
    printf("[RL-UPDATE] REWARD: RUPTURE (%d) no preferred parent\n",
           RL_REWARD_RUPTURE);
    printf("[RL-TABLE]  #%u | URG[stay=%d sw=%d] MIN[stay=%d sw=%d] MAJ[stay=%d sw=%d]\n",
           rl_total_updates,
           q_table[0][0], q_table[0][1],
           q_table[1][0], q_table[1][1],
           q_table[2][0], q_table[2][1]);
    return;
  }

  uint16_t cur_etx = get_etx_or_default(dag->preferred_parent);
  uint16_t cur_tau = clamp_tau(calculate_candidate_score(dag->preferred_parent));

  /* COMMENTE : etat courant affiche dans [RL-TABLE], redondant ici
  printf("[RL-UPDATE] Current parent state: tau=%u  etx_raw=%u\n", cur_tau, cur_etx);
  */

  /* --- Compute hybrid reward --- */
  int16_t reward      = 0;
  int16_t r_etx       = 0;
  int16_t r_stability = 0;

  if(cur_etx > RPL_OF_TAU_MAX_ETX || cur_etx == 0xFFFF || cur_etx == 0) {
    reward = RL_REWARD_RUPTURE;
    rl_stability_streak = 0;
    /* GARDE : evenement de rupture important */
    printf("[RL-UPDATE] REWARD: RUPTURE (%d) etx=%u\n", RL_REWARD_RUPTURE, cur_etx);
  } else {
    if(rl_last_etx_raw > 0 && rl_last_etx_raw != 0xFFFF) {
      int32_t d = (int32_t)rl_last_etx_raw - (int32_t)cur_etx;
      r_etx = (int16_t)(d / RL_ETX_REWARD_SCALE);
      reward += r_etx;
      /* COMMENTE : detail par cycle, trop verbeux
      printf("[RL-UPDATE] REWARD: delta_ETX=%+ld raw => r_etx=%+d\n", (long)d, (int)r_etx);
      */
    }

    if(rl_last_action == RL_ACTION_STAY) {
      rl_stability_streak += RL_REWARD_STABILITY_STEP;
      if(rl_stability_streak > RL_REWARD_STABILITY_MAX)
        rl_stability_streak = RL_REWARD_STABILITY_MAX;
      r_stability = rl_stability_streak;
      reward += r_stability;
      /* COMMENTE : detail par cycle
      printf("[RL-UPDATE] REWARD: stability_bonus=%+d  (streak=%d)\n",
             (int)r_stability, (int)rl_stability_streak);
      */
    } else {
      rl_stability_streak = 0;
      /* COMMENTE : detail par cycle
      printf("[RL-UPDATE] REWARD: stability_bonus= 0  (streak reset after SWITCH)\n");
      */
    }
  }

  if(reward >  100) reward =  100;
  if(reward < -100) reward = -100;

  /* GARDE : recompense totale — necessaire pour analyze.py */
  printf("[RL-UPDATE] REWARD TOTAL: %+d  (etx=%+d  stability=%+d)\n",
         (int)reward, (int)r_etx, (int)r_stability);

  /* Next state estimate */
  uint8_t new_state = (cur_tau < 200) ? RL_STATE_URGENCY : RL_STATE_MINOR;
  /* COMMENTE : detail redondant avec [RL-TABLE]
  printf("[RL-UPDATE] Next state: %s (%u)  tau=%u\n",
         new_state == RL_STATE_URGENCY ? "URGENCY" : "MINOR", new_state, cur_tau);
  */

  /* Bellman update */
  int16_t max_next = (q_table[new_state][0] > q_table[new_state][1])
                     ? q_table[new_state][0] : q_table[new_state][1];
  int32_t target   = (int32_t)reward
                     + (int32_t)((RL_GAMMA_NUM * (int32_t)max_next) / RL_GAMMA_DEN);
  int32_t td_error = target - (int32_t)q_table[rl_last_state][rl_last_action];

  q_table[rl_last_state][rl_last_action] += (int16_t)(td_error >> RL_ALPHA_SHIFT);

  if(q_table[rl_last_state][rl_last_action] > RL_QVAL_MAX)
    q_table[rl_last_state][rl_last_action] = RL_QVAL_MAX;
  if(q_table[rl_last_state][rl_last_action] < RL_QVAL_MIN)
    q_table[rl_last_state][rl_last_action] = RL_QVAL_MIN;

  /* COMMENTE : detail Bellman step-by-step, utile pour debug profond
  printf("[RL-UPDATE] Bellman: target=%ld  td_err=%ld  alpha=1/8\n",
         (long)target, (long)td_error);
  printf("[RL-UPDATE] Q[%u][%s]: %d => %d\n",
         rl_last_state,
         rl_last_action == RL_ACTION_STAY ? "STAY  " : "SWITCH",
         (int)q_before,
         (int)q_table[rl_last_state][rl_last_action]);
  */

  /* Update memory */
  rl_last_state   = new_state;
  rl_last_etx_raw = cur_etx;
  if(rl_total_updates < 0xFFFF) rl_total_updates++;

  /* GARDE : snapshot Q-table — necessaire pour analyze.py */
  printf("[RL-TABLE]  #%u | URG[stay=%d sw=%d] MIN[stay=%d sw=%d] MAJ[stay=%d sw=%d]\n",
         rl_total_updates,
         q_table[0][0], q_table[0][1],
         q_table[1][0], q_table[1][1],
         q_table[2][0], q_table[2][1]);
}

/* ==============================================================================
   PANIC MONITOR
   ============================================================================== */

static void
handle_panic_monitor(void *ptr)
{
  rpl_instance_t *default_instance = rpl_get_default_instance();
  if(default_instance != NULL && default_instance->current_dag != NULL) {
    rpl_dag_t *dag = default_instance->current_dag;
    rpl_parent_t *p = dag->preferred_parent;

    if(p != NULL) {
      uint16_t live_tau = calculate_candidate_score(p);
      if(live_tau < RPL_OF_TAU_PANIC_THRESHOLD) {
        /* GARDE : evenement critique */
        printf("RPL: OF-TAU PANIC! Parent tau=%u < %u. Forcing switch!\n",
               live_tau, RPL_OF_TAU_PANIC_THRESHOLD);
        rpl_select_parent(dag);
      }
    }

    rl_qlearning_update(dag);
  }

  ctimer_set(&panic_monitor_timer, 5 * CLOCK_SECOND, handle_panic_monitor, NULL);
}

/* ==============================================================================
   FONCTIONS OF STANDARD
   ============================================================================== */

static int
parent_has_usable_link(rpl_parent_t *p)
{
  if(p == NULL) return 0;

  if(!panic_timer_started) {
    ctimer_set(&panic_monitor_timer, 5 * CLOCK_SECOND, handle_panic_monitor, NULL);
    panic_timer_started = 1;
  }

  uint16_t tau = clamp_tau(calculate_candidate_score(p));
  uint16_t etx = get_etx_or_default(p);

  /* COMMENTE : appele tres frequemment, ralentit la simulation
  if(tau < RPL_OF_TAU_MIN_TAU) {
    printf("[OF-TAU] REJECT parent: tau=%u < MIN_TAU=%u\n", tau, RPL_OF_TAU_MIN_TAU);
  }
  if(etx == 0xFFFF || etx == 0) {
    printf("[OF-TAU] REJECT parent: etx=0x%04X (invalid)\n", etx);
  }
  if(etx > RPL_OF_TAU_MAX_ETX) {
    printf("[OF-TAU] REJECT parent: etx=%u > MAX_ETX=%u\n", etx, RPL_OF_TAU_MAX_ETX);
  }
  */

  if(tau < RPL_OF_TAU_MIN_TAU)    return 0;
  if(etx == 0xFFFF || etx == 0)   return 0;
  if(etx > RPL_OF_TAU_MAX_ETX)    return 0;
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
   BEST_PARENT — COUCHE 1 (MRHOF+TAU) + COUCHE 2 (SUPERVISEUR QL)
   ============================================================================== */

/*---------------------------------------------------------------------------*/
/**
 * \brief Selectionne le meilleur parent — logique a deux couches.
 *
 * Logs actifs dans cette fonction :
 *   [RL-STATE]              => 1 par appel avec preferred_parent (pour analyze.py)
 *   [LAYER1] Classic decision => STAY/SWITCH  (pour analyze.py)
 *   [LAYER2] BOOTSTRAP MODE  => pour analyze.py
 *   [LAYER2] *** PROACTIVE SWITCH *** => evenement important
 *   [LAYER2] *** PROTECTIVE STAY ***  => evenement important
 *   [LAYER2] QL UNCERTAIN    => pour analyze.py
 *   [OF-TAU] FINAL =>        => uniquement si QL a override la couche 1
 */
static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  if(p1 == NULL) return p2;
  if(p2 == NULL) return p1;

  /* Filtre 1 : liens inutilisables */
  if(!parent_has_usable_link(p1)) {
    /* COMMENTE : trop frequent
    printf("[OF-TAU] p1 has unusable link => returning p2\n");
    */
    return p2;
  }
  if(!parent_has_usable_link(p2)) {
    /* COMMENTE : trop frequent
    printf("[OF-TAU] p2 has unusable link => returning p1\n");
    */
    return p1;
  }

  /* Filtre 2 : rank constraint */
  rpl_rank_t r1 = rank_via_parent(p1);
  rpl_rank_t r2 = rank_via_parent(p2);
  if(r1 == INFINITE_RANK && r2 == INFINITE_RANK) {
    /* COMMENTE : rare mais pas utile dans le flux normal
    printf("[OF-TAU] Both ranks INFINITE => returning p1 by default\n");
    */
    return p1;
  }
  if(r1 == INFINITE_RANK) return p2;
  if(r2 == INFINITE_RANK) return p1;

  rpl_dag_t *dag = p1->dag;

  uint16_t pc1 = (uint16_t)MIN((uint32_t)p1->rank + get_etx_or_default(p1), 0xffff);
  uint16_t pc2 = (uint16_t)MIN((uint32_t)p2->rank + get_etx_or_default(p2), 0xffff);

  uint16_t t1 = clamp_tau(calculate_candidate_score(p1));
  uint16_t t2 = clamp_tau(calculate_candidate_score(p2));

  /* COMMENTE : une ligne par comparaison, tres verbeux
  printf("RPL: OF-TAU comparing: tau1=%u (pc=%u) vs tau2=%u (pc=%u)\n",
         t1, pc1, t2, pc2);
  */

  /* ================================================================
   * CAS A : Un des candidats est le preferred_parent courant.
   * ================================================================ */
  if(dag != NULL && (p1 == dag->preferred_parent || p2 == dag->preferred_parent)) {

    rpl_parent_t *pref = dag->preferred_parent;
    rpl_parent_t *cand = (pref == p1) ? p2 : p1;
    uint16_t t_pref    = (pref == p1) ? t1 : t2;
    uint16_t t_cand    = (pref == p1) ? t2 : t1;
    uint16_t pc_pref   = (pref == p1) ? pc1 : pc2;
    uint16_t pc_cand   = (pref == p1) ? pc2 : pc1;

    /* COMMENTE : header + details internes LAYER1, trop verbeux
    printf("[LAYER1] ------ MRHOF+TAU decision ------\n");
    printf("[LAYER1] pref : tau=%u  pc=%u\n", t_pref, pc_pref);
    printf("[LAYER1] cand : tau=%u  pc=%u\n", t_cand, pc_cand);
    printf("[LAYER1] hyst=%u  mrhof_thr=%u\n",
           RPL_OF_TAU_SWITCH_THRESHOLD, MRHOF_PC_THRESHOLD);
    */

    /* --- COUCHE 1 : Decision classique MRHOF + TAU --- */
    rpl_parent_t *classic_result;
    {
      int pc_close = (pc_pref <= pc_cand + MRHOF_PC_THRESHOLD) &&
                     (pc_cand <= pc_pref + MRHOF_PC_THRESHOLD);

      if(pc_close) {
        /* COMMENTE : detail de la branche ETX
        printf("[LAYER1] ETX paths EQUIVALENT => TAU decides\n");
        */
        if(t_cand <= (uint16_t)(t_pref + RPL_OF_TAU_SWITCH_THRESHOLD)) {
          classic_result = pref;
          /* COMMENTE
          printf("[LAYER1] TAU: cand(%u)<=pref(%u)+hyst(%u) => STAY\n",
                 t_cand, t_pref, RPL_OF_TAU_SWITCH_THRESHOLD);
          */
        } else {
          classic_result = cand;
          /* COMMENTE
          printf("[LAYER1] TAU: cand(%u)>pref(%u)+hyst(%u) => SWITCH\n",
                 t_cand, t_pref, RPL_OF_TAU_SWITCH_THRESHOLD);
          */
        }
      } else if(pc_pref <= pc_cand) {
        /* COMMENTE
        printf("[LAYER1] ETX DIVERGE: pref_pc=%u < cand_pc=%u\n", pc_pref, pc_cand);
        */
        classic_result = (t_cand <= (uint16_t)(t_pref + RPL_OF_TAU_SWITCH_THRESHOLD))
                         ? pref : cand;
      } else {
        /* COMMENTE
        printf("[LAYER1] ETX DIVERGE: cand_pc=%u < pref_pc=%u\n", pc_cand, pc_pref);
        */
        classic_result = (t_pref <= (uint16_t)(t_cand + RPL_OF_TAU_SWITCH_THRESHOLD))
                         ? cand : pref;
      }
    }

    /* GARDE : decision classique — necessaire pour analyze.py */
    printf("[LAYER1] Classic decision => %s  (pref_tau=%u pc=%u | cand_tau=%u pc=%u)\n",
           classic_result == pref ? "STAY  " : "SWITCH",
           t_pref, pc_pref, t_cand, pc_cand);

    /* --- COUCHE 2 : Superviseur Q-Learning --- */
    uint8_t rl_state = compute_rl_state(t_pref, t_cand);

    if(rl_total_updates < RL_MIN_UPDATES_TO_TRUST) {
      /* GARDE : important pour analyze.py (bootstrap counter) */
      printf("[LAYER2] BOOTSTRAP MODE: %u/%u updates => using LAYER1\n",
             rl_total_updates, RL_MIN_UPDATES_TO_TRUST);
    } else {
      int16_t q_stay   = q_table[rl_state][RL_ACTION_STAY];
      int16_t q_switch = q_table[rl_state][RL_ACTION_SWITCH];

      /* COMMENTE : details Q-values affiches dans [RL-TABLE]
      printf("[LAYER2] state=%u  Q[STAY]=%d  Q[SWITCH]=%d  margin=%+d  threshold=%u\n",
             rl_state, q_stay, q_switch, (int)(q_switch - q_stay), RL_OVERRIDE_THRESHOLD);
      */

      if(classic_result == pref && q_switch > q_stay + RL_OVERRIDE_THRESHOLD) {
        /* GARDE : evenement important — override proactif */
        printf("[LAYER2] *** PROACTIVE SWITCH *** state=%u Q[STAY]=%d Q[SWITCH]=%d\n",
               rl_state, q_stay, q_switch);
        /* COMMENTE : sous-details redondants
        printf("[LAYER2]   Classic: STAY => QL overrides => SWITCH\n");
        */
        rl_last_state   = rl_state;
        rl_last_action  = RL_ACTION_SWITCH;
        rl_last_etx_raw = get_etx_or_default(cand);
        /* GARDE : decision finale apres override */
        printf("[OF-TAU] FINAL => SWITCH (QL proactive override)\n");
        return cand;

      } else if(classic_result == cand && q_stay > q_switch + RL_OVERRIDE_THRESHOLD) {
        /* GARDE : evenement important — protection preventive */
        printf("[LAYER2] *** PROTECTIVE STAY *** state=%u Q[STAY]=%d Q[SWITCH]=%d\n",
               rl_state, q_stay, q_switch);
        /* COMMENTE : sous-details redondants
        printf("[LAYER2]   Classic: SWITCH => QL blocks => STAY\n");
        */
        rl_last_state   = rl_state;
        rl_last_action  = RL_ACTION_STAY;
        rl_last_etx_raw = get_etx_or_default(pref);
        /* GARDE : decision finale apres override */
        printf("[OF-TAU] FINAL => STAY (QL protective override)\n");
        return pref;

      } else {
        /* GARDE : pour analyze.py (compteur uncertain) */
        printf("[LAYER2] QL UNCERTAIN: margin=%+d <= threshold(%u)\n",
               (int)(q_switch - q_stay), RL_OVERRIDE_THRESHOLD);
      }
    }

    /* Record action for next learning cycle */
    rl_last_state   = rl_state;
    rl_last_action  = (classic_result == cand) ? RL_ACTION_SWITCH : RL_ACTION_STAY;
    rl_last_etx_raw = get_etx_or_default(classic_result);

    /* COMMENTE : final LAYER1 (deja log dans "Classic decision =>")
    printf("[OF-TAU] FINAL => %s (LAYER1)\n",
           classic_result == pref ? "STAY  " : "SWITCH");
    */
    return classic_result;
  }

  /* ================================================================
   * CAS B : Pas de preferred_parent => selection directe MRHOF + TAU
   * ================================================================ */
  /* COMMENTE : peu frequent et sans interet pour analyze.py
  printf("[OF-TAU] No preferred parent => direct MRHOF+TAU selection\n");
  */
  {
    int pc_close = (pc1 <= pc2 + MRHOF_PC_THRESHOLD) &&
                   (pc2 <= pc1 + MRHOF_PC_THRESHOLD);

    if(pc_close) {
      /* COMMENTE
      printf("[OF-TAU] ETX equivalent => TAU decides: t1=%u t2=%u\n", t1, t2);
      */
      if(t2 > t1) return p2;
      if(t1 > t2) return p1;
    } else if(pc1 < pc2) {
      /* COMMENTE
      printf("[OF-TAU] ETX diverge: pc1=%u < pc2=%u\n", pc1, pc2);
      */
      int sw = t2 > (uint16_t)(t1 + RPL_OF_TAU_SWITCH_THRESHOLD);
      return sw ? p2 : p1;
    } else {
      /* COMMENTE
      printf("[OF-TAU] ETX diverge: pc2=%u < pc1=%u\n", pc2, pc1);
      */
      int sw = t1 > (uint16_t)(t2 + RPL_OF_TAU_SWITCH_THRESHOLD);
      return sw ? p1 : p2;
    }
  }

  /* Tie-breaks — COMMENTES : evenements rares, pas utiles pour analyze.py
  if(pc2 < pc1) { printf("[OF-TAU] Tie-break1: pc2 < pc1 => p2\n"); return p2; }
  if(pc1 < pc2) { printf("[OF-TAU] Tie-break1: pc1 < pc2 => p1\n"); return p1; }
  if(r2 < r1)   { printf("[OF-TAU] Tie-break2: r2 < r1 => p2\n");   return p2; }
  if(r1 < r2)   { printf("[OF-TAU] Tie-break2: r1 < r2 => p1\n");   return p1; }
  printf("[OF-TAU] Tie-break3: direct ETX\n");
  */
  if(pc2 < pc1) return p2;
  if(pc1 < pc2) return p1;
  if(r2 < r1)   return p2;
  if(r1 < r2)   return p1;
  return (parent_link_metric(p2) < parent_link_metric(p1)) ? p2 : p1;
}

/* ==============================================================================
   BEST_DAG & UPDATE_METRIC_CONTAINER
   ============================================================================== */

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
  instance->mc.type = RPL_DAG_MC_NONE;
}
/*---------------------------------------------------------------------------*/
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
