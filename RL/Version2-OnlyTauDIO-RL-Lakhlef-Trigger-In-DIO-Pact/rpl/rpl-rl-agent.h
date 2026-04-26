/*---------------------------------------------------------------------------*/
/* rpl-rl-agent.h
 *
 * Q-Learning based parent selection engine for RPL OF-TAU.
 *
 * Triggers:
 *   1. DIO received from current parent with tau_cand state-bucket change.
 *   2. Panic Monitor detecting link degradation (RSSI/ETX below thresholds).
 *
 * The RL agent makes the FINAL parent selection decision on every trigger.
 * OF-TAU best_parent() still runs for rank computation (DODAG loop-free).
 */
/*---------------------------------------------------------------------------*/

#ifndef RPL_RL_AGENT_H
#define RPL_RL_AGENT_H

#include "net/rpl/rpl.h"
#include "sys/clock.h"
#include "net/link-stats.h"

/*---------------------------------------------------------------------------*/
/* Tunable constants — override in project-conf.h                            */
/*---------------------------------------------------------------------------*/

/* Q-Learning rate: Alpha * 100 (e.g., 10 => Alpha=0.10) */
#ifndef RL_ALPHA
#define RL_ALPHA             10
#endif

/* Initial exploration rate * 100 (90 => epsilon=0.90) */
#ifndef RL_EPSILON_INITIAL
#define RL_EPSILON_INITIAL   90
#endif

/* Minimum exploration floor * 100 (10 => epsilon_min=0.10) */
#ifndef RL_EPSILON_MIN
#define RL_EPSILON_MIN       10
#endif

/* Epsilon decay factor * 100 (95 => decay=0.95) */
#ifndef RL_EPSILON_DECAY
#define RL_EPSILON_DECAY     95
#endif

/* Stability reward normalizer (in clock ticks) */
#ifndef RL_STABILITY_CONSTANT
#define RL_STABILITY_CONSTANT (30 * CLOCK_SECOND)
#endif

/* Maximum clipped reward value (×10 internally for fixed-point) */
#ifndef RL_MAX_REWARD
#define RL_MAX_REWARD        10
#endif

/* Neighbor considered dead after this long without a DIO */
#ifndef RL_NEIGHBOR_TIMEOUT
#define RL_NEIGHBOR_TIMEOUT  (3 * 60 * CLOCK_SECOND)
#endif

/* Panic monitor: RSSI threshold (dBm) — link is healthy if RSSI > this */
#ifndef RL_RSSI_THRESHOLD
#define RL_RSSI_THRESHOLD    (-85)
#endif

/* Panic monitor: ETX threshold — link is healthy if ETX < this */
#ifndef RL_ETX_THRESHOLD
#ifndef LINK_STATS_ETX_DIVISOR
#define LINK_STATS_ETX_DIVISOR 128
#endif
#define RL_ETX_THRESHOLD     (6 * LINK_STATS_ETX_DIVISOR)
#endif

/*---------------------------------------------------------------------------*/
/* Q-table dimensions                                                        */
/*---------------------------------------------------------------------------*/
#define RL_NUM_STATES   10
#define RL_NUM_ACTIONS  10
#define RL_TAU_STEP     100   /* Each bin covers [k*100, (k+1)*100) */

/*---------------------------------------------------------------------------*/
/* Public API                                                                */
/*---------------------------------------------------------------------------*/

/**
 * Initialize the Q-table to zero and reset epsilon.
 * Call once at node startup (lazy-init on first DIO).
 */
void rpl_rl_agent_init(void);

/**
 * Called after every DIO reception (in rpl_process_dio), once tau_cand is fresh.
 *
 * - Updates RL candidate fields: delta_rssi, last_rssi, time_last_dio.
 * - If p is the current parent and its tau bin changed, returns 1 (trigger needed).
 * - Otherwise returns 0.
 */
int rpl_rl_on_dio_received(rpl_dag_t *dag, rpl_parent_t *p, int16_t rssi_now);

/**
 * Execute the full Q-Learning parent selection procedure (Section 6.2 of RL.md).
 *
 * - Builds ActiveAction mask.
 * - Selects action via epsilon-greedy.
 * - Selects physical parent in chosen action range (best delta_rssi).
 * - If new != current: updates Q-table, decays epsilon, switches parent.
 * - Calls rpl_set_preferred_parent() + rpl_process_parent_event() internally.
 *
 * Returns the newly selected parent (may equal dag->preferred_parent).
 */
rpl_parent_t *rpl_rl_trigger(rpl_dag_t *dag);

#endif /* RPL_RL_AGENT_H */
