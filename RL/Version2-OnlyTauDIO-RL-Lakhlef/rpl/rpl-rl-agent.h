/*---------------------------------------------------------------------------*/
/* rpl-rl-agent.h
 *
 * Q-Learning based parent selection engine for RPL OF-TAU.
 *
 * Entry point: rpl_rl_on_dio_received() — called from the DIO reception
 * hook in rpl-dag.c.  Implements a three-gate decision flow:
 *   Gate 1: Is the current parent degraded / is a neighbor worth evaluating?
 *   Gate 2: Does the Q-table justify a switch (Delta_Q >= RL_MIN_SWITCH_GAIN)?
 *   Gate 3: Physical hysteresis — is the candidate clearly better?
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
/* Three-Gate Decision Flow constants (override in project-conf.h)           */
/*---------------------------------------------------------------------------*/

/* Gate 1B: RSSI below which current parent is considered weak (dBm) */
#ifndef RL_RSSI_WEAK_THRESHOLD
#define RL_RSSI_WEAK_THRESHOLD  (-85)
#endif

/* Gate 1B: ETX above which current parent is considered weak */
#ifndef RL_ETX_WEAK_THRESHOLD
#define RL_ETX_WEAK_THRESHOLD   (3 * LINK_STATS_ETX_DIVISOR)
#endif

/* Gate 2: Minimum Q-gain (×10 stored format) to justify a switch */
#ifndef RL_MIN_SWITCH_GAIN
#define RL_MIN_SWITCH_GAIN      3   /* = 0.3 in real units */
#endif

/* Gate 3: Minimum TAU advantage for candidate to justify switch */
#ifndef RL_HYSTERESIS_TAU
#define RL_HYSTERESIS_TAU       20
#endif

/* Gate 3: Minimum RSSI advantage in dBm */
#ifndef RL_HYSTERESIS_RSSI
#define RL_HYSTERESIS_RSSI      3
#endif

/* Gate 3: Minimum ETX improvement (in raw link-stats units) */
#ifndef RL_HYSTERESIS_ETX
#define RL_HYSTERESIS_ETX       64   /* = 0.5 × LINK_STATS_ETX_DIVISOR */
#endif

/*---------------------------------------------------------------------------*/
/* Learning Phase (Off-Policy Pre-Training)                                  */
/*---------------------------------------------------------------------------*/

/* Duration of the learning phase in seconds.
 * After this time has elapsed since first boot the agent enters
 * production mode automatically.
 * Rule of thumb: cover at least 3 full trickle timer cycles at
 * DIO_INTERVAL_MAX so every neighbor has sent several DIOs and
 * the candidate table is fully populated.                                   */
#ifndef RL_LEARNING_DURATION_S
#define RL_LEARNING_DURATION_S      300u      /* 5 minutes */
#endif
#define RL_LEARNING_DURATION \
    ((clock_time_t)(RL_LEARNING_DURATION_S) * CLOCK_SECOND)

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
 * Two-phase architecture:
 *   LEARNING (clock_time() - rl_learning_start < RL_LEARNING_DURATION):
 *     Deterministic parent selection (best tau_cand wins).
 *     Q-table updated by observing each switch (off-policy pre-training).
 *
 *   PRODUCTION (learning duration elapsed):
 *     Full three-gate decision flow with epsilon-greedy RL.
 *     Gate 1 → RL block (Q-update + decay) → Gate 2 → Gate 3 → Commit.
 *
 * In both modes, the Q-table update ALWAYS executes on every switch.
 */
void rpl_rl_on_dio_received(rpl_dag_t *dag, rpl_parent_t *p, int16_t rssi_now);

#endif /* RPL_RL_AGENT_H */

