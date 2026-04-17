/*
 * Copyright (c) 2024
 * All rights reserved.
 *
 * \file
 *   RIMS-RPL: Reinforcement Learning-based Intelligent Mobility-Support
 *   Q-Learning agent for proactive link health monitoring.
 *
 *   This module runs a periodic ctimer (default 5s) that evaluates the
 *   link quality to the preferred parent and takes corrective actions
 *   (REPAIR, DIS, NONE) based on a 3×3 Q-Table updated via the
 *   Bellman equation, using fixed-point integer arithmetic.
 *
 * \author PFE Implementation
 */

#include "contiki.h"
#include "net/rpl/rpl-private.h"
#include "net/rpl/rpl-rims.h"
#include "net/link-stats.h"
#include "lib/random.h"

#include <string.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
/* Q-Table: 3 states × 3 actions, values scaled ×100 (fixed-point) */
static int16_t q_table[RIMS_NUM_STATES][RIMS_NUM_ACTIONS];

/* Periodic monitoring timer */
static struct ctimer rl_timer;

/* Previous ERP value for degradation detection */
static uint16_t prev_erp;

/* Parent change tracking counters */
static uint8_t bp_unchanged_count;
static uint8_t bp_changed_count;

/* Pointer to the last known preferred parent */
static rpl_parent_t *last_bp;

/* Flag to indicate if RIMS is running */
static uint8_t rims_running;

/*---------------------------------------------------------------------------*/
/**
 * \brief Calculate the ERP (Expected Reliability Percentage) for a parent.
 *        ERP = 100 - (12800 / etx)
 *        ETX=128 (perfect) → ERP=0, ETX=256 (50% PRR) → ERP=50
 *        Higher ERP = worse link quality.
 * \param p The parent to evaluate.
 * \return ERP value 0-100, or 100 if stats unavailable.
 */
static uint16_t
calculate_erp(rpl_parent_t *p)
{
  const struct link_stats *stats;

  if(p == NULL) {
    return 100; /* Worst case */
  }

  stats = rpl_get_parent_link_stats(p);
  if(stats == NULL || stats->etx == 0) {
    return 100; /* No stats available, assume worst */
  }

  /* ERP = 100 - (LINK_STATS_ETX_DIVISOR * 100) / etx
   * LINK_STATS_ETX_DIVISOR = 128 in Contiki-NG
   * For ETX=128 (perfect): ERP = 100 - 12800/128 = 0
   * For ETX=256 (50% PRR): ERP = 100 - 12800/256 = 50
   * For ETX=1024 (bad):    ERP = 100 - 12800/1024 = 87
   */
  uint16_t reliability = (uint16_t)((LINK_STATS_ETX_DIVISOR * 100UL) / stats->etx);

  if(reliability > 100) {
    reliability = 100;
  }

  return (uint16_t)(100 - reliability);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Determine the current RL state based on parent change counters.
 * \return One of RIMS_ST_OUTDATED, RIMS_ST_HANDOFF, RIMS_ST_STABLE.
 */
static uint8_t
get_current_state(void)
{
  if(bp_unchanged_count > RIMS_THRESHOLD_U) {
    return RIMS_ST_OUTDATED;
  } else if(bp_changed_count > RIMS_THRESHOLD_C) {
    return RIMS_ST_HANDOFF;
  } else {
    return RIMS_ST_STABLE;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Find the action with the highest Q-value for a given state.
 * \param state The current state index (0-2).
 * \return The action index (0-2) with the maximum Q-value.
 */
static uint8_t
argmax_action(uint8_t state)
{
  uint8_t best_action = 0;
  int16_t best_q = q_table[state][0];
  uint8_t a;

  for(a = 1; a < RIMS_NUM_ACTIONS; a++) {
    if(q_table[state][a] > best_q) {
      best_q = q_table[state][a];
      best_action = a;
    }
  }
  return best_action;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Find the maximum Q-value across all actions for a given state.
 * \param state The state index.
 * \return The maximum Q-value (scaled ×100).
 */
static int16_t
max_q_value(uint8_t state)
{
  int16_t max_q = q_table[state][0];
  uint8_t a;

  for(a = 1; a < RIMS_NUM_ACTIONS; a++) {
    if(q_table[state][a] > max_q) {
      max_q = q_table[state][a];
    }
  }
  return max_q;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Execute the chosen RL action.
 * \param action The action to execute (REPAIR, DIS, NONE).
 */
static void
execute_action(uint8_t action)
{
  rpl_instance_t *instance = default_instance;

  if(instance == NULL || instance->current_dag == NULL) {
    return;
  }

  switch(action) {
  case RIMS_ACT_REPAIR:
    PRINTF("RIMS: Executing ACT_REPAIR - nullifying parent, resetting DIO\n");
    /* Nullify the preferred parent to force re-selection */
    if(instance->current_dag->preferred_parent != NULL) {
      rpl_nullify_parent(instance->current_dag->preferred_parent);
    }
    /* Reset the Trickle timer to accelerate DIO exchange */
    rpl_reset_dio_timer(instance);
    /* Reset change counters after repair */
    bp_unchanged_count = 0;
    bp_changed_count = 0;
    break;

  case RIMS_ACT_DIS:
    PRINTF("RIMS: Executing ACT_DIS - sending DIS\n");
    /* Send a multicast DIS to solicit fresh DIOs from neighbors */
    dis_output(NULL);
    break;

  case RIMS_ACT_NONE:
    PRINTF("RIMS: Executing ACT_NONE\n");
    /* Do nothing, link is considered acceptable */
    break;

  default:
    break;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Main Q-Learning monitoring callback.
 *        Called periodically by the ctimer.
 *        Evaluates link health and takes corrective actions
 *        using the Q-Learning policy.
 */
static void
handle_rl_monitoring(void *ptr)
{
  rpl_instance_t *instance = default_instance;
  rpl_parent_t *current_bp;
  uint16_t current_erp;
  uint8_t current_state;
  uint8_t action;
  uint8_t next_state;
  uint16_t new_erp;
  int16_t reward;
  uint16_t k;

  /* Safety check: only run if we're in a DODAG */
  if(instance == NULL || instance->current_dag == NULL) {
    ctimer_reset(&rl_timer);
    return;
  }

  current_bp = instance->current_dag->preferred_parent;

  /* Update parent change counters */
  if(current_bp == last_bp) {
    if(bp_unchanged_count < 255) {
      bp_unchanged_count++;
    }
    bp_changed_count = 0; /* Reset changed counter on stability */
  } else {
    if(bp_changed_count < 255) {
      bp_changed_count++;
    }
    bp_unchanged_count = 0; /* Reset unchanged counter on change */
    last_bp = current_bp;
  }

  /* Calculate current ERP of the preferred parent */
  if(current_bp != NULL) {
    current_erp = calculate_erp(current_bp);
  } else {
    current_erp = 100; /* No parent = worst case */
  }

  /* Only trigger RL if link quality has degraded */
  if(current_erp > prev_erp) {
    PRINTF("RIMS: ERP degraded %u -> %u, triggering RL\n", prev_erp, current_erp);

    current_state = get_current_state();

    /* Epsilon-greedy action selection */
    k = random_rand() % RIMS_FP_SCALE; /* k in [0, 99] */

    if(k < RIMS_RL_EPSILON) {
      /* EXPLORATION: use heuristic based on mobility counters */
      PRINTF("RIMS: Exploration mode\n");
      if(bp_unchanged_count > RIMS_THRESHOLD_U) {
        action = RIMS_ACT_REPAIR;
      } else if(bp_changed_count > RIMS_THRESHOLD_C) {
        action = RIMS_ACT_DIS;
      } else {
        action = RIMS_ACT_NONE;
      }
    } else {
      /* EXPLOITATION: pick the best action from Q-Table */
      PRINTF("RIMS: Exploitation mode\n");
      action = argmax_action(current_state);
    }

    /* Execute the chosen action */
    execute_action(action);

    /* Recalculate ERP after action (may have changed parent) */
    current_bp = (instance->current_dag != NULL) ?
                  instance->current_dag->preferred_parent : NULL;
    if(current_bp != NULL) {
      new_erp = calculate_erp(current_bp);
    } else {
      new_erp = 100;
    }

    /* Calculate reward: 1 (×SCALE) if ERP improved or stable, 0 otherwise */
    reward = (new_erp <= current_erp) ? RIMS_FP_SCALE : 0;

    /* Determine next state */
    next_state = get_current_state();

    /* Bellman equation update (all values scaled ×100):
     * Q[s][a] += ALPHA * (R + GAMMA * max(Q[s']) - Q[s][a]) / SCALE
     *
     * With ALPHA=40, GAMMA=30, SCALE=100:
     * delta = reward + (GAMMA * maxQ_next / SCALE) - Q[s][a]
     * Q[s][a] += (ALPHA * delta) / SCALE
     */
    {
      int16_t max_q_next = max_q_value(next_state);
      int16_t delta = reward
                    + (int16_t)((int32_t)RIMS_RL_GAMMA * max_q_next / RIMS_FP_SCALE)
                    - q_table[current_state][action];
      q_table[current_state][action] +=
        (int16_t)((int32_t)RIMS_RL_ALPHA * delta / RIMS_FP_SCALE);
    }

    PRINTF("RIMS: Q[%u][%u]=%d, reward=%d, new_erp=%u\n",
           current_state, action, q_table[current_state][action],
           (int)reward, new_erp);

    /* Update previous ERP to the new measurement */
    prev_erp = new_erp;
  } else {
    /* Link is stable or improving, just track */
    prev_erp = current_erp;
  }

  /* Reschedule the timer */
  ctimer_reset(&rl_timer);
}
/*---------------------------------------------------------------------------*/
void
rims_init(void)
{
  /* Zero the Q-table */
  memset(q_table, 0, sizeof(q_table));

  /* Reset all counters */
  prev_erp = 0;
  bp_unchanged_count = 0;
  bp_changed_count = 0;
  last_bp = NULL;
  rims_running = 0;

  PRINTF("RIMS: Initialized (ALPHA=%u, GAMMA=%u, EPSILON=%u)\n",
         RIMS_RL_ALPHA, RIMS_RL_GAMMA, RIMS_RL_EPSILON);
}
/*---------------------------------------------------------------------------*/
void
rims_start(void)
{
  if(!rims_running) {
    rims_running = 1;
    ctimer_set(&rl_timer, RIMS_MONITOR_INTERVAL,
               handle_rl_monitoring, NULL);
    PRINTF("RIMS: Monitoring started (interval %lu ticks)\n",
           (unsigned long)RIMS_MONITOR_INTERVAL);
  }
}
/*---------------------------------------------------------------------------*/
void
rims_stop(void)
{
  if(rims_running) {
    rims_running = 0;
    ctimer_stop(&rl_timer);
    PRINTF("RIMS: Monitoring stopped\n");
  }
}
/*---------------------------------------------------------------------------*/
void
rims_notify_parent_change(void)
{
  /* Called externally when a parent switch occurs.
   * This increments the change counter to influence state detection. */
  if(bp_changed_count < 255) {
    bp_changed_count++;
  }
  bp_unchanged_count = 0;
}
