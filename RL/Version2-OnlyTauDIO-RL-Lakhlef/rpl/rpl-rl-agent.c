/*---------------------------------------------------------------------------*/
/* rpl-rl-agent.c
 *
 * Q-Learning based parent selection for RPL OF-TAU (Version2-OnlyTauDIO).
 *
 * Two-phase architecture:
 *   LEARNING (clock_time() - rl_learning_start < RL_LEARNING_DURATION):
 *     Off-policy pre-training. Parent selection is deterministic (best
 *     tau_cand wins). The Q-table is silently updated by observing each
 *     switch, so it is pre-populated with meaningful experience.
 *
 *   PRODUCTION (learning duration elapsed):
 *     Full three-gate RL decision flow with epsilon-greedy policy.
 *     Gate 1 → RL block → Gate 2 (Delta_Q) → Gate 3 (hysteresis) → Commit.
 *
 * Implementation notes:
 *   - Q-values stored as int16_t scaled x10 (preserves 1 decimal place).
 *   - Alpha, epsilon stored as integer percentages (×100).
 *   - All floating-point avoided for Contiki/Z1 compatibility.
 *   - tau_cand computation is NOT modified; uses rpl_tau_compute_cand().
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "net/rpl/rpl-private.h"
#include "net/rpl/rpl-rl-agent.h"
#include "net/link-stats.h"
#include "lib/random.h"
#include "sys/clock.h"

#include <string.h>
#include <stdint.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
/* External functions                                                        */
/*---------------------------------------------------------------------------*/
extern uint16_t calculate_candidate_score(rpl_parent_t *p);

/*---------------------------------------------------------------------------*/
/* Internal state                                                             */
/*---------------------------------------------------------------------------*/

/* Q-table: q_table[state][action], values scaled ×10. */
static int16_t q_table[RL_NUM_STATES][RL_NUM_ACTIONS];

/* Current exploration rate (×100, so 90 means ε=0.90) */
static uint8_t epsilon;

/* Lazy-init flag */
static uint8_t rl_initialized = 0;

/* Counts how many parent switches have been committed.
 * Kept for logging only — not used as mode gate.
 * Never reset after initialization.                                    */
static uint16_t rl_switch_count = 0;

/* Timestamp recorded on the very first call to ensure_init().
 * The learning phase runs from this point for RL_LEARNING_DURATION.
 * Never reset after initialization.                                    */
static clock_time_t rl_learning_start = 0;

/* True while the agent is still in learning mode.                      */
#define RL_IS_LEARNING() \
    (clock_time() - rl_learning_start < RL_LEARNING_DURATION)

/*---------------------------------------------------------------------------*/
/* Internal helpers                                                           */
/*---------------------------------------------------------------------------*/

static void
ensure_init(void)
{
  if(!rl_initialized) {
    rpl_rl_agent_init();
    rl_learning_start = clock_time();   /* start the learning clock */
    printf("[RL] Initialized. Learning phase duration: %lu s.\n",
           (unsigned long)RL_LEARNING_DURATION_S);
  }
}
/*---------------------------------------------------------------------------*/
static uint8_t
get_bin(uint16_t tau_cand)
{
  uint8_t bin = (uint8_t)(tau_cand / RL_TAU_STEP);
  if(bin >= RL_NUM_STATES) {
    bin = RL_NUM_STATES - 1;
  }
  return bin;
}
/*---------------------------------------------------------------------------*/
/**
 * Composite tie-break score (higher = better candidate).
 * ETX is already a smoothed EMA from link-stats — use it as primary signal.
 * delta_rssi is instantaneous — use it only as a secondary tiebreak.
 * (Secret 3 — MRHOF: use smoothed ETX from link-stats EMA)
 */
static uint16_t
candidate_score(rpl_parent_t *p)
{
  uint16_t etx_norm  = rpl_etx_norm(p);          /* already in [0,1000] */
  int16_t  d_rssi    = p->rl_delta_rssi;
  int16_t  rssi_contrib = (int16_t)((d_rssi + 20) * 10); /* map [-20,+20] → [0,400] */
  if(rssi_contrib < 0)   rssi_contrib = 0;
  if(rssi_contrib > 400) rssi_contrib = 400;

  /* ETX contributes 70%, delta_rssi trend contributes 30% */
  return (uint16_t)((etx_norm * 7 + (uint16_t)rssi_contrib * 3) / 10);
}
/*---------------------------------------------------------------------------*/
/**
 * Called at the start of every switch (learning or production) to record
 * the reward for the PREVIOUS decision before committing the new one.
 *
 * prev_state  = rl_state_S recorded on the outgoing parent at attachment time
 * prev_action = get_bin(rl_tau_at_choice) of the outgoing parent
 * reward      = capped connection duration
 */
static void
update_q_table(rpl_parent_t *current)
{
  if(current == NULL || current->rl_time_attachment == 0) {
    return;
  }

  clock_time_t conn_time = clock_time() - current->rl_time_attachment;
  int16_t reward = (int16_t)(conn_time / RL_STABILITY_CONSTANT);
  if(reward > RL_MAX_REWARD) { reward = RL_MAX_REWARD; }

  uint8_t prev_state  = current->rl_state_S;
  uint8_t prev_action = get_bin(current->rl_tau_at_choice);

  int16_t old_q = q_table[prev_state][prev_action];
  int32_t delta = (int32_t)(reward * 10) - (int32_t)old_q;
  int16_t new_q = (int16_t)(old_q + (int16_t)((delta * (int32_t)RL_ALPHA) / 100L));
  q_table[prev_state][prev_action] = new_q;

  printf("[RL] Q[%u][%u]: %d -> %d (reward=%d, mode=%s)\n",
         prev_state, prev_action, old_q, new_q, (int)reward,
         RL_IS_LEARNING() ? "LEARNING" : "PRODUCTION");
}
/*---------------------------------------------------------------------------*/
/* Public API                                                                 */
/*---------------------------------------------------------------------------*/

void
rpl_rl_agent_init(void)
{
  memset(q_table, 0, sizeof(q_table));
  epsilon = RL_EPSILON_INITIAL;
  rl_initialized = 1;
  printf("[RL] Agent initialized. epsilon=%u/100\n", epsilon);
}
/*---------------------------------------------------------------------------*/
/**
 * Build the ActiveAction mask.
 * mask[a] = 1 iff at least one fresh candidate has tau_cand in bucket a.
 */
static void
build_active_mask(rpl_dag_t *dag, uint8_t mask[RL_NUM_ACTIONS])
{
  rpl_parent_t *p;
  clock_time_t now = clock_time();

  memset(mask, 0, RL_NUM_ACTIONS * sizeof(uint8_t));

  for(p = nbr_table_head(rpl_parents); p != NULL;
      p = nbr_table_next(rpl_parents, p)) {

    if(p->dag != dag) { continue; }
    if(p->rank == INFINITE_RANK) { continue; }
    if(p->tau_cand == 0) { continue; }
    if(p->rl_time_last_dio == 0) { continue; }
    if((now - p->rl_time_last_dio) > RL_NEIGHBOR_TIMEOUT) { continue; }

    mask[get_bin(p->tau_cand)] = 1;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * Choose an action using ε-greedy policy (production mode only).
 * Returns chosen action index (0-9) or -1 if no active action exists.
 */
static int8_t
choose_action(uint8_t current_state, const uint8_t mask[RL_NUM_ACTIONS])
{
  uint8_t i;
  uint8_t n_active = 0;

  for(i = 0; i < RL_NUM_ACTIONS; i++) {
    if(mask[i]) { n_active++; }
  }

  if(n_active == 0) {
    return -1;
  }

  uint8_t x = (uint8_t)(random_rand() % 100);

  if(x < epsilon) {
    /* --- Explore: uniform random over active actions --- */
    uint8_t pick = (uint8_t)(random_rand() % n_active);
    uint8_t count = 0;
    for(i = 0; i < RL_NUM_ACTIONS; i++) {
      if(mask[i]) {
        if(count == pick) {
          return (int8_t)i;
        }
        count++;
      }
    }
  } else {
    /* --- Exploit: argmax Q[state][a] over active actions --- */
    int8_t  best_action = -1;
    int16_t best_q      = -32767;
    for(i = 0; i < RL_NUM_ACTIONS; i++) {
      if(!mask[i]) { continue; }
      if(best_action == -1 || q_table[current_state][i] > best_q) {
        best_q      = q_table[current_state][i];
        best_action = (int8_t)i;
      }
    }
    return best_action;
  }

  return -1;
}
/*---------------------------------------------------------------------------*/
/**
 * Learning-mode parent selection.
 * Returns the neighbor with the highest tau_cand among all candidates
 * that pass the freshness and usability checks.
 * Returns NULL if no valid candidate exists.
 */
static rpl_parent_t *
select_best_tau_parent(rpl_dag_t *dag)
{
  rpl_parent_t *best     = NULL;
  uint16_t      best_tau = 0;
  clock_time_t  now      = clock_time();

  rpl_parent_t *p;
  for(p = nbr_table_head(rpl_parents); p != NULL;
      p = nbr_table_next(rpl_parents, p)) {

    if(p->dag != dag)                                    { continue; }
    if(p->rank == INFINITE_RANK)                         { continue; }
    if(p->tau_cand == 0)                                 { continue; }
    if(p->rl_time_last_dio == 0)                         { continue; }
    if((now - p->rl_time_last_dio) > RL_NEIGHBOR_TIMEOUT) { continue; }

    if(p->tau_cand > best_tau) {
      best_tau = p->tau_cand;
      best     = p;
    }
  }
  return best;
}
/*---------------------------------------------------------------------------*/
/**
 * Select the physical parent from candidates in the chosen action range.
 * Tie-breaking: highest composite candidate_score() (Secret 3).
 */
static rpl_parent_t *
select_physical_parent(rpl_dag_t *dag, uint8_t chosen_action)
{
  rpl_parent_t *p;
  rpl_parent_t *best = NULL;
  uint16_t      best_score = 0;
  clock_time_t  now = clock_time();

  for(p = nbr_table_head(rpl_parents); p != NULL;
      p = nbr_table_next(rpl_parents, p)) {

    if(p->dag != dag) { continue; }
    if(p->rank == INFINITE_RANK) { continue; }
    if(p->rl_time_last_dio == 0) { continue; }
    if((now - p->rl_time_last_dio) > RL_NEIGHBOR_TIMEOUT) { continue; }
    if(get_bin(p->tau_cand) != chosen_action) { continue; }

    uint16_t sc = candidate_score(p);
    if(best == NULL || sc > best_score) {
      best = p;
      best_score = sc;
    }
  }

  return best;
}
/*---------------------------------------------------------------------------*/
/**
 * Internal trigger — two-branch structure (LEARNING / PRODUCTION).
 *
 * LEARNING:  Deterministic selection (best tau_cand), Q-table observation.
 * PRODUCTION: Full RL with epsilon-greedy, Gate 2, Gate 3.
 *
 * In both modes, update_q_table() is called on every switch.
 */
static rpl_parent_t *
rpl_rl_trigger(rpl_dag_t *dag)
{
  if(dag == NULL || dag->instance == NULL) { return NULL; }
  ensure_init();

  rpl_parent_t *current = dag->preferred_parent;

  /* ------------------------------------------------------------------ */
  if(RL_IS_LEARNING()) {
  /* ================================================================== */
  /*  LEARNING MODE                                                       */
  /*  Select parent by best tau_cand (deterministic, proven rule).       */
  /*  Update Q-table by observing the outcome of this selection.         */
  /* ================================================================== */

    rpl_parent_t *new_parent = select_best_tau_parent(dag);

    if(new_parent == NULL) {
      printf("[RL:LEARN] No valid candidate found.\n");
      return current;
    }

    if(new_parent == current) {
      printf("[RL:LEARN] Best tau is already current parent (tau=%u).\n",
             current ? current->tau_cand : 0u);
      return current;
    }

    /* Record reward for the previous cycle and update Q-table */
    update_q_table(current);

    /* Commit the switch */
    rl_switch_count++;

    printf("[RL:LEARN] Switch #%u -> tau=%u bin=%u (t=%lu s remaining)\n",
           rl_switch_count,
           new_parent->tau_cand,
           get_bin(new_parent->tau_cand),
           (unsigned long)((RL_LEARNING_DURATION -
               (clock_time() - rl_learning_start)) / CLOCK_SECOND));

    /* Record RL metadata on the new parent so the next update_q_table()
     * call can retrieve the state and action of THIS decision.         */
    new_parent->rl_time_attachment = clock_time();
    new_parent->rl_state_S         = get_bin(new_parent->tau_cand);
    new_parent->rl_tau_at_choice   = new_parent->tau_cand;

    rpl_set_preferred_parent(dag, new_parent);
    dag->rank = rpl_rank_via_parent(new_parent);
    return new_parent;

  } else {
  /* ================================================================== */
  /*  PRODUCTION MODE                                                     */
  /*  Full RL decision flow with all three gates.                        */
  /* ================================================================== */

    /* Log the first entry into production mode */
    static uint8_t production_announced = 0;
    if(!production_announced) {
      printf("[RL] *** LEARNING COMPLETE after %lu s and %u switches. "
             "Entering PRODUCTION mode. ***\n",
             (unsigned long)RL_LEARNING_DURATION_S,
             rl_switch_count);
      production_announced = 1;
    }

    /* Step 1: Build ActiveAction mask */
    uint8_t mask[RL_NUM_ACTIONS];
    build_active_mask(dag, mask);

    /* Step 2: Re-evaluate current state */
    uint8_t new_state = (current != NULL) ? get_bin(current->tau_cand) : 0;
    uint8_t A_current = (current != NULL) ? get_bin(current->tau_cand) : 0;

    /* Step 3: Choose action (epsilon-greedy) */
    int8_t chosen_action = choose_action(new_state, mask);
    if(chosen_action < 0) {
      printf("[RL:PROD] No active candidates. Abort.\n");
      return current;
    }

    printf("[RL:PROD] State S=%u -> chose action A=%d (eps=%u/100)\n",
           new_state, chosen_action, epsilon);

    /* Step 4: Always update Q-table (even if no switch happens) */
    update_q_table(current);

    /* Decay epsilon */
    uint16_t new_eps = ((uint16_t)epsilon * (uint16_t)RL_EPSILON_DECAY) / 100u;
    epsilon = (uint8_t)((new_eps < RL_EPSILON_MIN) ? RL_EPSILON_MIN : new_eps);

    /* Gate 2 — Delta_Q */
    int16_t Q_switch = q_table[new_state][(uint8_t)chosen_action];
    int16_t Q_stay   = q_table[new_state][A_current];
    int16_t gain     = Q_switch - Q_stay;

    if(gain < RL_MIN_SWITCH_GAIN) {
      if(current != NULL) { current->rl_state_S = new_state; }
      printf("[RL:PROD] Gate 2 blocked. gain=%d < %d\n",
             gain, RL_MIN_SWITCH_GAIN);
      return current;
    }

    /* Step 5: Select physical parent */
    rpl_parent_t *new_parent = select_physical_parent(dag, (uint8_t)chosen_action);
    if(new_parent == NULL || new_parent == current) {
      if(current != NULL) { current->rl_state_S = new_state; }
      return current;
    }

    /* Gate 3 — Physical hysteresis */
    if(current != NULL) {
      int16_t  delta_tau  = (int16_t)new_parent->tau_cand
                          - (int16_t)current->tau_cand;
      int16_t  delta_rssi = (int16_t)new_parent->rl_last_rssi
                          - (int16_t)current->rl_last_rssi;
      uint16_t cur_etx    = rpl_get_parent_link_metric(current);
      uint16_t new_etx    = rpl_get_parent_link_metric(new_parent);
      int16_t  delta_etx  = (int16_t)cur_etx - (int16_t)new_etx;

      int current_still_acceptable =
        (current->rl_last_rssi > (int16_t)RL_RSSI_WEAK_THRESHOLD) &&
        (cur_etx               < (uint16_t)RL_ETX_WEAK_THRESHOLD);

      int16_t delta_tau;
      int16_t delta_rssi;
      uint32_t cur_etx_raw;
      uint32_t v_etx_raw;
      uint32_t cur_etx_sq;
      uint32_t v_etx_sq;
      int32_t delta_etx_sq;
      int32_t confidence;
      int candidate_clearly_better;

      /* 1. Recalculate fresh TAU for the current parent to prevent stale comparison */
      current->tau_cand = calculate_candidate_score(current);

      /* 2. Calculate physical differences */
      delta_tau  = (int16_t)v->tau_cand - (int16_t)current->tau_cand;
      delta_rssi = (int16_t)v->rl_last_rssi - (int16_t)current->rl_last_rssi;
      
      /* 3. Apply MRHOF Secret 4: Squared ETX */
      cur_etx_raw = cur_etx;
      v_etx_raw   = rpl_get_parent_link_metric(v);
      
      cur_etx_sq = (cur_etx_raw * cur_etx_raw) / LINK_STATS_ETX_DIVISOR;
      v_etx_sq   = (v_etx_raw * v_etx_raw) / LINK_STATS_ETX_DIVISOR;
      
      delta_etx_sq = (int32_t)cur_etx_sq - (int32_t)v_etx_sq; /* Positive if V is better */

      /* 4. Calculate Composite Confidence Score */
      confidence = (int32_t)delta_tau;
      confidence += (delta_rssi * 2);
      if (delta_etx_sq < 0) {
          confidence += (delta_etx_sq * 2); /* Double penalty for worse ETX */
      } else {
          confidence += delta_etx_sq;       /* Normal bonus for better ETX */
      }

      /* 5. Final Decision */
      candidate_clearly_better = (confidence > (int32_t)RL_HYSTERESIS_TAU);

      if(current_still_acceptable && !candidate_clearly_better) {
        current->rl_state_S = new_state;
        printf("[RL:PROD] Gate 3 blocked. Current acceptable, "
               "candidate not convincingly better.\n");
        return current;
      }
    }

    /* All gates passed: commit the switch */
    rl_switch_count++;

    printf("[RL:PROD] Switch #%u -> tau=%u bin=%u eps=%u/100\n",
           rl_switch_count, new_parent->tau_cand,
           get_bin(new_parent->tau_cand), epsilon);

    new_parent->rl_time_attachment = clock_time();
    new_parent->rl_state_S         = get_bin(new_parent->tau_cand);
    new_parent->rl_tau_at_choice   = new_parent->tau_cand;

    rpl_set_preferred_parent(dag, new_parent);
    dag->rank = rpl_rank_via_parent(new_parent);
    return new_parent;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * Entry point — called on every DIO reception.
 *
 * Step 0: Always update candidate table.
 * Gate 1: Decide whether to invoke the RL trigger.
 *   - No current parent → always trigger (initial join).
 *   - V is current parent & link degraded → trigger.
 *   - V is other neighbor & current parent weak → trigger.
 *
 * The trigger function handles both LEARNING and PRODUCTION modes internally.
 */
void
rpl_rl_on_dio_received(rpl_dag_t *dag, rpl_parent_t *v, int16_t measured_rssi)
{
  if(dag == NULL || v == NULL || dag->instance == NULL) {
    return;
  }

  ensure_init();

  rpl_parent_t *current = dag->preferred_parent;

  /* ===== Step 0: Always update candidate table (no condition) ===== */
  v->rl_delta_rssi    = measured_rssi - v->rl_last_rssi;
  v->rl_last_rssi     = measured_rssi;
  v->rl_time_last_dio = clock_time();
  /* tau_cand is already computed in rpl-dag.c before calling us */

  /* ===== Gate 1: Decide whether to invoke RL trigger ===== */
  if(current == NULL) {
    /* No preferred parent yet — must select one */
    rpl_rl_trigger(dag);
    return;
  }

  if(v == current) {
    /* --- CASE A: DIO from current parent --- */
    uint8_t new_state = get_bin(v->tau_cand);
    if(new_state >= current->rl_state_S) {
      /* Link is stable or improving — update state, do nothing else */
      current->rl_state_S = new_state;
      return;
    }
    /* Link degraded → trigger RL */
    printf("[RL] Current parent degraded: S=%u -> S=%u. Triggering.\n",
           current->rl_state_S, new_state);
    rpl_rl_trigger(dag);

  } else {
    /* --- CASE B: DIO from another neighbor --- */
    uint16_t current_etx  = rpl_get_parent_link_metric(current);
    int16_t  current_rssi = current->rl_last_rssi;

    int current_is_weak =
      (current_rssi < (int16_t)RL_RSSI_WEAK_THRESHOLD) ||
      (current_etx  > (uint16_t)RL_ETX_WEAK_THRESHOLD);

    int16_t delta_tau;
    int16_t delta_rssi;
    uint32_t cur_etx_raw;
    uint32_t v_etx_raw;
    uint32_t cur_etx_sq;
    uint32_t v_etx_sq;
    int32_t delta_etx_sq;
    int32_t confidence;
    int candidate_is_much_better;

    /* 1. Recalculate fresh TAU for the current parent to prevent stale comparison */
    current->tau_cand = calculate_candidate_score(current);

    /* 2. Calculate physical differences */
    delta_tau  = (int16_t)v->tau_cand - (int16_t)current->tau_cand;
    delta_rssi = (int16_t)v->rl_last_rssi - (int16_t)current_rssi;
    
    /* 3. Apply MRHOF Secret 4: Squared ETX */
    cur_etx_raw = current_etx;
    v_etx_raw   = rpl_get_parent_link_metric(v);
    
    cur_etx_sq = (cur_etx_raw * cur_etx_raw) / LINK_STATS_ETX_DIVISOR;
    v_etx_sq   = (v_etx_raw * v_etx_raw) / LINK_STATS_ETX_DIVISOR;
    
    delta_etx_sq = (int32_t)cur_etx_sq - (int32_t)v_etx_sq; /* Positive if V is better */

    /* 4. Calculate Composite Confidence Score */
    confidence = (int32_t)delta_tau;
    confidence += (delta_rssi * 2);
    if (delta_etx_sq < 0) {
        confidence += (delta_etx_sq * 2); /* Double penalty for worse ETX */
    } else {
        confidence += delta_etx_sq;       /* Normal bonus for better ETX */
    }

    /* 5. Final Decision */
    candidate_is_much_better = (confidence > (int32_t)RL_HYSTERESIS_TAU);

    if(current_is_weak || candidate_is_much_better) {
      printf("[RL] Triggering on DIO. weak=%d, much_better=%d. \n",
             current_is_weak, candidate_is_much_better);
      rpl_rl_trigger(dag);
    }
    /* else: candidate table updated, nothing more needed */
  }
}
/*---------------------------------------------------------------------------*/
