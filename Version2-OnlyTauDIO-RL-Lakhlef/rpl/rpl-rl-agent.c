/*---------------------------------------------------------------------------*/
/* rpl-rl-agent.c
 *
 * Q-Learning based parent selection for RPL OF-TAU (Version2-OnlyTauDIO).
 *
 * State  : tau_cand of current parent, discretized into 10 bins of 100.
 * Action : target tau_cand bin for the new parent.
 * Reward : MIN(connection_time / STABILITY_CONSTANT, MAX_REWARD).
 * Policy : epsilon-greedy with decaying epsilon (floor = RL_EPSILON_MIN).
 *
 * Q-update (TD-0):  Q[S][A] += Alpha * (Reward - Q[S][A])
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
/* Internal state                                                             */
/*---------------------------------------------------------------------------*/

/* Q-table: q_table[state][action], values scaled ×10.
 * Range: a reward of MAX_REWARD=10 gives 10*10=100 in raw units.
 * int16_t is sufficient (-32768..32767).                                    */
static int16_t q_table[RL_NUM_STATES][RL_NUM_ACTIONS];

/* Current exploration rate (×100, so 90 means ε=0.90) */
static uint8_t epsilon;

/* Lazy-init flag */
static uint8_t rl_initialized = 0;

/*---------------------------------------------------------------------------*/
/* Internal helpers                                                           */
/*---------------------------------------------------------------------------*/

static void
ensure_init(void)
{
  if(!rl_initialized) {
    rpl_rl_agent_init();
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
 * Called on every DIO reception after p->tau_cand is updated.
 *
 * Case 1 (current parent): update RL fields + detect state change.
 *   Returns 1 if state changed (trigger needed), 0 otherwise.
 * Case 2 (other neighbor): only update RL candidate fields. Returns 0.
 */
int
rpl_rl_on_dio_received(rpl_dag_t *dag, rpl_parent_t *p, int16_t rssi_now)
{
  if(dag == NULL || p == NULL) {
    return 0;
  }

  ensure_init();

  /* --- Update RL Candidate Table fields (Section 4.1) --- */
  p->rl_delta_rssi    = rssi_now - p->rl_last_rssi;
  p->rl_last_rssi     = rssi_now;
  p->rl_time_last_dio = clock_time();

  /* --- Case 1: DIO from the current preferred parent --- */
  if(p == dag->preferred_parent) {
    uint8_t new_state = get_bin(p->tau_cand);

    if(new_state == p->rl_state_S) {
      /* No state bucket change → no RL trigger needed */
      return 0;
    }

    /* State changed → update and signal caller to trigger RL */
    printf("[RL] Current parent tau-bin changed: S=%u -> S=%u. Trigger pending.\n",
           p->rl_state_S, new_state);
    p->rl_state_S = new_state;
    return 1;  /* Trigger RL */
  }

  /* Case 2: other neighbor — candidate table already updated above */
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * Build the ActiveAction mask (Section 5).
 * mask[a] = 1 iff at least one fresh candidate (not timed-out, non-infinite rank)
 * has tau_cand in bucket a.
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

    /* Freshness check: only count neighbors that sent a DIO recently.
     * rl_time_last_dio == 0 means we have never received a DIO from this
     * neighbor via the RL hook → skip to be safe.                          */
    if(p->rl_time_last_dio == 0) { continue; }
    if((now - p->rl_time_last_dio) > RL_NEIGHBOR_TIMEOUT) { continue; }

    mask[get_bin(p->tau_cand)] = 1;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * Choose an action using ε-greedy policy (Section 6.2 Step 2).
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

  /* x in [0, 99] */
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
 * Select the physical parent from candidates in the chosen action range.
 * Tie-breaking: highest delta_rssi (Section 6.2 Step 3).
 */
static rpl_parent_t *
select_physical_parent(rpl_dag_t *dag, uint8_t chosen_action)
{
  rpl_parent_t *p;
  rpl_parent_t *best = NULL;
  clock_time_t now   = clock_time();

  for(p = nbr_table_head(rpl_parents); p != NULL;
      p = nbr_table_next(rpl_parents, p)) {

    if(p->dag != dag) { continue; }
    if(p->rank == INFINITE_RANK) { continue; }
    if(p->rl_time_last_dio == 0) { continue; }
    if((now - p->rl_time_last_dio) > RL_NEIGHBOR_TIMEOUT) { continue; }
    if(get_bin(p->tau_cand) != chosen_action) { continue; }

    if(best == NULL || p->rl_delta_rssi > best->rl_delta_rssi) {
      best = p;
    }
  }

  return best;
}
/*---------------------------------------------------------------------------*/
/**
 * Full RL trigger (Sections 6.2 & 7 of RL.md).
 *
 * Steps:
 *   1. Build ActiveAction mask.
 *   2. Epsilon-greedy action selection.
 *   3. Select physical parent (best delta_rssi in chosen action range).
 *   4. If new != current: update Q-table, decay epsilon, switch parent.
 *
 * The Q-table update uses the PREVIOUS state/action pair — reward is only
 * observable at the next decision point (TD-0 with delayed reward).
 */
rpl_parent_t *
rpl_rl_trigger(rpl_dag_t *dag)
{
  if(dag == NULL || dag->instance == NULL) {
    return NULL;
  }

  ensure_init();

  rpl_parent_t *current = dag->preferred_parent;

  /* Step 1: Build ActiveAction mask */
  uint8_t mask[RL_NUM_ACTIONS];
  build_active_mask(dag, mask);

  /* Determine current state S from current parent's tau_cand bucket */
  uint8_t current_state = 0;
  if(current != NULL) {
    current_state = current->rl_state_S;
  }

  /* Step 2: Choose action */
  int8_t chosen_action = choose_action(current_state, mask);
  if(chosen_action < 0) {
    printf("[RL] No active candidates in any range. Abort.\n");
    return current;
  }

  printf("[RL] State S=%u -> chose action A=%d (eps=%u/100)\n",
         current_state, chosen_action, epsilon);

  /* Step 3: Select physical parent in chosen action range */
  rpl_parent_t *new_parent = select_physical_parent(dag, (uint8_t)chosen_action);
  if(new_parent == NULL) {
    printf("[RL] No physical candidate in A=%d range. Abort.\n", chosen_action);
    return current;
  }

  /* Step 4: Update Q-table and switch parent if different */
  if(new_parent != current) {
    clock_time_t now = clock_time();

    /* Compute reward for the PREVIOUS decision (connection stability) */
    int16_t reward = 0;
    if(current != NULL && current->rl_time_attachment != 0) {
      clock_time_t conn_time = now - current->rl_time_attachment;
      reward = (int16_t)(conn_time / RL_STABILITY_CONSTANT);
      if(reward > RL_MAX_REWARD) { reward = RL_MAX_REWARD; }
    }

    /* Previous state S and action A (recovered from current parent record) */
    uint8_t prev_state  = (current != NULL) ? current->rl_state_S           : 0;
    uint8_t prev_action = (current != NULL) ? get_bin(current->rl_tau_at_choice) : 0;

    /* Q-table TD(0) update — values stored ×10 for 1 decimal precision:
     *   Q[S][A] += Alpha * (Reward - Q[S][A])
     * In integer form (Alpha = RL_ALPHA / 100, Reward stored ×10):
     *   delta = reward*10 - Q[S][A]
     *   Q[S][A] += (delta * RL_ALPHA) / 100                                 */
    int16_t old_q  = q_table[prev_state][prev_action];
    int32_t delta  = (int32_t)(reward * 10) - (int32_t)old_q;
    int16_t new_q  = (int16_t)(old_q + (int16_t)((delta * (int32_t)RL_ALPHA) / 100L));
    q_table[prev_state][prev_action] = new_q;

    printf("[RL] Q[%u][%u]: %d -> %d (reward=%d)\n",
           prev_state, prev_action, old_q, new_q, (int)reward);

    /* Decay epsilon (floor at RL_EPSILON_MIN) */
    uint16_t new_eps = ((uint16_t)epsilon * (uint16_t)RL_EPSILON_DECAY) / 100u;
    epsilon = (uint8_t)((new_eps < RL_EPSILON_MIN) ? RL_EPSILON_MIN : new_eps);

    /* Record attachment time and RL metadata on the new parent */
    new_parent->rl_time_attachment = now;
    new_parent->rl_state_S         = get_bin(new_parent->tau_cand);
    new_parent->rl_tau_at_choice   = new_parent->tau_cand;

    printf("[RL] Switching parent -> tau_cand=%u bin=%u, new eps=%u/100\n",
           new_parent->tau_cand, new_parent->rl_state_S, epsilon);

    /* Perform the actual RPL parent switch */
    rpl_set_preferred_parent(dag, new_parent);
    rpl_process_parent_event(dag->instance, new_parent);

  } else {
    printf("[RL] RL chose same parent (tau=%u). No switch.\n",
           (current != NULL) ? current->tau_cand : 0u);
  }

  return new_parent;
}
/*---------------------------------------------------------------------------*/
