# Implementation Prompt — RL Learning Phase (Off-Policy Pre-Training, Time-Based)

## Context

The RL agent starts with a Q-table of all zeros, which means early decisions
are uninformed and poison the table with bad rewards. The fix is a bounded
learning phase where parent selection uses the proven deterministic rule
(best `tau_cand` wins), while the Q-table is silently updated by observing
every switch. When the learning phase ends, the Q-table is already populated
with real, meaningful experience and the RL agent takes over immediately.

This pattern is called off-policy pre-training: the **behavior policy** during
learning is deterministic (OF-TAU logic), the **learning target** is the
Q-table that will later drive the RL policy.

The learning phase is bounded by a **wall-clock duration**, not a switch count,
so the transition point is predictable and topology-independent.

Do NOT modify `rpl-of-tau.c` or `rpl-mrhof.c`. All changes are confined to
`rpl-rl-agent.c` and `rpl-rl-agent.h`.

---

## 1. Mode Tracking

```c
/* Timestamp recorded on the very first call to ensure_init().
 * The learning phase runs from this point for RL_LEARNING_DURATION.
 * Never reset after initialization.                                    */
static clock_time_t rl_learning_start = 0;

/* Switch counter — kept for logging only, not used as mode gate.       */
static uint16_t rl_switch_count = 0;

/* True while the agent is still in learning mode.                      */
#define RL_IS_LEARNING() \
    (clock_time() - rl_learning_start < RL_LEARNING_DURATION)
```

Initialize `rl_learning_start` inside `ensure_init()`, which is already called
at the top of `rpl_rl_trigger()`:

```c
static void
ensure_init(void)
{
    static uint8_t initialized = 0;
    if(initialized) { return; }

    memset(q_table, 0, sizeof(q_table));
    epsilon           = RL_EPSILON_INITIAL;
    rl_learning_start = clock_time();   /* start the learning clock */
    initialized       = 1;

    printf("[RL] Initialized. Learning phase duration: %lu s.\n",
           (unsigned long)RL_LEARNING_DURATION_S);
}
```

---

## 2. Learning Mode — Parent Selection

During learning, completely bypass `choose_action()` and epsilon-greedy.
Instead, select the candidate with the highest `tau_cand` among all fresh,
valid neighbors. This is the same criterion used by `best_parent()` in
`rpl-of-tau.c`.

Add this function to `rpl-rl-agent.c`:

```c
/*
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

        if(p->dag != dag)                                      { continue; }
        if(p->rank == INFINITE_RANK)                           { continue; }
        if(p->tau_cand == 0)                                   { continue; }
        if(p->rl_time_last_dio == 0)                           { continue; }
        if((now - p->rl_time_last_dio) > RL_NEIGHBOR_TIMEOUT) { continue; }

        if(p->tau_cand > best_tau) {
            best_tau = p->tau_cand;
            best     = p;
        }
    }
    return best;
}
```

---

## 3. Q-Table Update (Shared Between Both Modes)

Factor the Q-table update into a standalone function called identically in
both learning and production mode. The reward is always based on connection
stability duration:

```c
/*
 * Records the reward for the PREVIOUS parent decision and updates the
 * Q-table entry for that decision.
 *
 * Must be called at the start of every switch, before the new parent
 * is committed, so that conn_time reflects the full previous cycle.
 *
 * prev_state  = rl_state_S  recorded on the outgoing parent at attachment
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
```

---

## 4. Revised `rpl_rl_trigger()` Structure

Replace the body of `rpl_rl_trigger()` with the following two-branch
structure. Both branches call `update_q_table()` before committing any switch.

```c
rpl_parent_t *
rpl_rl_trigger(rpl_dag_t *dag)
{
    if(dag == NULL || dag->instance == NULL) { return NULL; }
    ensure_init();

    rpl_parent_t *current = dag->preferred_parent;

    /* ------------------------------------------------------------------ */
    if(RL_IS_LEARNING()) {
    /* ================================================================== */
    /*  LEARNING MODE                                                       */
    /*  Select parent by best tau_cand (deterministic, proven OF-TAU rule) */
    /*  Update Q-table silently by observing the outcome.                  */
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
        rpl_process_parent_event(dag->instance, new_parent);
        return new_parent;

    } else {
    /* ================================================================== */
    /*  PRODUCTION MODE                                                     */
    /*  Full RL decision flow: three-gate policy.                          */
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

        /* Step 2: Re-evaluate current state from fresh tau_cand */
        uint8_t new_state = (current != NULL) ? get_bin(current->tau_cand) : 0;
        uint8_t A_current = new_state;

        /* Step 3: Choose action (epsilon-greedy) */
        int8_t chosen_action = choose_action(new_state, mask);
        if(chosen_action < 0) {
            printf("[RL:PROD] No active candidates. Abort.\n");
            return current;
        }

        /* Step 4: Always update Q-table (even if no switch happens) */
        update_q_table(current);

        /* Decay epsilon */
        uint16_t new_eps = ((uint16_t)epsilon * (uint16_t)RL_EPSILON_DECAY) / 100u;
        epsilon = (uint8_t)((new_eps < RL_EPSILON_MIN) ? RL_EPSILON_MIN : new_eps);

        /* Gate 2 — Delta_Q: is switching Q-worthy from this degraded state? */
        int16_t Q_switch = q_table[new_state][(uint8_t)chosen_action];
        int16_t Q_stay   = q_table[new_state][A_current];
        int16_t gain     = Q_switch - Q_stay;

        if(gain < RL_MIN_SWITCH_GAIN) {
            if(current != NULL) { current->rl_state_S = new_state; }
            printf("[RL:PROD] Gate 2 blocked. gain=%d < %d\n",
                   gain, RL_MIN_SWITCH_GAIN);
            return current;
        }

        /* Step 5: Select physical parent in chosen action range */
        rpl_parent_t *new_parent = select_physical_parent(dag, (uint8_t)chosen_action);
        if(new_parent == NULL || new_parent == current) {
            return current;
        }

        /* Gate 3 — Physical hysteresis */
        int16_t  delta_tau  = (int16_t)new_parent->tau_cand
                            - (int16_t)current->tau_cand;
        int16_t  delta_rssi = (int16_t)new_parent->rl_last_rssi
                            - (int16_t)current->rl_last_rssi;
        uint16_t cur_etx    = rpl_get_parent_link_metric(current);
        uint16_t new_etx    = rpl_get_parent_link_metric(new_parent);
        int16_t  delta_etx  = (int16_t)cur_etx - (int16_t)new_etx;

        int current_still_acceptable =
            (current->rl_last_rssi > RL_RSSI_WEAK_THRESHOLD) &&
            (cur_etx               < RL_ETX_WEAK_THRESHOLD);

        int candidate_clearly_better =
            (delta_tau  > RL_HYSTERESIS_TAU)  &&
            (delta_rssi > RL_HYSTERESIS_RSSI) &&
            (delta_etx  > RL_HYSTERESIS_ETX);

        if(current_still_acceptable && !candidate_clearly_better) {
            current->rl_state_S = new_state;
            printf("[RL:PROD] Gate 3 blocked. Current acceptable, "
                   "candidate not convincingly better.\n");
            return current;
        }

        /* All gates passed: commit the switch */
        rl_switch_count++;

        printf("[RL:PROD] Switch #%u -> tau=%u bin=%u eps=%u/100\n",
               rl_switch_count,
               new_parent->tau_cand,
               get_bin(new_parent->tau_cand),
               epsilon);

        new_parent->rl_time_attachment = clock_time();
        new_parent->rl_state_S         = get_bin(new_parent->tau_cand);
        new_parent->rl_tau_at_choice   = new_parent->tau_cand;

        rpl_set_preferred_parent(dag, new_parent);
        rpl_process_parent_event(dag->instance, new_parent);
        return new_parent;
    }
}
```

---

## 5. Trigger Policy (Identical in Both Modes)

The DIO reception hook applies the same two-tier trigger policy in both
learning and production mode. There is no reason to trigger more often during
learning — the deterministic selection is conservative enough on its own.

```c
void
rpl_rl_on_dio_received(rpl_dag_t *dag, rpl_parent_t *v, int16_t measured_rssi)
{
    rpl_parent_t *current = dag->preferred_parent;

    /* Step 0: Always update candidate table — unconditional, every DIO */
    v->rl_delta_rssi    = measured_rssi - v->rl_last_rssi;
    v->rl_last_rssi     = measured_rssi;
    v->tau_cand         = calculate_candidate_score(v);
    v->rl_time_last_dio = clock_time();

    if(current == NULL) {
        /* No parent yet: run immediately to find one */
        rpl_rl_trigger(dag);
        return;
    }

    if(v == current) {
        /* ------------------------------------------------------------ */
        /* CASE A: DIO from current parent                               */
        /* ------------------------------------------------------------ */
        uint8_t new_state = get_bin(v->tau_cand);

        if(new_state >= current->rl_state_S) {
            /* Link stable or improving: update state only, no RL */
            current->rl_state_S = new_state;
            return;
        }

        /* Link degraded: trigger RL */
        rpl_rl_trigger(dag);

    } else {
        /* ------------------------------------------------------------ */
        /* CASE B: DIO from a different neighbor                         */
        /* ------------------------------------------------------------ */
        uint16_t current_etx  = rpl_get_parent_link_metric(current);
        int16_t  current_rssi = current->rl_last_rssi;

        int current_is_weak =
            (current_rssi < RL_RSSI_WEAK_THRESHOLD) ||
            (current_etx  > RL_ETX_WEAK_THRESHOLD);

        if(current_is_weak) {
            rpl_rl_trigger(dag);
        }
        /* else: candidate table already updated above, nothing more */
    }
}
```

---

## 6. Constants

Add all of the following to `rpl-rl-agent.h` with `#ifndef` guards so they
can be overridden per-platform in `project-conf.h`.

```c
/* ------------------------------------------------------------------ */
/* Learning phase                                                       */
/* ------------------------------------------------------------------ */

/* Duration of the learning phase in seconds.
 * After this time has elapsed since first boot the agent enters
 * production mode automatically.
 * Rule of thumb: cover at least 3 full trickle timer cycles at
 * DIO_INTERVAL_MAX so every neighbor has sent several DIOs and
 * the candidate table is fully populated.                             */
#ifndef RL_LEARNING_DURATION_S
#define RL_LEARNING_DURATION_S      300u      /* 5 minutes */
#endif
#define RL_LEARNING_DURATION \
    ((clock_time_t)(RL_LEARNING_DURATION_S) * CLOCK_SECOND)

/* ------------------------------------------------------------------ */
/* Q-Learning core                                                      */
/* ------------------------------------------------------------------ */

#ifndef RL_ALPHA
#define RL_ALPHA                    10        /* learning rate ×100 → 0.10 */
#endif

#ifndef RL_MAX_REWARD
#define RL_MAX_REWARD               100       /* stored ×10 → 10.0 */
#endif

#ifndef RL_STABILITY_CONSTANT
#define RL_STABILITY_CONSTANT       (30 * CLOCK_SECOND)
#endif

/* ------------------------------------------------------------------ */
/* Epsilon-greedy (production mode only)                               */
/* ------------------------------------------------------------------ */

#ifndef RL_EPSILON_INITIAL
#define RL_EPSILON_INITIAL          90        /* 90% exploration at start */
#endif

#ifndef RL_EPSILON_DECAY
#define RL_EPSILON_DECAY            95        /* ×0.95 per production switch */
#endif

#ifndef RL_EPSILON_MIN
#define RL_EPSILON_MIN              10        /* 10% floor, never goes below */
#endif

/* ------------------------------------------------------------------ */
/* Gate 2 — Delta_Q                                                    */
/* ------------------------------------------------------------------ */

#ifndef RL_MIN_SWITCH_GAIN
#define RL_MIN_SWITCH_GAIN          5         /* stored ×10 → 0.5 real units */
#endif

/* ------------------------------------------------------------------ */
/* Gate 3 — Physical hysteresis                                        */
/* ------------------------------------------------------------------ */

#ifndef RL_HYSTERESIS_TAU
#define RL_HYSTERESIS_TAU           75        /* min TAU advantage for switch */
#endif

#ifndef RL_HYSTERESIS_RSSI
#define RL_HYSTERESIS_RSSI          5         /* min RSSI advantage in dBm    */
#endif

#ifndef RL_HYSTERESIS_ETX
#define RL_HYSTERESIS_ETX           64        /* 0.5 × LINK_STATS_ETX_DIVISOR */
#endif

/* ------------------------------------------------------------------ */
/* Weakness detection (Case B trigger + Gate 3)                        */
/* ------------------------------------------------------------------ */

#ifndef RL_RSSI_WEAK_THRESHOLD
#define RL_RSSI_WEAK_THRESHOLD      (-85)     /* dBm */
#endif

#ifndef RL_ETX_WEAK_THRESHOLD
#define RL_ETX_WEAK_THRESHOLD       (3 * LINK_STATS_ETX_DIVISOR)
#endif

/* ------------------------------------------------------------------ */
/* Neighbor freshness                                                   */
/* ------------------------------------------------------------------ */

#ifndef RL_NEIGHBOR_TIMEOUT
#define RL_NEIGHBOR_TIMEOUT         (3 * RPL_DIO_INTERVAL_MIN * CLOCK_SECOND)
#endif
```

### Tuning guidance for `RL_LEARNING_DURATION_S`

| Simulation length | Suggested learning duration |
|---|---|
| Short (< 10 min) | 60–90 s |
| Standard (30 min) | 300 s (5 min) |
| Long (> 1 hour) | 600 s (10 min) |

The learning window must be long enough for every frequently visited
state-action pair to receive **at least one reward update** before the RL
agent takes over. If `DIO_INTERVAL_MAX` is large (e.g. 4096 ms), increase
the learning duration accordingly.

---

## 7. Behavior Summary

| Behavior | Learning mode | Production mode |
|---|---|---|
| Mode gate | `clock_time() - rl_learning_start < RL_LEARNING_DURATION` | same condition, inverted |
| Parent selection | Best `tau_cand` — deterministic | Epsilon-greedy via Q-table |
| Epsilon | Unused | Decays per switch, floor at `RL_EPSILON_MIN` |
| Gate 2 (Delta_Q) | Not applicable | Active |
| Gate 3 (Hysteresis) | Standard thresholds | Standard thresholds |
| Case B trigger | Only if current parent weak | Only if current parent weak |
| Q-table update | Always on every switch | Always when RL block is entered |
| Absolute TAU floor | Not enforced (deterministic selection handles it naturally) | Enforced via Gate 3 |

---

## 8. Invariants That Must Not Change

1. **Step 0 (candidate table update) is unconditional** — executes on every
   DIO from every neighbor, with no gate or condition.

2. **`update_q_table()` is called on every switch in both modes** — the reward
   for the previous cycle is always recorded before the new parent is committed.

3. **`rl_time_attachment`, `rl_state_S`, and `rl_tau_at_choice` are written
   on the new parent immediately after every switch** — these three fields are
   what `update_q_table()` reads on the next switch. If any one of them is
   missing the reward computation is wrong.

4. **`rl_learning_start` and `rl_switch_count` are never reset** — the
   learning phase is a one-time bootstrap, not a recurring cycle.

5. **`best_parent()` in `rpl-of-tau.c` is not modified** — it is called
   independently by the RPL core engine and must remain fully functional.

6. **`rpl-mrhof.c` is not modified** — it remains the untouched reference
   baseline for comparison experiments.