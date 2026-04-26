# Implementation MRHOF Secrets Integration + Full RL Decision Flow

## Context

The codebase already contains a working RL-based RPL parent selection agent
(`rpl-rl-agent.c`) and a custom Objective Function (`rpl-of-tau.c`).
This prompt describes two orthogonal tasks:

1. **Absorb the four mechanisms** that make MRHOF robust into the existing
   `rpl-of-tau.c` and `rpl-rl-agent.c` layers.
2. **Replace the current `rpl_rl_trigger()` logic** with the new three-gate
   decision flow described in Section 2.

Do NOT rewrite the codebase from scratch. Apply surgical modifications
to the existing files only.

---

## Task 1 - Integrate the Four MRHOF Secrets

### Secret 1 - Aggressive hysteresis in `best_parent()` (already partially present, strengthen it)

**File:** `rpl-of-tau.c` → `best_parent()`

**Current state:** `RPL_OF_TAU_SWITCH_THRESHOLD = 75` is applied to TAU only.

**Required change:** Apply a **dual-axis hysteresis** — the candidate must beat
the current preferred parent on BOTH path-cost ETX AND TAU before a switch is
allowed. Specifically, mirror the MRHOF approach:

```c
/* A switch is only allowed if the candidate is better by more than
 * MRHOF_PC_THRESHOLD on path-cost AND by more than
 * RPL_OF_TAU_SWITCH_THRESHOLD on TAU.
 * Either condition failing alone keeps the current preferred parent. */
int pc_advantage  = (pc_cand + MRHOF_PC_THRESHOLD < pc_pref);
int tau_advantage = (t_cand  > t_pref + RPL_OF_TAU_SWITCH_THRESHOLD);

if(!pc_advantage && !tau_advantage) {
    return pref;   /* Neither metric is convincingly better — stay */
}
```

The existing `MRHOF_PC_THRESHOLD = 96` constant is already defined in the file.
Keep it. Only restructure the logic inside the hysteresis block.

---

### Secret 2 - Path cost, not link cost, as the comparison base (already present, verify)

**File:** `rpl-of-tau.c`

**Verify** that `parent_path_cost()` returns `parent_rank + ETX` and that
`best_parent()` compares `pc1` vs `pc2` (path costs), not raw ETX values.
This is already the case in the current implementation — confirm and leave
untouched if correct.

---

### Secret 3 - Use the smoothed ETX from `link-stats` EMA as a primary RL input

**File:** `rpl-rl-agent.c` → `rpl_rl_trigger()` and `select_physical_parent()`

**Problem:** The current tie-breaking criterion inside `select_physical_parent()`
uses `delta_rssi` (instantaneous, noisy). RSSI varies by ±5 dBm between two
consecutive measurements even on a static link.

**Required change:** Replace the sole use of `delta_rssi` as tie-breaker with
a **composite score** that combines the smoothed ETX with delta_rssi:

```c
/* Composite tie-break score (higher = better candidate).
 * ETX is already a smoothed EMA from link-stats — use it as primary signal.
 * delta_rssi is instantaneous — use it only as a secondary tiebreak.
 * Both terms are normalized to [0, 1000] before combining.             */
static uint16_t
candidate_score(rpl_parent_t *p)
{
    uint16_t etx_norm  = rpl_etx_norm(p);          /* already in [0,1000] */
    int16_t  d_rssi    = p->rl_delta_rssi;          /* signed, clamp to safe range */
    int16_t  rssi_contrib = (int16_t)((d_rssi + 20) * 10); /* map [-20,+20] → [0,400] */
    if(rssi_contrib < 0)   rssi_contrib = 0;
    if(rssi_contrib > 400) rssi_contrib = 400;

    /* ETX contributes 70%, delta_rssi trend contributes 30% */
    return (uint16_t)((etx_norm * 7 + (uint16_t)rssi_contrib * 3) / 10);
}
```

Use `candidate_score()` as the comparison key inside `select_physical_parent()`
instead of raw `delta_rssi`.

---

### Secret 4 - Superlinear ETX penalty inside `Tau_cand` computation

**File:** `rpl-of-tau.c` → `calculate_candidate_score()` and the underlying
`rpl_tau_compute_cand()` function (wherever it is defined).

**Required change:** When computing `ETX_norm` to feed into `Tau_cand`,
apply a squaring penalty before normalizing, identically to
`RPL_MRHOF_SQUARED_ETX`:

```c
/* Apply squared-ETX penalty before normalization.
 * ETX=1 (perfect) → cost=1, ETX=2 → cost=4, ETX=3 → cost=9.
 * This strongly penalizes marginal links and keeps the node on reliable parents. */
uint32_t etx_raw = (uint32_t)stats->etx;
uint32_t etx_sq  = (etx_raw * etx_raw) / LINK_STATS_ETX_DIVISOR;
uint16_t etx_penalized = (uint16_t)MIN(etx_sq, 0xFFFF);
```

Then use `etx_penalized` in place of the raw `stats->etx` inside
`rpl_etx_norm()` or `calculate_candidate_score()`.

Guard it behind a compile-time flag for easy A/B comparison:

```c
#ifndef RPL_OF_TAU_SQUARED_ETX
#define RPL_OF_TAU_SQUARED_ETX 1   /* enabled by default */
#endif
```

---

## Task 2 - Replace `rpl_rl_trigger()` with the Three-Gate Decision Flow

**File:** `rpl-icmp6.c` (DIO reception hook) + `rpl-rl-agent.c`

The full decision flow must be implemented as described below.
The entry point called from the DIO reception hook in `rpl-icmp6.c` is:

```c
void rpl_rl_on_dio_received(rpl_dag_t *dag, rpl_parent_t *v, int16_t measured_rssi);
```

---

### Step 0 - Always update the candidate table (no condition)

```c
/* Executed unconditionally on every DIO, for every neighbor V */
v->rl_delta_rssi   = measured_rssi - v->rl_last_rssi;
v->rl_last_rssi    = measured_rssi;
v->tau_cand        = calculate_candidate_score(v);   /* rpl-of-tau formula */
v->rl_time_last_dio = clock_time();
```

---

### Gate 1 - Is V the current parent?

#### CASE A: V is the current parent

```c
uint8_t new_state = get_bin(v->tau_cand);

if(new_state >= current->rl_state_S) {
    /* Link is stable or improving — update state, do nothing else */
    current->rl_state_S = new_state;
    return;
}
/* else: link degraded → fall through to RL agent */
```

#### CASE B: V is a different neighbor

```c
uint16_t current_etx  = rpl_get_parent_link_metric(current);
int16_t  current_rssi = current->rl_last_rssi;

if(current_rssi > RL_RSSI_WEAK_THRESHOLD && current_etx < RL_ETX_WEAK_THRESHOLD) {
    /* Current parent is healthy — no need to run RL */
    return;
}
/* else: current parent shows weakness → fall through to RL agent */
```

**Constants to define:**

| Constant | Meaning | Suggested value |
|---|---|---|
| `RL_RSSI_WEAK_THRESHOLD` | RSSI below which current parent is considered weak | `-85` dBm |
| `RL_ETX_WEAK_THRESHOLD` | ETX above which current parent is considered weak | `3 * LINK_STATS_ETX_DIVISOR` |

---

### RL Agent Block (reached only if a gate above fell through)

**Step 1:** Build the `ActiveAction` mask (existing `build_active_mask()` — unchanged).

**Step 2:** Re-evaluate the current state from the current parent's fresh `tau_cand`:

```c
uint8_t new_state  = get_bin(current->tau_cand);
uint8_t A_chosen   = choose_action(new_state, mask);    /* existing function */
uint8_t A_current  = get_bin(current->tau_cand);
```

**Step 3:** Always update Q-table and decay epsilon (existing logic — unchanged).
This must execute regardless of whether a switch will happen.

**Step 4 - Gate 2 (Delta_Q): is switching Q-worthy?**

```c
int16_t Q_switch = q_table[new_state][A_chosen];
int16_t Q_stay   = q_table[new_state][A_current];
int16_t gain     = Q_switch - Q_stay;

if(gain < RL_MIN_SWITCH_GAIN) {
    /* Q-table does not justify a switch from this state */
    current->rl_state_S = new_state;   /* acknowledge degradation */
    return;
}
```

**Constant to define:**

| Constant | Meaning | Suggested starting value |
|---|---|---|
| `RL_MIN_SWITCH_GAIN` | Minimum Q-gain (×10, stored format) to justify a switch | `5` (= 0.5 in real units) |

> **Note on early Q-table:** When the Q-table is near zero (early deployment),
> `gain` will also be near zero and Gate 2 will almost always pass.
> This is the correct behavior — early on, the agent should switch freely
> to gather experience. Gate 2 only becomes a real filter once the Q-table
> has been trained.

**Step 5:** Select the physical parent (existing `select_physical_parent()` —
but now using the composite `candidate_score()` from Secret 3 as tie-breaker):

```c
rpl_parent_t *new_parent = select_physical_parent(dag, A_chosen);

if(new_parent == NULL || new_parent == current) {
    return current;   /* abort */
}
```

---

### Gate 3 - Physical hysteresis: is the switch worth it?

```c
int16_t  delta_tau  = (int16_t)new_parent->tau_cand    - (int16_t)current->tau_cand;
int16_t  delta_rssi = (int16_t)new_parent->rl_last_rssi - (int16_t)current->rl_last_rssi;
uint16_t current_etx     = rpl_get_parent_link_metric(current);
uint16_t new_parent_etx  = rpl_get_parent_link_metric(new_parent);
int16_t  delta_etx  = (int16_t)current_etx - (int16_t)new_parent_etx;

int current_still_acceptable =
    (current->rl_last_rssi > RL_RSSI_WEAK_THRESHOLD) &&
    (current_etx           < RL_ETX_WEAK_THRESHOLD);

int candidate_clearly_better =
    (delta_tau  > RL_HYSTERESIS_TAU)  &&
    (delta_rssi > RL_HYSTERESIS_RSSI) &&
    (delta_etx  > RL_HYSTERESIS_ETX);

if(current_still_acceptable && !candidate_clearly_better) {
    /* Q-table was updated, learning happened, but physical evidence disagrees */
    current->rl_state_S = new_state;
    return;
}
```

**Constants to define:**

| Constant | Meaning | Suggested value |
|---|---|---|
| `RL_HYSTERESIS_TAU` | Min TAU advantage for candidate to justify switch | `75` (same as `RPL_OF_TAU_SWITCH_THRESHOLD`) |
| `RL_HYSTERESIS_RSSI` | Min RSSI advantage in dBm | `5` |
| `RL_HYSTERESIS_ETX` | Min ETX improvement (in ETX units × `LINK_STATS_ETX_DIVISOR`) | `0.5 * LINK_STATS_ETX_DIVISOR` |

---

### Commit the switch (reached only if Gate 3 passed)

```c
new_parent->rl_time_attachment = clock_time();
new_parent->rl_state_S         = get_bin(new_parent->tau_cand);
new_parent->rl_tau_at_choice   = new_parent->tau_cand;

rpl_set_preferred_parent(dag, new_parent);
rpl_process_parent_event(dag->instance, new_parent);
```

---

## Summary of All New Constants

All constants below must be added to `rpl-rl-agent.h` with the `#ifndef` guard
pattern so they can be overridden per-platform in `project-conf.h`.

| Constant | File | Suggested default |
|---|---|---|
| `RL_RSSI_WEAK_THRESHOLD` | `rpl-rl-agent.h` | `-85` |
| `RL_ETX_WEAK_THRESHOLD` | `rpl-rl-agent.h` | `3 * LINK_STATS_ETX_DIVISOR` |
| `RL_MIN_SWITCH_GAIN` | `rpl-rl-agent.h` | `5` |
| `RL_HYSTERESIS_TAU` | `rpl-rl-agent.h` | `75` |
| `RL_HYSTERESIS_RSSI` | `rpl-rl-agent.h` | `5` |
| `RL_HYSTERESIS_ETX` | `rpl-rl-agent.h` | `64` (= 0.5 × 128) |
| `RPL_OF_TAU_SQUARED_ETX` | `rpl-of-tau.c` | `1` |

---

## Implementation Constraints

1. Do NOT modify `rpl-mrhof.c` — it is the reference baseline and must remain
   untouched for comparison experiments.
2. Do NOT change the `Tau_cand` formula itself — only change how `ETX_norm` is
   computed before being fed into it (Secret 4).
3. The Q-table update (reward + epsilon decay) must ALWAYS execute when the RL
   block is entered, regardless of Gate 2 or Gate 3 outcome — the learning
   cycle must never be skipped.
4. The new entry point `rpl_rl_on_dio_received()` replaces the current direct
   call to `rpl_rl_trigger()` from the DIO hook. Remove the old call site.
5. `best_parent()` in `rpl-of-tau.c` must remain fully functional — it is
   called independently by the RPL core engine and must not be removed or
   stubbed out.