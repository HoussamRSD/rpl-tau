# Implementation Of RL-Based RPL Parent Selection (Q-Learning)

---

## 1. Context & Objective

Implement a **Q-Learning-based parent selection mechanism** for an RPL (Routing Protocol for Low-Power and Lossy Networks) node.
work  on this version : Version2-OnlyTauDIO
The goal is to replace the static parent selection logic with a reinforcement learning agent that learns, over time, which category of parent (in terms of link quality metric `Tau_cand`) leads to the most stable connections.
Note that the Version2-OnlyTauDIO already gives a very good PDR 98% so the goal is to improve the PDR not reduce it.
---

## 2. Key Metric: `Tau_cand`

`Tau_cand` is a link quality metric computed from each received DIO message.  
Its value belongs to the range **[0, 1000]**.  
Refer to the existing codebase for its exact computation formula — **do not redefine it**.

---

## 3. Q-Learning Setup

### 3.1 State Space (10 states)

The state represents the **current parent's `Tau_cand`**, discretized into 10 equal ranges:

| State | Range |
|-------|-------|
| S0 | [0, 100[ |
| S1 | [100, 200[ |
| S2 | [200, 300[ |
| ... | ... |
| S9 | [900, 1000] |

> The state is always derived from the **current parent's** `Tau_cand`, not a candidate's.

### 3.2 Action Space (10 actions)

The action represents the **target `Tau_cand` range** to select the new parent from:

| Action | Range |
|--------|-------|
| A0 | [0, 100[ |
| A1 | [100, 200[ |
| ... | ... |
| A9 | [900, 1000] |

> Choosing action `Ak` means: select the next parent from candidates whose `Tau_cand` falls within `Ak`'s range.

### 3.3 Q-Table

- Dimensions: **10 × 10** (`State × Action`)
- Initialized to zero.
- Update rule (TD(0)):

```
Q[S][A] += Alpha * (Reward - Q[S][A])
```

Where `Alpha` is the learning rate (suggested: `0.1`).

### 3.4 Reward Function

The reward is based on the **stability duration** of the connection with the previous parent:

```
Reward = MIN(Connection_Time / STABILITY_CONSTANT, MAX_REWARD_VALUE)
```

- `Connection_Time` = `current_time - parent.time_attachment`
- `STABILITY_CONSTANT` and `MAX_REWARD_VALUE` are tunable constants.
- **Capping the reward is mandatory** to avoid mathematical divergence for very long connections.

### 3.5 Exploration Policy (ε-Greedy)

- At the start, `ε` is high (e.g., `0.9`) to encourage exploration.
- `ε` decreases over time but **must never fall below a minimum floor** (e.g., `ε_min = 0.10`).
- Keeping a minimum `ε` is critical to allow the node to re-adapt if the network topology changes drastically.

```
ε = MAX(ε * decay_factor, ε_min)
```

---

## 4. Data Structures

### 4.1 Candidate Parent Table Entry

Each known neighbor must store the following fields:

| Field | Description |
|---|---|
| `node_id` | Neighbor identifier |
| `tau_cand` | Last computed `Tau_cand` for this neighbor |
| `last_rssi` | RSSI value at the time of the last received DIO |
| `time_last_dio` | Timestamp of the last received DIO |
| `delta_rssi` | Computed as: `current_DIO_RSSI - last_rssi` |

> **Important:** The RSSI value is **measured locally** upon DIO reception. It is NOT a field inside the DIO message.

### 4.2 Current Parent Record

In addition to the candidate table entry fields, the current parent must also store:

| Field | Description |
|---|---|
| `time_attachment` | Timestamp when this node became the current parent |
| `state_S` | State `S` computed at the time of the last RL decision |
| `tau_cand_at_last_choice` | `Tau_cand` value of this parent at the time of the last RL decision |

---

## 5. Active Action Mask

Before every decision, compute the **`ActiveAction[10]`** binary mask:

```
For each candidate C in Candidate_Table:
    if (current_time - C.time_last_dio < NEIGHBOR_TIMEOUT):
        range_index = floor(C.tau_cand / 100)
        ActiveAction[range_index] = 1
```

Only actions where `ActiveAction[A] == 1` are eligible for selection.

> If an action has no active candidates, its Q-value column is **frozen (not reset)** — it simply cannot be selected until a candidate re-appears in that range.




--

## 6. Main Algorithm

### 6.1 On DIO Reception: `on_DIO_received(neighbor V, DIO message)`

**Step 1 — Update Candidate Table for V:**
```
V.delta_rssi       = measured_RSSI - V.last_rssi
V.last_rssi        = measured_RSSI
V.tau_cand         = compute_tau_cand(DIO)   // use existing formula
V.time_last_dio    = current_time()
```

**Step 2 — Determine trigger context:**

#### CASE 1: DIO received from the current parent
```
new_state_S = floor(V.tau_cand / 100)

if (new_state_S == current_parent.state_S):
    return  // No state change → do nothing

else:
    current_parent.state_S = new_state_S
    trigger_RL_process()
```

#### CASE 2: DIO received from any other neighbor
```
// Only update the candidate table (already done in Step 1).
// Do NOT trigger the RL process here.
return
```

---

### 6.2 Procedure: `trigger_RL_process()`

**Step 1 — Build the Active Action Mask**
```
Compute ActiveAction[10] as described in Section 5.
```

**Step 2 — Choose an Action (ε-Greedy)**
```
x = random(0, 100)

if (x < ε * 100):
    chosen_action = random choice among A where ActiveAction[A] == 1

else:
    chosen_action = argmax over A (where ActiveAction[A] == 1) of Q[current_parent.state_S][A]
```

**Step 3 — Select the Physical Parent (Tie-Breaking)**
```
filtered_list = all candidates C where:
    floor(C.tau_cand / 100) == chosen_action
    AND (current_time - C.time_last_dio < NEIGHBOR_TIMEOUT)

new_parent = candidate in filtered_list with the highest delta_rssi
```

**Step 4 — Update Q-Table and Switch Parent**
```
if (new_parent != current_parent):

    // Compute reward for the PREVIOUS decision
    connection_time = current_time() - current_parent.time_attachment
    reward = MIN(connection_time / STABILITY_CONSTANT, MAX_REWARD_VALUE)

    previous_state  = current_parent.state_S
    previous_action = floor(current_parent.tau_cand_at_last_choice / 100)

    Q[previous_state][previous_action] += Alpha * (reward - Q[previous_state][previous_action])

    // Decay epsilon
    ε = MAX(ε * decay_factor, ε_min)

    // Switch parent
    current_parent = new_parent
    current_parent.time_attachment          = current_time()
    current_parent.state_S                  = floor(new_parent.tau_cand / 100)
    current_parent.tau_cand_at_last_choice  = new_parent.tau_cand
```

---

## 7. Panic Monitor (Periodic Timer)

A periodic background timer must verify the quality of the current parent link.

**Do NOT trigger the full RL process on every timer tick.**  
Reuse the existing monitoring mechanism in the codebase and adapt it as follows:

```
on_panic_timer_tick():

    measure current_parent RSSI

    if (current_parent RSSI > RSSI_THRESHOLD AND current_parent ETX < ETX_THRESHOLD):
        return  // Link is healthy, do nothing

    else:
        // Link is degraded → force a parent re-selection
        trigger_RL_process()
```

> This avoids unnecessarily re-running the RL agent on every tick while still guaranteeing a fast reaction to link failures.

---

## 8. Neighbor Failure Detection

A neighbor is considered **failed/unreachable** if:

```
current_time() - C.time_last_dio > NEIGHBOR_TIMEOUT
```

Such neighbors must be excluded from the `ActiveAction` mask computation (see Section 5) and from candidate selection (Step 3 of `trigger_RL_process`).

---

## 9. Summary of Tunable Constants

| Constant | Description | Suggested Default |
|---|---|---|
| `Alpha` | Q-Learning rate | `0.1` |
| `ε_initial` | Initial exploration rate | `0.9` |
| `ε_min` | Minimum exploration floor | `0.10` |
| `decay_factor` | ε decay per update | `0.95` |
| `STABILITY_CONSTANT` | Reward normalization divisor (seconds) | Tune to target stable duration |
| `MAX_REWARD_VALUE` | Maximum clipped reward | e.g., `10.0` |
| `NEIGHBOR_TIMEOUT` | Max time without DIO before neighbor is considered dead | e.g., `3 × DIO_interval` |
| `RSSI_THRESHOLD` | Minimum acceptable RSSI for current parent | Platform-dependent |
| `ETX_THRESHOLD` | Maximum acceptable ETX for current parent | Platform-dependent |

---

## 10. Implementation Notes

1. **Do not modify the `Tau_cand` computation formula** — use the existing implementation.
2. **RSSI is measured locally** on DIO arrival, never read from inside the DIO message payload.
3. The Q-Table update is always applied to the **previous state/action pair**, not the current one — the reward is only observable after the fact (at the next decision point).
4. The `ActiveAction` mask **freezes, never resets** Q-values for inactive columns — this preserves learned knowledge for ranges that temporarily have no candidates.
5. The RL process must be triggered **only** from CASE 1 (current parent state change) or the Panic Monitor — never from CASE 2.