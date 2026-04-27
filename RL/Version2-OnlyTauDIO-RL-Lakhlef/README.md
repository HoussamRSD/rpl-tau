# Version2-OnlyTauDIO-RL-Lakhlef

RPL with OF-TAU + Q-Learning parent selection agent.
Sends only Tau in DIO messages. Best baseline result: 98% PDR (20-node SmartCity).

---

## Change Log

### 1. RL Core Implementation — `ac6b2ad`
**Commit:** `ac6b2ad` — *feat: implement RL-based RPL routing logic and add simulation evaluation logs*

- Q-Learning agent (`rpl-rl-agent.c`, `rpl-rl-agent.h`) with 10×10 Q-table (state = current parent tau bin, action = target tau bin)
- Three-gate decision flow: Gate 1 (trigger policy), Gate 2 (Delta_Q), Gate 3 (physical hysteresis)
- MRHOF Secrets integrated into OF-TAU:
  - Secret 1: Dual-axis hysteresis (path-cost ETX AND TAU) in `best_parent()`
  - Secret 2: Path cost comparison (rank + ETX), not raw link ETX
  - Secret 3: Composite tie-break score (70% smoothed ETX + 30% delta_rssi) in `candidate_score()`
  - Secret 4: Squared ETX penalty with `RPL_OF_TAU_SQUARED_ETX` compile flag
- Time-based learning phase (off-policy pre-training, `RL_LEARNING_DURATION_S = 300s`)
- `rpl_rl_on_dio_received()` entry point called from DIO hook in `rpl-dag.c`
- All constants overridable via `project-conf.h` with `#ifndef` guards

**Files modified:**
- `rpl/rpl-rl-agent.c` — Full RL agent (two-branch: learning + production)
- `rpl/rpl-rl-agent.h` — Constants and public API
- `rpl/rpl-of-tau.c` — MRHOF secrets, panic monitor, squared ETX
- `rpl/rpl-dag.c` — DIO hook calls `rpl_rl_on_dio_received()`, added `#include "rpl-rl-agent.h"`
- `rpl/rpl.h` — RL fields in `rpl_parent_t` struct

---

### 2. Fix NPC Explosion (Dual Parent Selection Conflict) — `2e5f42e`
**Commit:** `2e5f42e` — *fix: eliminate dual parent selection conflict (NPC 1310->expected ~80)*

**Problem:** NPC exploded from 81 → 1310 (16×) due to two independent parent selection engines fighting each other:
1. The RL agent selects parent A via `rpl_rl_on_dio_received()`
2. Then `rpl_process_parent_event()` (called by RL agent internally AND by RPL core at L2028) triggers `rpl_select_dag()` → `best_parent()` which overrides the RL choice, causing ping-pong.

**Fix:** Replace `rpl_process_parent_event()` with simple rank updates (`dag->rank = rpl_rank_via_parent()`) in 3 locations, making the RL agent the sole decision-maker:

| File | Location | Change |
|---|---|---|
| `rpl-dag.c` | L2028 | `rpl_process_parent_event(instance, p)` → `dag->rank = rpl_rank_via_parent(dag->preferred_parent)` |
| `rpl-rl-agent.c` | L345 (learning commit) | `rpl_process_parent_event(dag->instance, new_parent)` → `dag->rank = rpl_rank_via_parent(new_parent)` |
| `rpl-rl-agent.c` | L447 (production commit) | Same replacement |

---

### 3. Fix Background Timer NPC Inflation (Periodic Overrides) — `45de358`
**Commit:** `45de358` — *fix: prevent periodic timer from overriding RL agent parent choice*

**Problem:** Even after fixing the immediate DIO handler, NPC was still 551 (while the RL agent only did 7 switches). The background timer `rpl_recalculate_ranks()` was firing every few seconds, calling `rpl_process_parent_event()`, which eventually called `rpl_select_parent()`. This function invoked the standard OF-TAU `best_parent()` logic, which disagreed with the RL agent's choice and overrode it, causing ongoing background ping-pong.

**Fix:** Modified `rpl_select_parent()` to respect the existing preferred parent. If the RL agent or Panic Monitor has set a valid parent, `rpl_select_parent()` returns it directly without calling `best_parent()`. This fully isolates the RL agent as the sole authority for parent handoffs.

| File | Location | Change |
|---|---|---|
| `rpl-dag.c` | L1283 | If `dag->preferred_parent != NULL`, skip `best_parent()` and return `dag->preferred_parent` directly. |

**Results:** Simulation logs for this version are stored in `e:\3emeAnneeEMP\PFE\Implémentation\Results\203040SmartCity\OnlyTauDIO-RL-Lakhlef\7-finalNPCFix`

---

### 4. PDR Recovery Tuning (Relaxing Hysteresis) — `5357eed`
**Commit:** `5357eed` — *perf: tune RL parameters for better PDR reactivity*

**Problem:** After completely isolating the RL agent in `7-finalNPCFix`, the network became extremely stable (NPC dropped from 551 to 12). However, this extreme stickiness caused the PDR to drop from 99% to ~93.6%, because nodes were holding onto their parents for too long when links degraded instead of jumping ship to a better neighbor.

**Fix:** Relaxed the Q-Learning and physical hysteresis thresholds to make the RL agent slightly more reactive, aiming to find the sweet spot between absolute stability and high packet delivery ratios.

| File | Parameter | Old Value | New Value | Effect |
|---|---|---|---|---|
| `rpl-rl-agent.h` | `RL_MIN_SWITCH_GAIN` | 5 | 3 | Agent requires less predicted Q-gain to switch |
| `rpl-rl-agent.h` | `RL_HYSTERESIS_TAU` | 75 | 40 | Agent reacts quicker to physical TAU advantages |
| `rpl-rl-agent.h` | `RL_HYSTERESIS_RSSI` | 5 | 3 | Smaller RSSI gap needed to confirm the switch |

**Results:** Simulation logs for this version are stored in `e:\3emeAnneeEMP\PFE\Implémentation\Results\203040SmartCity\OnlyTauDIO-RL-Lakhlef\8-PDRRecoveryTuning`

---

### 5. Gate 3 Hysteresis Logic Fix (`OR` instead of `AND`) — `30c88ab`
**Commit:** `30c88ab` — *fix: change hysteresis logic to OR and resolve compilation cache issue*

**Problem:** The previous run (`8-PDRRecoveryTuning`) yielded the exact same results as `7-finalNPCFix`. This was caused by two issues:
1. **Compilation Caching:** Cooja did not recompile the `.c` files when only `rpl-rl-agent.h` was changed, running the old binary.
2. **Logic Bug:** The code in `rpl-rl-agent.c` required a candidate to have better TAU, **AND** better RSSI, **AND** better ETX (`&&`). If ETX was already 1.0, it was mathematically impossible to improve it, blocking the RL agent from switching parents entirely.

**Fix:** Changed `&&` to `||` in `rpl-rl-agent.c`. Now, if a candidate is significantly better in *any* of the key metrics (TAU, RSSI, or ETX), the agent is permitted to switch.
*Note: Users must run `make clean` before reloading the simulation to ensure changes take effect!*

| File | Fix | Effect |
|---|---|---|
| `rpl-rl-agent.c` | Line 423 | `candidate_clearly_better` now uses `||` (OR) instead of `&&` (AND). |

**Results:** Simulation logs for this version are stored in `e:\3emeAnneeEMP\PFE\Implémentation\Results\203040SmartCity\OnlyTauDIO-RL-Lakhlef\9-Gate3LogicFix`

---

### 6. Gate 1 Proactive Routing Fix — `c5ab50d`
**Commit:** `c5ab50d` — *fix: wake up RL agent for better candidates even if current parent is healthy*

**Problem:** Even after fixing Gate 3 and recompiling, the RL agent was never running. Analysis of Gate 1 (Case B) revealed a strict rule: if the current parent was "healthy" (RSSI > -85 and ETX < 3), the agent completely ignored all DIOs from other neighbors to save CPU. This effectively disabled proactive routing, forcing the agent to stay with its first parent until it physically drove out of range. 

**Fix:** Modified Gate 1 in `rpl-rl-agent.c` to wake up the RL agent if either the current parent is weak, OR the new candidate's TAU score is significantly better than the current parent's (`candidate_is_much_better = v->tau_cand > current->tau_cand + RL_HYSTERESIS_TAU`).

| File | Fix | Effect |
|---|---|---|
| `rpl-rl-agent.c` | Line 512 | RL Agent now triggers proactively for better candidates, re-enabling optimal path finding. |

**Results:** Simulation logs for this version are stored in `e:\3emeAnneeEMP\PFE\Implémentation\Results\203040SmartCity\OnlyTauDIO-RL-Lakhlef\10-Gate1ProactiveFix`

---

### 7. Tuning for 99% PDR Recovery — `86efb2a`
**Commit:** `86efb2a` — *chore: tune hysteresis for higher PDR and increase Cooja RAM*

**Problem:** The proactive fix successfully woke up the agent, recovering PDR from 93.67% to 95.18%. However, to push it closer to the 99% mark while retaining a reasonable NPC, the agent needs to be slightly more eager to switch to better paths before the current link degrades further.
Additionally, Cooja was previously limited to 1.5GB of RAM, risking crashes during heavy mobility simulations.

**Fix:** 
1. Lowered `RL_HYSTERESIS_TAU` in `rpl-rl-agent.h` to make the agent switch routes more quickly when a moderately better path appears.
2. Increased Cooja Java RAM allocation in `Tunnel/RUN_TEST` from 1536MB to 6144MB (6GB) to improve stability during long simulations.

| File | Parameter | Old Value | New Value | Reason |
|---|---|---|---|---|
| `rpl-rl-agent.h` | `RL_HYSTERESIS_TAU` | 40 | 20 | Agent reacts even quicker to physical TAU advantages, recovering more packets before link loss. |
| `Tunnel/RUN_TEST` | `java -mx...` | `1536m` | `6144m` | Give Cooja 6GB of RAM to prevent OutOfMemory crashes. |

**Results:** Simulation logs for this version are stored in `e:\3emeAnneeEMP\PFE\Implémentation\Results\203040SmartCity\OnlyTauDIO-RL-Lakhlef\11-PDR99Tuning`

---

### 8. Strict "Way Better" Candidate Evaluation Fix — `<PENDING>`
**Commit:** `<will be filled after commit>` — *refactor: implement strict candidate evaluation with ETX safeguard*

**Problem:** Run 11 dropped PDR to 91.61% because the `OR` logic introduced in Run 10 caused too many parent switches (NPC = 51). However, the original `AND` logic from Run 7 caused the agent to lock up (NPC = 12, PDR = 93%) because it required candidates to have a *strictly better* ETX than the current parent. If the current parent had a perfect ETX (1.0), it was mathematically impossible for candidates to be "better", freezing the agent.

**Fix:** Refactored Gate 1 and Gate 3 in `rpl-rl-agent.c` to use a strict unified `AND` rule, as requested: The candidate must have better TAU, better RSSI, AND... **equal or better ETX** (`delta_etx >= 0`). This perfectly balances stability (by requiring multiple improvements) without mathematically locking the agent when the link is perfect.

| File | Fix | Effect |
|---|---|---|
| `rpl-rl-agent.c` | Line 423 | Gate 3 `candidate_clearly_better` now uses strict AND logic with `>= 0` ETX. |
| `rpl-rl-agent.c` | Line 512 | Gate 1 `candidate_is_much_better` now uses the exact same strict AND logic to wake up. |

**Results:** Simulation logs for this version are stored in `e:\3emeAnneeEMP\PFE\Implémentation\Results\203040SmartCity\OnlyTauDIO-RL-Lakhlef\12-WayBetterLogicFix`