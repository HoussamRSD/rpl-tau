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

### 3. Fix Background Timer NPC Inflation (Periodic Overrides) — `<PENDING>`
**Commit:** `<will be filled after commit>` — *fix: prevent periodic timer from overriding RL agent parent choice*

**Problem:** Even after fixing the immediate DIO handler, NPC was still 551 (while the RL agent only did 7 switches). The background timer `rpl_recalculate_ranks()` was firing every few seconds, calling `rpl_process_parent_event()`, which eventually called `rpl_select_parent()`. This function invoked the standard OF-TAU `best_parent()` logic, which disagreed with the RL agent's choice and overrode it, causing ongoing background ping-pong.

**Fix:** Modified `rpl_select_parent()` to respect the existing preferred parent. If the RL agent or Panic Monitor has set a valid parent, `rpl_select_parent()` returns it directly without calling `best_parent()`. This fully isolates the RL agent as the sole authority for parent handoffs.

| File | Location | Change |
|---|---|---|
| `rpl-dag.c` | L1283 | If `dag->preferred_parent != NULL`, skip `best_parent()` and return `dag->preferred_parent` directly. |