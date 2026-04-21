# Version 2 (No RL)

Custom OF-TAU RPL implementation focusing on heuristic, threshold-based mobility support without Reinforcement Learning.

**Key Features:**
- **OF-TAU Objective Function**: Implements customized weights and thresholds to facilitate proactive parent switching in high-mobility scenarios.
- **Metrics**: Introduces a Queue Load (QL) metric to reflect system buffer congestion and uses a Panic Monitor to rapidly detect link quality degradation.
- **Rank Optimization**: Adopts a `rank_via_parent` mechanism to align with MRHOF's robust ETX-based rank calculation, ensuring an efficient, loop-free DODAG construction.
