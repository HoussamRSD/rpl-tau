# Version 5 (RIMS)

The complete integration of the RIMS-RPL mobility-support protocol into the Contiki-NG stack.

**Key Features:**
- **Cross-Layer Metric**: Implements ERP (Expected Reliability Percentage) using MAC-layer statistics to accurately evaluate link quality.
- **Mobility-Aware OF-TAU**: Prioritizes fixed nodes and leverages hysteresis-based switching for robust parent selection.
- **Q-Learning Agent**: A machine learning agent periodically monitors link health and executes proactive corrective actions (REPAIR, DIS) to minimize disconnections, reduce control overhead, and stabilize the network topology in highly dynamic mobile environments.
