# Version 3 (v2 from Original RPL)

A clean re-implementation of the Version 2 (OF-TAU) logic, directly branched from and applied on top of the Original Contiki-NG RPL codebase.

**Purpose:**
Created to isolate and safely resolve the severe performance degradation ("DIO storms", high control overhead) observed in the initial v2 attempts. It restores standard Trickle timer safeguards and ensures baseline stability matches or exceeds MRHOF before further additions.
