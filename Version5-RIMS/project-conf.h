/*
 * project-conf.h
 *
 * Configuration for RIMS-RPL Version 5
 * Enables:
 *   - OF-TAU objective function (OCP=2)
 *   - RIMS-RPL Q-Learning mobility support
 *   - Cross-layer ERP metric
 *   - Mobility-aware parent selection
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*---------------------------------------------------------------------------*/
/* RIMS-RPL: Enable the Reinforcement Learning-based Intelligent
 * Mobility-Support extension. This activates:
 *   1. ERP (Expected Reliability Percentage) cross-layer metric
 *   2. OF-TAU objective function with mobility-aware parent selection
 *   3. Q-Learning proactive monitoring agent
 */
#define RPL_CONF_WITH_RIMS          1

/*---------------------------------------------------------------------------*/
/* Objective Function: Use OF-TAU (OCP=2) */
#define RPL_OCP_OF_TAU              2
#define RPL_CONF_OF_OCP             RPL_OCP_OF_TAU
#define RPL_CONF_SUPPORTED_OFS      {&rpl_of_tau}

/*---------------------------------------------------------------------------*/
/* Mobility is dynamically deduced via the RIMS-RPL Hybrid Algorithm (EWMA). */

/*---------------------------------------------------------------------------*/
/* RPL Statistics: Enable for parent switch logging */
#define RPL_CONF_STATS              1

/*---------------------------------------------------------------------------*/
/* RPL Timers: Conservative settings to avoid DIO storms */
#define RPL_CONF_DIO_INTERVAL_MIN   12   /* 2^12 ms = 4.096s */
#define RPL_CONF_DIO_INTERVAL_DOUBLINGS  8
#define RPL_CONF_DIO_REDUNDANCY     10

/*---------------------------------------------------------------------------*/
/* RIMS Q-Learning tuning (optional overrides) */
/* #define RIMS_CONF_THRESHOLD_U    10 */  /* Unchanged cycles before OUTDATED */
/* #define RIMS_CONF_THRESHOLD_C     5 */  /* Changed cycles before HANDOFF */
/* #define RIMS_CONF_MONITOR_INTERVAL (5 * CLOCK_SECOND) */

/*---------------------------------------------------------------------------*/
/* OF-TAU parameter tuning (optional overrides) */
/* #define RPL_OF_TAU_CONF_ALPHA_W  4 */   /* Weight for ETX */
/* #define RPL_OF_TAU_CONF_BETA_W   2 */   /* Weight for (ERP - RSSI) */
/* #define RPL_OF_TAU_CONF_SWITCH_THRESHOLD 300 */ /* Hysteresis */

/*---------------------------------------------------------------------------*/
/* Network configuration */
#define NETSTACK_CONF_RDC          nullrdc_driver
#define NETSTACK_CONF_MAC          csma_driver

/*---------------------------------------------------------------------------*/
/* Logging / Debugging (uncomment to enable verbose RPL output) */
/* #define RPL_DEBUG DEBUG_PRINT */

#endif /* PROJECT_CONF_H_ */
