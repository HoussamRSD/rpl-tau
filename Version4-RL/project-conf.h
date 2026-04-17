/*
 * project-conf.h — Version1 (OF-TAU)
 *
 * Configuration for RPL with custom Objective Function TAU.
 * Drop this file next to your Makefile / main .c file.
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* ──────────────────────────────────────────────────────── */
/*  Networking basics                                       */
/* ──────────────────────────────────────────────────────── */

#ifndef WITH_NON_STORING
#define WITH_NON_STORING 0 /* Set to 1 for non-storing mode */
#endif

#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#undef UIP_CONF_MAX_ROUTES

#ifdef TEST_MORE_ROUTES
#define NBR_TABLE_CONF_MAX_NEIGHBORS  10
#define UIP_CONF_MAX_ROUTES           30
#else
#define NBR_TABLE_CONF_MAX_NEIGHBORS  10
#define UIP_CONF_MAX_ROUTES           10
#endif

/* Disable RDC for Cooja simulation (pure CSMA) */
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC    nullrdc_driver
#undef NULLRDC_CONF_802154_AUTOACK
#define NULLRDC_CONF_802154_AUTOACK   1

/* Route lifetimes */
#define RPL_CONF_DEFAULT_LIFETIME_UNIT  60
#define RPL_CONF_DEFAULT_LIFETIME       10
#define RPL_CONF_DEFAULT_ROUTE_INFINITE_LIFETIME 1

/* Save ROM */
#undef  UIP_CONF_TCP
#define UIP_CONF_TCP  0
#undef  SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG 0

#if WITH_NON_STORING
#undef  RPL_NS_CONF_LINK_NUM
#define RPL_NS_CONF_LINK_NUM  40
#undef  UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES   0
#undef  RPL_CONF_MOP
#define RPL_CONF_MOP  RPL_MOP_NON_STORING
#endif

/* ──────────────────────────────────────────────────────── */
/*  OF-TAU selection                                        */
/* ──────────────────────────────────────────────────────── */

/* Use OF-TAU as the default Objective Function */
#undef RPL_CONF_SUPPORTED_OFS
#define RPL_CONF_SUPPORTED_OFS  { &rpl_of_tau, &rpl_mrhof }

#undef RPL_CONF_OF_OCP
#define RPL_CONF_OF_OCP  RPL_OCP_TAU   /* 0xF2, defined in rpl.h */




/* ──────────────────────────────────────────────────────── */
/*  DYNAMIQUE DU RÉSEAU (Mobility Optimizations)           */
/* ──────────────────────────────────────────────────────── */

/* Activer UNIQUEMENT le NUD Probing actif de RPL (sans accélérer le Trickle DIO) 
   Cela permet de vite purger un voisin mort sans inonder le réseau de contrôle. */
#undef RPL_CONF_PROBING_INTERVAL
#define RPL_CONF_PROBING_INTERVAL (10 * CLOCK_SECOND) 

/* ──────────────────────────────────────────────────────── */
/*  RL Agent tuning (optional overrides)                    */
/* ──────────────────────────────────────────────────────── */
/* Learning rate: 1/RL_ALPHA_INV. Increase for faster adaptation,
 * decrease for more stability. Default = 64 (alpha ≈ 0.015). */
/* #define RL_ALPHA_INV   64 */

/* Noise filter: ignore rewards smaller than this (0..1000 scale).
 * Default = 5. Increase if weights oscillate on a stable topology. */
/* #define RL_REWARD_DEADBAND  5 */

/* Update every N DIO cycles. Default = 4.
 * Set to 1 for maximum responsiveness in highly mobile scenarios. */
/* #define RL_UPDATE_PERIOD  4 */

/* ──────────────────────────────────────────────────────── */
/*  Energest (needed for real energy measurement)           */
/* ──────────────────────────────────────────────────────── */
#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON  1
#endif

/* ──────────────────────────────────────────────────────── */
/*  RPL stats (optional, for debugging)                     */
/* ──────────────────────────────────────────────────────── */
#define RPL_CONF_STATS 1

/* MC: we propagate tau via custom PE option, MC not strictly needed */
#define RPL_CONF_DAG_MC  RPL_DAG_MC_NONE

/* ──────────────────────────────────────────────────────── */
/* Hysteresis: challenger must beat current parent by this margin (0..1000 scale)
 * Raised to 150 to cut ping-pong switches in half while still allowing
 * genuine parent changes when a significantly better candidate exists. */
#undef  RPL_OF_TAU_SWITCH_THRESHOLD
#define RPL_OF_TAU_SWITCH_THRESHOLD 150

/* DIO Trickle redundancy constant k:
 * Trickle suppresses a DIO once k neighbors have already sent one.
 * Lowering from default 10 to 3 means Trickle silences much sooner
 * in a semi-stable network, dramatically cutting DIO overhead. */
#undef  RPL_CONF_DIO_REDUNDANCY
#define RPL_CONF_DIO_REDUNDANCY 3

#endif /* PROJECT_CONF_H_ */
