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
/*  TAU weights  (a, b, c, d, e, f, g)                     */
/*  τ_cand = a·RE + b·(1000-QL) + c·Deg + d·(1000-NPC)    */
/*         + e·ETX_norm + f·RSSI_norm + g·τ_parent         */
/* ──────────────────────────────────────────────────────── */
#define W_RE    4   /* a: Residual Energy         */
#define W_QL    2   /* b: Queue Load (inverted)   */
#define W_DEG   1   /* c: Degree                  */
#define W_NPC   2   /* d: Nb Parent Changes (inv) */
#define W_ETX   4   /* e: Direct link ETX         */
#define W_RSSI  2   /* f: Direct link RSSI        */
#define W_TAU   5   /* g: Recursive τ from parent */

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

#endif /* PROJECT_CONF_H_ */
