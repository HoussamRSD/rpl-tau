/*
 * project-conf.h -- Version5 (MA-RPL: Mobility-Aware RPL)
 *
 * Configuration for RPL with OF-TAU + fast link failure detection,
 * backup parent instant handover, and mobility-adaptive Trickle.
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/* ---------------------------------------------------------- */
/*  Networking basics                                          */
/* ---------------------------------------------------------- */

#ifndef WITH_NON_STORING
#define WITH_NON_STORING 0
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

/* ---------------------------------------------------------- */
/*  OF-TAU selection                                            */
/* ---------------------------------------------------------- */

#undef RPL_CONF_SUPPORTED_OFS
#define RPL_CONF_SUPPORTED_OFS  { &rpl_of_tau, &rpl_mrhof }

#undef RPL_CONF_OF_OCP
#define RPL_CONF_OF_OCP  RPL_OCP_TAU

/* ---------------------------------------------------------- */
/*  MA-RPL: Mobility-Aware Mechanisms                          */
/* ---------------------------------------------------------- */

/*
 * MECHANISM 1: Fast Link Failure Detection
 * After this many consecutive MAC-layer transmission failures to the
 * preferred parent, immediately trigger parent re-evaluation.
 * This replaces the slow ETX EWMA detection (which takes 3-5 probing
 * cycles = 30-50 seconds) with near-instant detection (~100ms).
 */
#define RPL_FAST_FAIL_THRESHOLD     3

/*
 * MECHANISM 2: Aggressive Probing
 * Probe the preferred parent every 5 seconds (vs default 120s).
 * Ensures fresh link-stats even when no data traffic flows.
 */
#undef RPL_CONF_PROBING_INTERVAL
#define RPL_CONF_PROBING_INTERVAL (5 * CLOCK_SECOND)

/*
 * MECHANISM 3: Mobility-Adaptive Trickle Bound
 * In standard RPL, the Trickle DIO interval can grow to 2^20 ms (~17 min).
 * Under mobility, we cap it to 2^14 ms (~16s) so the network never goes
 * silent for too long when nodes are moving.
 */
#undef RPL_CONF_DIO_INTERVAL_DOUBLINGS
#define RPL_CONF_DIO_INTERVAL_DOUBLINGS  2  /* max = 2^(12+2) = 16s */

/*
 * MECHANISM 4: Suppress Trickle reset on smooth handover.
 * When switching to a pre-computed backup parent, the topology change
 * is LOCAL and does not need a network-wide DIO flood.
 * 1 = suppress reset, 0 = standard behavior.
 */
#define RPL_MOB_SUPPRESS_RESET  1

/* DIO Trickle redundancy: suppress DIO if k neighbors already sent one.
 * Lower value = less DIO overhead in dense networks. */
#undef  RPL_CONF_DIO_REDUNDANCY
#define RPL_CONF_DIO_REDUNDANCY 3

/* ---------------------------------------------------------- */
/*  OF-TAU Static Weights (proven baseline)                    */
/* ---------------------------------------------------------- */
/*
 * tau_cand = (a*RE + b*(1000-QL) + c*Deg + d*(1000-NPC)
 *           + e*ETX_norm + f*RSSI_norm + g*tau_parent) / sum
 *
 * ETX is dominant -- it drives parent selection based on link quality.
 * Other metrics serve as tie-breakers.
 */
#define W_RE    4
#define W_QL    2
#define W_DEG   1
#define W_NPC   1
#define W_ETX   5
#define W_RSSI  3
#define W_TAU   5

/* ---------------------------------------------------------- */
/*  Hysteresis: prevent ping-pong parent switching             */
/* ---------------------------------------------------------- */
/* Challenger must beat current parent by this margin (0..1000 scale).
 * 75 = moderate: allows genuine switches, blocks noise-driven ones. */
#undef  RPL_OF_TAU_SWITCH_THRESHOLD
#define RPL_OF_TAU_SWITCH_THRESHOLD 75

/* ---------------------------------------------------------- */
/*  Energest (needed for RE metric)                            */
/* ---------------------------------------------------------- */
#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON  1
#endif

/* ---------------------------------------------------------- */
/*  RPL stats                                                  */
/* ---------------------------------------------------------- */
#define RPL_CONF_STATS 1

/* MC: we propagate tau via custom PE option */
#define RPL_CONF_DAG_MC  RPL_DAG_MC_NONE

#endif /* PROJECT_CONF_H_ */
