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

/* ──────────────────────────────────────────────────────── */
/*  POIDS DE LA FONCTION OBJECTIVE TAU (a, b, c, d, e, f, g) */
/*
 *  L'équation fondamentale de prise de décision pour devenir Parent :
 *  τ_cand = (a·RE) + (b·[1000-QL]) + (c·Deg) + (d·[1000-NPC])
 *         + (e·ETX_norm) + (f·RSSI_norm) + (g·τ_parent)
 *
 *  Ajustez ces poids (0 à 10+) pour modifier le comportement de la topologie :
 *  Par exemple, augmenter `W_RE` forcera les nœuds à privilégier les parents 
 *  qui ont le plus de batterie. Augmenter `W_ETX` privilégiera un bon signal WiFi.
 */
/* ──────────────────────────────────────────────────────── */
#define W_RE    4   /* a: RE (Residual Energy) - Protege la duree de vie globale du reseau */
#define W_QL    2   /* b: QL (Queue Load) - Inverse (1000-QL). Evite les noeuds goulots d etranglement */
#define W_DEG   1   /* c: Deg (Degree) - Pousse a rejoindre les noeuds hyper-centraux (bien connectes) */
#define W_NPC   1   /* d: NPC (Node Parent Changes) - Tres faible pour la Mobilite (on autorise le saut) ! */
#define W_ETX   5   /* e: ETX (Couche MAC) - Forte valeur pour privilegier un lien tres fiable (Anti ping-pong). */
#define W_RSSI  3   /* f: RSSI (Radio) - Favorise la proximite spatiale de maniere moderee */
#define W_TAU   5   /* g: Tau Parent - Garantit que le "bonheur" d un parent se propage recursivement aux enfants */

/* NB: W_RE et W_QL s'appliquent UNIQUEMENT au tau local diffuse dans les DIOs vers les enfants.
 *     Ils N'AFFECTENT PAS le calcul de tau_cand (choix du parent), qui utilise W_ETX, W_RSSI, W_TAU. */

/* ────────────────────────────────────────────────────────────────── */
/*  Energest (needed for real energy measurement)                     */
/* ────────────────────────────────────────────────────────────────── */
#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON  1
#endif

/* ────────────────────────────────────────────────────────────────── */
/*  Constantes energie residuelle (RE) - overridables                 */
/*  RPL_ENERGY_INIT_MJ : Capacite initiale de la batterie simulee (mJ) */
/*  RE_MAX = 1000 (normalise, batterie pleine)                        */
/* ────────────────────────────────────────────────────────────────── */
#ifndef RPL_ENERGY_INIT_MJ
#define RPL_ENERGY_INIT_MJ       3000UL  /* 3 Joules */
#endif
#ifndef RPL_ENERGY_VOLTAGE_MV
#define RPL_ENERGY_VOLTAGE_MV    3300UL  /* 3.3V */
#endif
#ifndef RPL_I_CPU_UA
#define RPL_I_CPU_UA              1800UL
#endif
#ifndef RPL_I_LPM_UA
#define RPL_I_LPM_UA                55UL
#endif
#ifndef RPL_I_TX_UA
#define RPL_I_TX_UA              17400UL
#endif
#ifndef RPL_I_RX_UA
#define RPL_I_RX_UA              19700UL
#endif

/* ────────────────────────────────────────────────────────────────── */
/*  Constante Queue Load (QL) - overridable                           */
/*  BUFFER_SIZE_MAX = 1000 (normalise, file vide)                     */
/* ────────────────────────────────────────────────────────────────── */
#ifndef RPL_QUEUE_MAX_PACKETS
#define RPL_QUEUE_MAX_PACKETS      8     /* taille max file CSMA Contiki */
#endif

/* ──────────────────────────────────────────────────────── */
/*  RPL stats (optional, for debugging)                     */
/* ──────────────────────────────────────────────────────── */
#define RPL_CONF_STATS 1

/* MC: we propagate tau via custom PE option, MC not strictly needed */
#define RPL_CONF_DAG_MC  RPL_DAG_MC_NONE

#undef  RPL_OF_TAU_SWITCH_THRESHOLD
#define RPL_OF_TAU_SWITCH_THRESHOLD 75

/* ──────────────────────────────────────────────────────── */
/*  Q-Learning Agent Parameters (Section 9 of RL.md)       */
/* ──────────────────────────────────────────────────────── */

/* Learning rate Alpha * 100 (10 = Alpha 0.10) */
#define RL_ALPHA              10

/* Initial exploration rate * 100 (90 = epsilon 0.90) */
#define RL_EPSILON_INITIAL    90

/* Minimum exploration floor * 100 (10 = epsilon_min 0.10) */
#define RL_EPSILON_MIN        10

/* Epsilon decay factor * 100 (95 = decay 0.95) */
#define RL_EPSILON_DECAY      95

/* Reward normalization: connection must last this long for reward = 1 */
#define RL_STABILITY_CONSTANT (30 * CLOCK_SECOND)

/* Maximum clipped reward value */
#define RL_MAX_REWARD         10

/* Neighbor considered unreachable after this many ticks without a DIO */
#define RL_NEIGHBOR_TIMEOUT   (3 * 60 * CLOCK_SECOND)

/* Panic monitor: minimum acceptable RSSI (dBm) */
#define RL_RSSI_THRESHOLD     (-85)

/* Panic monitor: maximum acceptable ETX (raw link-stats units) */
#define RL_ETX_THRESHOLD      (6 * LINK_STATS_ETX_DIVISOR)

#endif /* PROJECT_CONF_H_ */
