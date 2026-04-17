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
/* Hysteresis: challenger must beat current parent by this margin (0..1000 scale) */
#undef  RPL_OF_TAU_SWITCH_THRESHOLD
#define RPL_OF_TAU_SWITCH_THRESHOLD 75

#endif /* PROJECT_CONF_H_ */
