/*---------------------------------------------------------------------------*/
/* rpl-of-tau.c
 *
 * Objective Function TAU for RPL.
 *
 * Parent selection based on tau_cand (0..1000), which integrates:
 *   τ_cand = a·RE + b·(1000-QL) + c·Deg + d·(1000-NPC)
 *          + e·ETX_norm + f·RSSI_norm + g·τ_parent
 *   All normalized 0..1000, divided by (a+b+c+d+e+f+g).
 *
 * best_parent() = argmax(tau_cand) + hysteresis + rank constraint
 * rank_via_parent() = ETX-based (guarantees DODAG loop-free property)
 */
/*---------------------------------------------------------------------------*/

#include "contiki.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/link-stats.h"
#include <limits.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

/* Fallback if not defined by platform */
#ifndef LINK_STATS_ETX_DIVISOR
#define LINK_STATS_ETX_DIVISOR 128
#endif

/* Parameters */
#ifndef RPL_OF_TAU_MAX_ETX
#define RPL_OF_TAU_MAX_ETX (8 * LINK_STATS_ETX_DIVISOR) /* ETX <= 3 (Proactive rejection of weak links) */
#endif

#ifndef RPL_OF_TAU_INIT_ETX
#define RPL_OF_TAU_INIT_ETX (2 * LINK_STATS_ETX_DIVISOR) /* default ETX if stats absent */
#endif

#ifndef RPL_OF_TAU_SWITCH_THRESHOLD
#define RPL_OF_TAU_SWITCH_THRESHOLD 200 /* hysteresis in tau points (0..1000) */
#endif

#ifndef RPL_OF_TAU_MIN_TAU
#define RPL_OF_TAU_MIN_TAU 1 /* minimal tau to accept a parent */
#endif

/*---------------------------------------------------------------------------*/
static uint16_t
clamp_tau(uint16_t tau)
{
  if(tau > 1000) return 1000;
  return tau;
}
/*---------------------------------------------------------------------------*/
static uint16_t
get_etx_or_default(rpl_parent_t *p)
{
  const struct link_stats *ls = rpl_get_parent_link_stats(p);
  if(ls != NULL && ls->etx > 0) {
    return ls->etx;
  }
  return RPL_OF_TAU_INIT_ETX;
}
/*---------------------------------------------------------------------------*/
static void
reset(rpl_dag_t *dag)
{
  (void)dag;
  PRINTF("RPL: Reset OF-TAU\n");
}
/*---------------------------------------------------------------------------*/
static uint16_t
parent_link_metric(rpl_parent_t *p)
{
  /* ETX kept for debug / rank_via_parent.
   * Parent decision occurs via tau_cand in best_parent(). */
  return get_etx_or_default(p);
}
/*---------------------------------------------------------------------------*/
static int
parent_has_usable_link(rpl_parent_t *p)
{
  if(p == NULL) return 0;

  uint16_t tau = clamp_tau(p->tau_cand);
  uint16_t etx = get_etx_or_default(p);

  if(tau < RPL_OF_TAU_MIN_TAU) return 0;
  if(etx == 0xFFFF || etx == 0) return 0;
  if(etx > RPL_OF_TAU_MAX_ETX) return 0;

  return 1;
}
/*---------------------------------------------------------------------------*/
static uint16_t
parent_path_cost(rpl_parent_t *p)
{
  /* Lower = better.
   * tau_cand already encodes the full path quality (via merge).
   * Path cost = inverse of tau. */
  if(p == NULL) return 0xFFFF;

  uint16_t tau = clamp_tau(p->tau_cand);
  return (uint16_t)(1000 - tau);
}
/*---------------------------------------------------------------------------*/
static rpl_rank_t
rank_via_parent(rpl_parent_t *p)
{
  if(p == NULL || p->dag == NULL || p->dag->instance == NULL) {
    return INFINITE_RANK;
  }

  rpl_instance_t *instance = p->dag->instance;
  uint16_t etx = get_etx_or_default(p);

  if(etx == 0xFFFF || etx == 0 || etx > RPL_OF_TAU_MAX_ETX) {
    return INFINITE_RANK;
  }

  /* Rank increment based on ETX, guaranteed >= min_hoprankinc */
  uint32_t inc = ((uint32_t)instance->min_hoprankinc * (uint32_t)etx) / LINK_STATS_ETX_DIVISOR;
  if(inc < instance->min_hoprankinc) {
    inc = instance->min_hoprankinc;
  }

  if(instance->max_rankinc != 0 && inc > instance->max_rankinc) {
    inc = instance->max_rankinc;
  }

  if(p->rank == INFINITE_RANK) {
    return INFINITE_RANK;
  }
  if((uint32_t)p->rank + inc >= 0xFFFF) {
    return INFINITE_RANK;
  }
  return (rpl_rank_t)(p->rank + inc);
}
/*---------------------------------------------------------------------------*/
static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  if(p1 == NULL) return p2;
  if(p2 == NULL) return p1;

  /* Discard parents with unusable links */
  if(!parent_has_usable_link(p1)) return p2;
  if(!parent_has_usable_link(p2)) return p1;

  /* --- Hop count / rank constraint ---
   * Reject parents whose rank would make us infinite.
   * This guarantees Rank(parent) < Rank(child) => loop-free DODAG. */
  rpl_rank_t r1 = rank_via_parent(p1);
  rpl_rank_t r2 = rank_via_parent(p2);
  if(r1 == INFINITE_RANK && r2 == INFINITE_RANK) return p1; /* both bad */
  if(r1 == INFINITE_RANK) return p2;
  if(r2 == INFINITE_RANK) return p1;

  rpl_dag_t *dag = p1->dag; /* same DAG */
  uint16_t t1 = clamp_tau(p1->tau_cand);
  uint16_t t2 = clamp_tau(p2->tau_cand);

  /* Hysteresis: if current preferred parent is "close enough", keep it */
  if(dag != NULL && (p1 == dag->preferred_parent || p2 == dag->preferred_parent)) {
    rpl_parent_t *pref = dag->preferred_parent;
    if(pref == p1) {
      if(t2 <= (uint16_t)(t1 + RPL_OF_TAU_SWITCH_THRESHOLD)) {
        return p1;
      }
    } else if(pref == p2) {
      if(t1 <= (uint16_t)(t2 + RPL_OF_TAU_SWITCH_THRESHOLD)) {
        return p2;
      }
    }
  }

  /* Primary rule: max tau_cand */
  if(t2 > t1) return p2;
  if(t1 > t2) return p1;

  /* Tie-break 1: prefer lower rank (closer to root = fewer hops) */
  if(r2 < r1) return p2;
  if(r1 < r2) return p1;

  /* Tie-break 2: prefer lower ETX (better direct link) */
  {
    uint16_t e1 = parent_link_metric(p1);
    uint16_t e2 = parent_link_metric(p2);
    return (e2 < e1) ? p2 : p1;
  }
}
/*---------------------------------------------------------------------------*/
static rpl_dag_t *
best_dag(rpl_dag_t *d1, rpl_dag_t *d2)
{
  if(d1 == NULL) return d2;
  if(d2 == NULL) return d1;

  if(d1->grounded != d2->grounded) {
    return d1->grounded ? d1 : d2;
  }
  if(d1->preference != d2->preference) {
    return d1->preference > d2->preference ? d1 : d2;
  }
  return d1->rank <= d2->rank ? d1 : d2;
}
/*---------------------------------------------------------------------------*/
static void
update_metric_container(rpl_instance_t *instance)
{
  /* OF-TAU propagates state via custom PE option, no standard MC needed */
  instance->mc.type = RPL_DAG_MC_NONE;
}
/*---------------------------------------------------------------------------*/
/* Export OF */
rpl_of_t rpl_of_tau = {
  .reset = reset,
#if RPL_WITH_DAO_ACK
  .dao_ack_callback = NULL,
#endif
  .parent_link_metric = parent_link_metric,
  .parent_has_usable_link = parent_has_usable_link,
  .parent_path_cost = parent_path_cost,
  .rank_via_parent = rank_via_parent,
  .best_parent = best_parent,
  .best_dag = best_dag,
  .update_metric_container = update_metric_container,
  .ocp = RPL_OCP_TAU
};
