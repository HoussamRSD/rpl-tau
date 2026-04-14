/*---------------------------------------------------------------------------*/
/* rpl-of-tau.c — Objective Function TAU for RPL (Version 3)
 *
 * Parent selection based on tau_cand (0..1000):
 *   tau_cand = weighted_sum(RE, 1000-QL, Deg, 1000-NPC, ETX_norm, RSSI_norm, tau_parent)
 *              / sum_of_weights
 *
 * best_parent() = argmax(tau_cand) with hysteresis + rank loop-free constraint
 * rank_via_parent() = ETX-based rank increment (guarantees loop-free DODAG)
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/link-stats.h"
#include "sys/ctimer.h"
#include <limits.h>

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#ifndef LINK_STATS_ETX_DIVISOR
#define LINK_STATS_ETX_DIVISOR 128
#endif

/* Maximum ETX to accept a parent (raw link-stats units) */
#ifndef RPL_OF_TAU_MAX_ETX
#define RPL_OF_TAU_MAX_ETX  (8 * LINK_STATS_ETX_DIVISOR)
#endif

/* Default ETX when link-stats have no data yet */
#ifndef RPL_OF_TAU_INIT_ETX
#define RPL_OF_TAU_INIT_ETX (2 * LINK_STATS_ETX_DIVISOR)
#endif

/* Challenger must beat current parent by this margin to trigger a switch */
#ifndef RPL_OF_TAU_SWITCH_THRESHOLD
#define RPL_OF_TAU_SWITCH_THRESHOLD 50   /* 5% margin: fast switching for mobility */
#endif

/* Minimum tau_cand to consider a parent usable at all */
#ifndef RPL_OF_TAU_MIN_TAU
#define RPL_OF_TAU_MIN_TAU  1
#endif

/* Emergency floor: if preferred parent tau drops below this, switch immediately */
#ifndef RPL_OF_TAU_PANIC_THRESHOLD
#define RPL_OF_TAU_PANIC_THRESHOLD  350  
#endif

static struct ctimer panic_monitor_timer;
static int panic_timer_started = 0;
static void handle_panic_monitor(void *ptr);

/*---------------------------------------------------------------------------*/
static uint16_t
clamp_tau(uint16_t tau)
{
  return tau > 1000 ? 1000 : tau;
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
handle_panic_monitor(void *ptr)
{
  rpl_instance_t *instance = rpl_get_default_instance();
  if(instance != NULL && instance->current_dag != NULL) {
    rpl_dag_t *dag = instance->current_dag;
    rpl_parent_t *p = dag->preferred_parent;
    /* Read the already-computed tau_cand — do NOT overwrite it here */
    if(p != NULL && p->tau_cand < RPL_OF_TAU_PANIC_THRESHOLD) {
      PRINTF("RPL TAU: PANIC tau=%u < %u, forcing re-eval\n",
             p->tau_cand, (uint16_t)RPL_OF_TAU_PANIC_THRESHOLD);
      rpl_select_parent(dag);
    }
  }
  /* Reschedule with explicit interval (never use ctimer_reset) */
  ctimer_set(&panic_monitor_timer, 2000 * CLOCK_SECOND, handle_panic_monitor, NULL);
}
/*---------------------------------------------------------------------------*/
static void
reset(rpl_dag_t *dag)
{
  (void)dag;
  PRINTF("RPL: OF-TAU reset\n");
}
/*---------------------------------------------------------------------------*/
static uint16_t
parent_link_metric(rpl_parent_t *p)
{
  return get_etx_or_default(p);
}
/*---------------------------------------------------------------------------*/
static int
parent_has_usable_link(rpl_parent_t *p)
{
  uint16_t tau, etx;
  if(p == NULL) return 0;

  /* Lazy-start panic monitor on first call */
  if(!panic_timer_started) {
    panic_timer_started = 1;
    ctimer_set(&panic_monitor_timer, 5 * CLOCK_SECOND, handle_panic_monitor, NULL);
  }

  tau = clamp_tau(p->tau_cand);
  etx = get_etx_or_default(p);

  if(tau < RPL_OF_TAU_MIN_TAU)          return 0;
  if(etx == 0xFFFF || etx == 0)         return 0;
  if(etx > RPL_OF_TAU_MAX_ETX)          return 0;
  return 1;
}
/*---------------------------------------------------------------------------*/
static uint16_t
parent_path_cost(rpl_parent_t *p)
{
  if(p == NULL) return 0xFFFF;
  return (uint16_t)(1000 - clamp_tau(p->tau_cand));
}
/*---------------------------------------------------------------------------*/
static rpl_rank_t
rank_via_parent(rpl_parent_t *p)
{
  uint16_t min_hoprankinc;
  uint16_t path_cost;

  if(p == NULL || p->dag == NULL || p->dag->instance == NULL) {
    return INFINITE_RANK;
  }

  min_hoprankinc = p->dag->instance->min_hoprankinc;
  /* path_cost = parent rank + link ETX (additive, same as MRHOF) */
  path_cost = (uint16_t)MIN((uint32_t)p->rank + get_etx_or_default(p), 0xffff);

  /* Rank lower-bound: parent rank + min_hoprankinc  ← exact MRHOF formula */
  return MAX(MIN((uint32_t)p->rank + min_hoprankinc, 0xffff), path_cost);
}
/*---------------------------------------------------------------------------*/
static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  rpl_rank_t r1, r2;
  rpl_dag_t *dag;
  uint16_t t1, t2;

  if(p1 == NULL) return p2;
  if(p2 == NULL) return p1;

  /* Filter 1: unusable links */
  if(!parent_has_usable_link(p1)) return p2;
  if(!parent_has_usable_link(p2)) return p1;

  /* Filter 2: rank/loop constraint */
  r1 = rank_via_parent(p1);
  r2 = rank_via_parent(p2);
  if(r1 == INFINITE_RANK && r2 == INFINITE_RANK) return p1;
  if(r1 == INFINITE_RANK) return p2;
  if(r2 == INFINITE_RANK) return p1;

  dag = p1->dag;
  t1  = clamp_tau(p1->tau_cand);
  t2  = clamp_tau(p2->tau_cand);

  PRINTF("RPL: OF-TAU comparing parents: tau1=%u vs tau2=%u\n", t1, t2);

  /* Filter 3: Emergency floor — bypass hysteresis if preferred parent is critical */
  if(dag != NULL) {
    rpl_parent_t *pref = dag->preferred_parent;
    if(pref != NULL && clamp_tau(pref->tau_cand) < RPL_OF_TAU_PANIC_THRESHOLD) {
      return (t2 > t1) ? p2 : p1;
    }
  }

  /* Filter 4: Hysteresis — keep current preferred parent if challenger is not clearly better */
  if(dag != NULL && (p1 == dag->preferred_parent || p2 == dag->preferred_parent)) {
    rpl_parent_t *pref = dag->preferred_parent;
    if(pref == p1) {
      if(t2 <= (uint16_t)(t1 + RPL_OF_TAU_SWITCH_THRESHOLD)) return p1;
    } else if(pref == p2) {
      if(t1 <= (uint16_t)(t2 + RPL_OF_TAU_SWITCH_THRESHOLD)) return p2;
    }
  }

  /* Filter 5: argmax(tau_cand) */
  if(t2 > t1) return p2;
  if(t1 > t2) return p1;

  /* Tie-break 1: lower rank (fewer hops) */
  if(r2 < r1) return p2;
  if(r1 < r2) return p1;

  /* Tie-break 2: lower ETX (better direct link) */
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
  if(d1->grounded != d2->grounded) return d1->grounded ? d1 : d2;
  if(d1->preference != d2->preference) return d1->preference > d2->preference ? d1 : d2;
  return d1->rank <= d2->rank ? d1 : d2;
}
/*---------------------------------------------------------------------------*/
static void
update_metric_container(rpl_instance_t *instance)
{
  instance->mc.type = RPL_DAG_MC_NONE;
}
/*---------------------------------------------------------------------------*/
rpl_of_t rpl_of_tau = {
  .reset                  = reset,
#if RPL_WITH_DAO_ACK
  .dao_ack_callback       = NULL,
#endif
  .parent_link_metric     = parent_link_metric,
  .parent_has_usable_link = parent_has_usable_link,
  .parent_path_cost       = parent_path_cost,
  .rank_via_parent        = rank_via_parent,
  .best_parent            = best_parent,
  .best_dag               = best_dag,
  .update_metric_container= update_metric_container,
  .ocp                    = RPL_OCP_TAU
};
