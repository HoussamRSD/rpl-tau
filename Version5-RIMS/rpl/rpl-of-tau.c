/*
 * Copyright (c) 2024
 * All rights reserved.
 *
 * \file
 *   OF-TAU: Objective Function with Traffic-Aware Utility
 *   Enhanced with RIMS-RPL mobility-aware parent selection.
 *
 *   This OF uses a composite metric combining ETX, ERP (Expected
 *   Reliability Percentage), and RSSI, with fixed/mobile node
 *   priority and hysteresis-based parent switching.
 *
 *   Formula: score = (ALPHA_W * etx) - BETA_W * (path_erp - rssi)
 *   Lower score = better parent.
 *
 * \author PFE Implementation
 */

#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/nbr-table.h"
#include "net/link-stats.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
/* OF-TAU Configuration Parameters */

/* Weight for ETX component in path metric */
#ifndef RPL_OF_TAU_CONF_ALPHA_W
#define RPL_OF_TAU_ALPHA_W          4
#else
#define RPL_OF_TAU_ALPHA_W          RPL_OF_TAU_CONF_ALPHA_W
#endif

/* Weight for (ERP - RSSI) component in path metric */
#ifndef RPL_OF_TAU_CONF_BETA_W
#define RPL_OF_TAU_BETA_W           2
#else
#define RPL_OF_TAU_BETA_W           RPL_OF_TAU_CONF_BETA_W
#endif

/* Parent switch hysteresis threshold (scaled same as path_metric) */
#ifndef RPL_OF_TAU_CONF_SWITCH_THRESHOLD
#define RPL_OF_TAU_SWITCH_THRESHOLD 300
#else
#define RPL_OF_TAU_SWITCH_THRESHOLD RPL_OF_TAU_CONF_SWITCH_THRESHOLD
#endif

/* Maximum acceptable link metric (ETX) — reject parents above this */
#define MAX_LINK_METRIC             1024  /* Eq ETX of 8 */

/* Maximum acceptable path cost */
#define MAX_PATH_COST               32768 /* Eq path ETX of 256 */

/* Default RSSI fallback when not available from link-stats */
#define RSSI_FALLBACK               50    /* Treated as absolute value */

/* OCP for OF-TAU (custom, non-standard) */
#ifndef RPL_OCP_OF_TAU
#define RPL_OCP_OF_TAU              2
#endif

/*---------------------------------------------------------------------------*/
/**
 * \brief Calculate the ERP (Expected Reliability Percentage) for a parent.
 *        ERP = 100 - (12800 / etx)
 *        ETX=128 (perfect) → ERP=0, Higher ETX → Higher ERP (worse)
 *
 * \param p The parent to evaluate.
 * \return ERP value 0-100.
 */
static uint16_t
calculate_erp(rpl_parent_t *p)
{
  const struct link_stats *stats;

  if(p == NULL) {
    return 100;
  }

  stats = rpl_get_parent_link_stats(p);
  if(stats == NULL || stats->etx == 0) {
    return 100;
  }

  /* ERP = 100 - (LINK_STATS_ETX_DIVISOR * 100) / etx */
  uint16_t reliability = (uint16_t)((LINK_STATS_ETX_DIVISOR * 100UL) / stats->etx);
  if(reliability > 100) {
    reliability = 100;
  }

  return (uint16_t)(100 - reliability);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Get RSSI as positive magnitude for a parent.
 *        Uses RSSI from link-stats if available, else returns RSSI_FALLBACK.
 *        Returns the absolute value (e.g., RSSI=-70dBm → returns 70).
 *
 * \param p The parent.
 * \return RSSI absolute value (higher = weaker signal).
 */
static uint16_t
get_parent_rssi(rpl_parent_t *p)
{
  const struct link_stats *stats;

  if(p == NULL) {
    return RSSI_FALLBACK;
  }

  stats = rpl_get_parent_link_stats(p);
  if(stats == NULL) {
    return RSSI_FALLBACK;
  }

  /* link_stats stores rssi as a signed value (negative dBm).
   * If rssi field is not available in this Contiki build,
   * fall back to a constant. We use absolute value. */
#ifdef LINK_STATS_CONF_RSSI
  {
    int16_t rssi_val = stats->rssi;
    return (uint16_t)((rssi_val < 0) ? -rssi_val : rssi_val);
  }
#else
  /* Fallback: use a reasonable default */
  return RSSI_FALLBACK;
#endif
}
/*---------------------------------------------------------------------------*/
static void
reset(rpl_dag_t *dag)
{
  printf("RPL: Reset OF-TAU\n");
}
/*---------------------------------------------------------------------------*/
#if RPL_WITH_DAO_ACK
static void
dao_ack_callback(rpl_parent_t *p, int status)
{
  if(status == RPL_DAO_ACK_UNABLE_TO_ADD_ROUTE_AT_ROOT) {
    return;
  }
  printf("RPL: OF-TAU DAO ACK received with status: %d\n", status);
  if(status >= RPL_DAO_ACK_UNABLE_TO_ACCEPT) {
    link_stats_packet_sent(rpl_get_parent_lladdr(p), MAC_TX_OK, 10);
  } else if(status == RPL_DAO_ACK_TIMEOUT) {
    link_stats_packet_sent(rpl_get_parent_lladdr(p), MAC_TX_OK, 10);
  }
}
#endif /* RPL_WITH_DAO_ACK */
/*---------------------------------------------------------------------------*/
static uint16_t
parent_link_metric(rpl_parent_t *p)
{
  const struct link_stats *stats = rpl_get_parent_link_stats(p);
  if(stats != NULL) {
    return stats->etx;
  }
  return 0xffff;
}
/*---------------------------------------------------------------------------*/
static uint16_t
parent_path_cost(rpl_parent_t *p)
{
  uint16_t base;

  if(p == NULL || p->dag == NULL || p->dag->instance == NULL) {
    return 0xffff;
  }

#if RPL_WITH_MC
  switch(p->dag->instance->mc.type) {
    case RPL_DAG_MC_ETX:
      base = p->mc.obj.etx;
      break;
    case RPL_DAG_MC_ENERGY:
      base = p->mc.obj.energy.energy_est << 8;
      break;
    default:
      base = p->rank;
      break;
  }
#else /* RPL_WITH_MC */
  base = p->rank;
#endif /* RPL_WITH_MC */

  /* path cost upper bound: 0xffff */
  return MIN((uint32_t)base + parent_link_metric(p), 0xffff);
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
  path_cost = parent_path_cost(p);

  /* Rank lower-bound: parent rank + min_hoprankinc */
  return MAX(MIN((uint32_t)p->rank + min_hoprankinc, 0xffff), path_cost);
}
/*---------------------------------------------------------------------------*/
static int
parent_is_acceptable(rpl_parent_t *p)
{
  uint16_t link_metric = parent_link_metric(p);
  uint16_t path_cost = parent_path_cost(p);
  return link_metric <= MAX_LINK_METRIC && path_cost <= MAX_PATH_COST;
}
/*---------------------------------------------------------------------------*/
static int
parent_has_usable_link(rpl_parent_t *p)
{
  uint16_t link_metric = parent_link_metric(p);
  return link_metric <= MAX_LINK_METRIC;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Compute the composite OF-TAU path metric for a parent.
 *        Updates p->path_erp and p->path_metric.
 *
 * \param p The parent to evaluate.
 */
static void
compute_path_metric(rpl_parent_t *p)
{
  uint16_t local_erp;
  uint16_t etx;
  uint16_t rssi;
  int32_t score;

  if(p == NULL) {
    return;
  }

  /* Step 1: Calculate path ERP (Min-Max: take the worst along the path) */
  local_erp = calculate_erp(p);
  p->path_erp = (p->dio_erp > local_erp) ? p->dio_erp : local_erp;

  /* Step 2: Get link-level metrics */
  etx = parent_link_metric(p);
  rssi = get_parent_rssi(p);

  /* Step 3: Compute combined score
   * score = ALPHA_W * etx + BETA_W * (path_erp + rssi)
   * Note: Larger ERP and larger RSSI magnitude mean worse link quality.
   *       Adding them correctly penalizes bad links and increases the score.
   */
  score = (int32_t)RPL_OF_TAU_ALPHA_W * (int32_t)etx
        + (int32_t)RPL_OF_TAU_BETA_W * ((int32_t)p->path_erp + (int32_t)rssi);

  /* Soft Mobility Penalty: If the parent is mobile, add a massive penalty 
   * so fixed infrastructure is aggressively preferred, but not strictly forced 
   * (Allows fallback to excellent mobile links if fixed is completely dead) */
  if(!p->is_fixed) {
    score += 500;
  }

  /* Clamp to uint16_t range */
  if(score < 0) {
    p->path_metric = 0;
  } else if(score > 0xffff) {
    p->path_metric = 0xffff;
  } else {
    p->path_metric = (uint16_t)score;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief OF-TAU best parent selection with mobility awareness.
 *
 *   1. Reject unacceptable parents
 *   2. Priority to fixed nodes over mobile nodes
 *   3. Compare composite path_metric with hysteresis
 *
 * \param p1 First parent candidate
 * \param p2 Second parent candidate
 * \return The better parent, or NULL if neither is acceptable.
 */
static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  rpl_dag_t *dag;
  int p1_is_acceptable;
  int p2_is_acceptable;

  p1_is_acceptable = p1 != NULL && parent_is_acceptable(p1);
  p2_is_acceptable = p2 != NULL && parent_is_acceptable(p2);

  if(!p1_is_acceptable) {
    return p2_is_acceptable ? p2 : NULL;
  }
  if(!p2_is_acceptable) {
    return p1_is_acceptable ? p1 : NULL;
  }

  dag = p1->dag;

  /* Compute path metrics for both candidates */
  compute_path_metric(p1);
  compute_path_metric(p2);

  printf("RPL: OF-TAU comparing p1(metric=%u,fixed=%u,erp=%u) "
         "vs p2(metric=%u,fixed=%u,erp=%u)\n",
         p1->path_metric, p1->is_fixed, p1->path_erp,
         p2->path_metric, p2->is_fixed, p2->path_erp);

  /*--- Soft Priority Rule is mathematically handled by the +500 penalty above ---*/

  /* Maintain stability of the preferred parent */
  if(p1 == dag->preferred_parent || p2 == dag->preferred_parent) {
    uint16_t dynamic_threshold = RPL_OF_TAU_SWITCH_THRESHOLD;
    
    /* Dynamic Hysteresis: If this node is mobile, significantly lower 
     * the hysteresis so it can flexibly escape dead zones! */
    if(!rims_is_fixed()) {
      dynamic_threshold = 100;
    }

    /* Only switch if the improvement exceeds the hysteresis threshold */
    if(p1->path_metric < p2->path_metric + dynamic_threshold &&
       p1->path_metric > p2->path_metric - dynamic_threshold) {
      return dag->preferred_parent;
    }
  }

  /* Select the parent with the lower path_metric (lower = better) */
  return p1->path_metric < p2->path_metric ? p1 : p2;
}
/*---------------------------------------------------------------------------*/
static rpl_dag_t *
best_dag(rpl_dag_t *d1, rpl_dag_t *d2)
{
  if(d1->grounded != d2->grounded) {
    return d1->grounded ? d1 : d2;
  }

  if(d1->preference != d2->preference) {
    return d1->preference > d2->preference ? d1 : d2;
  }

  return d1->rank < d2->rank ? d1 : d2;
}
/*---------------------------------------------------------------------------*/
#if !RPL_WITH_MC
static void
update_metric_container(rpl_instance_t *instance)
{
  instance->mc.type = RPL_DAG_MC_NONE;
}
#else /* RPL_WITH_MC */
static void
update_metric_container(rpl_instance_t *instance)
{
  rpl_dag_t *dag;
  uint16_t path_cost;
  uint8_t type;

  dag = instance->current_dag;
  if(dag == NULL || !dag->joined) {
    printf("RPL: Cannot update the metric container when not joined\n");
    return;
  }

  if(dag->rank == ROOT_RANK(instance)) {
    instance->mc.type = RPL_DAG_MC;
    instance->mc.flags = 0;
    instance->mc.aggr = RPL_DAG_MC_AGGR_ADDITIVE;
    instance->mc.prec = 0;
    path_cost = dag->rank;
  } else {
    path_cost = parent_path_cost(dag->preferred_parent);
  }

  switch(instance->mc.type) {
    case RPL_DAG_MC_NONE:
      break;
    case RPL_DAG_MC_ETX:
      instance->mc.length = sizeof(instance->mc.obj.etx);
      instance->mc.obj.etx = path_cost;
      break;
    case RPL_DAG_MC_ENERGY:
      instance->mc.length = sizeof(instance->mc.obj.energy);
      if(dag->rank == ROOT_RANK(instance)) {
        type = RPL_DAG_MC_ENERGY_TYPE_MAINS;
      } else {
        type = RPL_DAG_MC_ENERGY_TYPE_BATTERY;
      }
      instance->mc.obj.energy.flags = type << RPL_DAG_MC_ENERGY_TYPE;
      instance->mc.obj.energy.energy_est = path_cost >> 8;
      break;
    default:
      printf("RPL: OF-TAU, non-supported MC %u\n", instance->mc.type);
      break;
  }
}
#endif /* RPL_WITH_MC */
/*---------------------------------------------------------------------------*/
rpl_of_t rpl_of_tau = {
  reset,
#if RPL_WITH_DAO_ACK
  dao_ack_callback,
#endif
  parent_link_metric,
  parent_has_usable_link,
  parent_path_cost,
  rank_via_parent,
  best_parent,
  best_dag,
  update_metric_container,
  RPL_OCP_OF_TAU
};

/** @}*/
