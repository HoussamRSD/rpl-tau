#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/link-stats.h"
#include "net/ipv6/uip-ds6.h"
#include "sys/clock.h"
#include "drl_agent.h"
#include <stdio.h>

#ifndef RPL_DAG_MC_ETX_DIVISOR
#define RPL_DAG_MC_ETX_DIVISOR 128
#endif

/*
 * NOTE
 * ----
 * PE advertisement (E, InvV, Deg, Stb, TauTx) + tau propagation is implemented
 * in rpl-dag.c / rpl-icmp6.c (custom DIO option).
 * This OF only consumes rpl_pe_* and per-parent p->tau_cand / p->pe_*.
 */

/* ====== Link features (0..1000) ====== */
static uint16_t rssi_norm(int16_t rssi_dbm)
{
  /* map [-95..-60] => [0..1000] */
  if(rssi_dbm <= -95) return 0;
  if(rssi_dbm >= -60) return 1000;
  return (uint16_t)(((int32_t)rssi_dbm + 95) * 1000 / 35);
}

static uint16_t etx_norm(uint16_t etx_div128)
{
  /* ETX in divisor units; map [1..8] approx */
  if(etx_div128 <= 1 * RPL_DAG_MC_ETX_DIVISOR) return 1000;
  if(etx_div128 >= 8 * RPL_DAG_MC_ETX_DIVISOR) return 0;
  uint32_t x = etx_div128 - 1 * RPL_DAG_MC_ETX_DIVISOR;
  uint32_t span = 7 * RPL_DAG_MC_ETX_DIVISOR;
  return (uint16_t)(1000 - (x * 1000 / span));
}

static uint16_t link_stab_norm(const struct link_stats *s)
{
  if(s == NULL) return 0;
  /* freshness often 0..16 => map to 0..1000 */
  uint16_t f = s->freshness;
  if(f >= 16) return 1000;
  return (uint16_t)(f * 1000 / 16);
}

static uint16_t hops_norm(rpl_parent_t *p)
{
  if(p == NULL || p->dag == NULL || p->dag->instance == NULL) return 0;
  uint16_t inc = p->dag->instance->min_hoprankinc;
  if(inc == 0) return 0;
  uint16_t hops = p->rank / inc;
  if(hops >= 10) return 0;
  return (uint16_t)(1000 - hops * 100);
}

/* ====== Rank / path cost (monotonic) ====== */
static uint16_t parent_link_metric(rpl_parent_t *p)
{
  const struct link_stats *s = rpl_get_parent_link_stats(p);
  if(s == NULL) return 0xFFFF;

  /* Use ETX as base */
  uint16_t etx = s->etx;

  /* Add penalty if RSSI weak */
  int16_t rssi = (s->rssi == 0) ? -100 : s->rssi;
  if(rssi < -90) etx += 2 * RPL_DAG_MC_ETX_DIVISOR;

  /* Add penalty if stale */
#ifdef link_stats_is_fresh
  if(!link_stats_is_fresh(s)) etx += 2 * RPL_DAG_MC_ETX_DIVISOR;
#endif

  return etx;
}

static int parent_has_usable_link(rpl_parent_t *p)
{
  return parent_link_metric(p) != 0xFFFF;
}

static uint16_t parent_path_cost(rpl_parent_t *p)
{
  if(p == NULL) return 0xFFFF;
  uint32_t sum = (uint32_t)p->rank + (uint32_t)parent_link_metric(p);
  return (sum > 0xFFFF) ? 0xFFFF : (uint16_t)sum;
}

static rpl_rank_t rank_via_parent(rpl_parent_t *p)
{
  if(p == NULL || p->dag == NULL || p->dag->instance == NULL) return INFINITE_RANK;

  uint16_t inc = p->dag->instance->min_hoprankinc;
  uint16_t lm  = parent_link_metric(p);
  if(lm == 0xFFFF) return INFINITE_RANK;

  uint32_t rank_inc = (uint32_t)inc * (uint32_t)lm / RPL_DAG_MC_ETX_DIVISOR;
  uint32_t r = (uint32_t)p->rank + rank_inc;
  if(r > INFINITE_RANK) r = INFINITE_RANK;
  return (rpl_rank_t)r;
}

/* ====== DRL-based best_parent ====== */
static rpl_parent_t *best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  if(p1 == NULL) return p2;
  if(p2 == NULL) return p1;

  if(!parent_has_usable_link(p1)) return parent_has_usable_link(p2) ? p2 : NULL;
  if(!parent_has_usable_link(p2)) return parent_has_usable_link(p1) ? p1 : NULL;

  /* Local Pi */
  uint16_t E_i   = rpl_pe_E;
  uint16_t InvV_i= rpl_pe_InvV;
  uint16_t Deg_i = rpl_pe_Deg;
  uint16_t Stb_i = rpl_pe_Stb;

  /* Link stats for each parent */
  const struct link_stats *s1 = rpl_get_parent_link_stats(p1);
  const struct link_stats *s2 = rpl_get_parent_link_stats(p2);

  uint16_t rssi1 = rssi_norm((s1 && s1->rssi) ? s1->rssi : -100);
  uint16_t rssi2 = rssi_norm((s2 && s2->rssi) ? s2->rssi : -100);

  uint16_t etx1  = etx_norm((s1) ? s1->etx : 8 * RPL_DAG_MC_ETX_DIVISOR);
  uint16_t etx2  = etx_norm((s2) ? s2->etx : 8 * RPL_DAG_MC_ETX_DIVISOR);

  uint16_t ls1   = link_stab_norm(s1);
  uint16_t ls2   = link_stab_norm(s2);

  uint16_t h1    = hops_norm(p1);
  uint16_t h2    = hops_norm(p2);

  /* tau_candidate computed from received PE */
  uint16_t tau1 = p1->tau_cand;
  uint16_t tau2 = p2->tau_cand;

  int32_t q1 = drl_score_parent(p1, tau1, rssi1, etx1, ls1, h1, E_i, InvV_i, Deg_i, Stb_i);
  int32_t q2 = drl_score_parent(p2, tau2, rssi2, etx2, ls2, h2, E_i, InvV_i, Deg_i, Stb_i);

  return (q1 >= q2) ? p1 : p2;
}

static rpl_dag_t *best_dag(rpl_dag_t *d1, rpl_dag_t *d2)
{
  if(d1->grounded != d2->grounded) return d1->grounded ? d1 : d2;
  if(d1->preference != d2->preference) return d1->preference > d2->preference ? d1 : d2;
  return d1->rank < d2->rank ? d1 : d2;
}

/* update_metric_container: refresh metrics before advertising / decision */
static void update_metric_container(rpl_instance_t *instance)
{
  /* We rely on our custom PE option; keep the standard Metric Container disabled */
  instance->mc.type = RPL_DAG_MC_NONE;
  instance->mc.flags = 0;
  instance->mc.aggr = RPL_DAG_MC_AGGR_ADDITIVE;
  instance->mc.prec = 0;
  instance->mc.length = 0;

  /* Refresh local PE values used by the DRL state */
  rpl_pe_update_local(instance);
}

static void reset(rpl_dag_t *dag)
{
  (void)dag;
  drl_init();
}

rpl_of_t rpl_of_drl = {
  reset,
#if RPL_WITH_DAO_ACK
  NULL,
#endif
  parent_link_metric,
  parent_has_usable_link,
  parent_path_cost,
  rank_via_parent,
  best_parent,
  best_dag,
  update_metric_container,
  RPL_OCP_DRL
};

