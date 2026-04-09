/*
 * Copyright (c) 2026, Improved Stability OF implementation.
 * Basé sur l'État de l'Art : Hybrid ETX/RSSI avec Pénalités.
 */

#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/nbr-table.h"
#include "net/link-stats.h"

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/* --- CORRECTION DE L'ERREUR DE COMPILATION --- */
/* Sur Z1/Contiki, cette constante manque parfois. On la force à 128. */
#ifndef RPL_DAG_MC_ETX_DIVISOR
#define RPL_DAG_MC_ETX_DIVISOR 128
#endif

/* --- PARAMÈTRES D'OPTIMISATION --- */

/* SEUIL DE QUALITÉ (RSSI Threshold) */
#define RSSI_THRESHOLD -80 

/* PÉNALITÉ (Penalty) */
#define RSSI_PENALTY   (2 * RPL_DAG_MC_ETX_DIVISOR)

/* HYSTÉRÉSIS (Switch Threshold) */
#define PARENT_SWITCH_THRESHOLD 192 

static void reset(rpl_dag_t *dag);
static int parent_is_acceptable(rpl_parent_t *p);

/*---------------------------------------------------------------------------*/
static void
reset(rpl_dag_t *dag)
{
  PRINTF("RPL: Reset Stability OF\n");
}

/*---------------------------------------------------------------------------*/
/* Helpers */
static int8_t get_parent_rssi(rpl_parent_t *p) {
  if(p == NULL) return -100;
  return p->rssi; 
}

/*---------------------------------------------------------------------------*/
/* CALCUL DU COÛT DU LIEN (Link Metric) */
static uint16_t
parent_link_metric(rpl_parent_t *p)
{
  const struct link_stats *stats = rpl_get_parent_link_stats(p);
  uint16_t etx_value;
  int8_t rssi_value;
  uint16_t penalty = 0;

  /* 1. Base : ETX */
  if(stats != NULL) {
    etx_value = stats->etx;
  } else {
    etx_value = 0xffff; 
  }

  /* 2. PÉNALITÉ RSSI */
  rssi_value = get_parent_rssi(p);

  /* Si le signal est trop faible (Zone Grise), on pénalise le lien */
  if(rssi_value < RSSI_THRESHOLD && rssi_value != 0) {
      uint16_t diff = (uint16_t)(RSSI_THRESHOLD - rssi_value);
      penalty = diff * (RPL_DAG_MC_ETX_DIVISOR / 4); 
      
      if(penalty > 5 * RPL_DAG_MC_ETX_DIVISOR) {
          penalty = 5 * RPL_DAG_MC_ETX_DIVISOR;
      }
  }

  return etx_value + penalty;
}

/*---------------------------------------------------------------------------*/
/* CALCUL DU COÛT TOTAL DU CHEMIN (Path Cost) */
static uint16_t
parent_path_cost(rpl_parent_t *p)
{
  if(p == NULL || p->dag == NULL || p->dag->instance == NULL) {
    return 0xffff;
  }

  uint16_t base = p->rank;
  uint16_t link_cost = parent_link_metric(p);
  
  return MIN((uint32_t)base + link_cost, 0xffff);
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

  return MAX(MIN((uint32_t)p->rank + min_hoprankinc, 0xffff), path_cost);
}

/*---------------------------------------------------------------------------*/
/* FILTRE D'ACCEPTABILITÉ */
static int
parent_is_acceptable(rpl_parent_t *p)
{
  uint16_t link_metric = parent_link_metric(p);
  
  if (link_metric == 0xFFFF) {
    return 0;
  }
  
  /* Rejeter les signaux < -95 dBm pour la stabilité */
  if(get_parent_rssi(p) < -95 && get_parent_rssi(p) != 0) {
      return 0;
  }

  return 1;
}

static int
parent_has_usable_link(rpl_parent_t *p)
{
  return parent_is_acceptable(p);
}

/*---------------------------------------------------------------------------*/
/* SÉLECTION DU MEILLEUR PARENT */
static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  rpl_dag_t *dag;
  uint16_t p1_cost;
  uint16_t p2_cost;
  int p1_ok, p2_ok;

  p1_ok = p1 != NULL && parent_is_acceptable(p1);
  p2_ok = p2 != NULL && parent_is_acceptable(p2);

  if(!p1_ok) return p2_ok ? p2 : NULL;
  if(!p2_ok) return p1_ok ? p1 : NULL;

  dag = p1->dag;

  p1_cost = parent_path_cost(p1);
  p2_cost = parent_path_cost(p2);

  /* Hystérésis pour la stabilité */
  if(p1 == dag->preferred_parent) {
    if(p2_cost < p1_cost && (p1_cost - p2_cost) > PARENT_SWITCH_THRESHOLD) {
      return p2; 
    }
    return p1; 
  }

  if(p2 == dag->preferred_parent) {
    if(p1_cost < p2_cost && (p2_cost - p1_cost) > PARENT_SWITCH_THRESHOLD) {
      return p1; 
    }
    return p2; 
  }

  return p1_cost < p2_cost ? p1 : p2;
}

/*---------------------------------------------------------------------------*/
static rpl_dag_t *
best_dag(rpl_dag_t *d1, rpl_dag_t *d2)
{
  if(d1->grounded != d2->grounded) return d1->grounded ? d1 : d2;
  if(d1->preference != d2->preference) return d1->preference > d2->preference ? d1 : d2;
  return d1->rank < d2->rank ? d1 : d2;
}

/*---------------------------------------------------------------------------*/
static void
update_metric_container(rpl_instance_t *instance)
{
  rpl_dag_t *dag = instance->current_dag;
  uint16_t path_cost;

  if(dag == NULL || !dag->joined) return;

  if(dag->rank == ROOT_RANK(instance)) {
    instance->mc.type = RPL_DAG_MC_ETX;
    instance->mc.flags = 0;
    instance->mc.aggr = RPL_DAG_MC_AGGR_ADDITIVE;
    instance->mc.prec = 0;
    path_cost = dag->rank;
  } else {
    path_cost = parent_path_cost(dag->preferred_parent);
  }

  if(instance->mc.type == RPL_DAG_MC_ETX) {
     instance->mc.length = sizeof(instance->mc.obj.etx);
     instance->mc.obj.etx = path_cost;
  }
}

/*---------------------------------------------------------------------------*/
rpl_of_t rpl_of1 = {
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
  RPL_OCP_OF1
};
