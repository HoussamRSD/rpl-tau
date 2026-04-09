/*
 * Copyright (c) 2024, Custom OF1 implementation.
 * All rights reserved.
 *
 * Objective Function: OF1 - Best Effort (Connectivité prioritaire)
 * Version modifiée pour accepter les liens faibles si nécessaire.
 */

#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/nbr-table.h"
#include "net/link-stats.h"
#include "sys/clock.h"

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

/* Sécurité de compilation pour le diviseur ETX */
#ifndef RPL_DAG_MC_ETX_DIVISOR
#define RPL_DAG_MC_ETX_DIVISOR 128
#endif

/*---------------------------------------------------------------------------*/
/* --- CONFIGURATION OF1 (Assouplie) --- */
/*---------------------------------------------------------------------------*/

/* 1. Poids ajustés : On réduit l'impact de l'Age pour pénaliser moins */
#ifdef RPL_OF1_CONF_WEIGHT_RSSI
#define WEIGHT_RSSI   RPL_OF1_CONF_WEIGHT_RSSI
#else
#define WEIGHT_RSSI   400 /* Augmenté (40%) */
#endif

#ifdef RPL_OF1_CONF_WEIGHT_ETX
#define WEIGHT_ETX    RPL_OF1_CONF_WEIGHT_ETX
#else
#define WEIGHT_ETX    500 /* Augmenté (50%) - Fiabilité prioritaire */
#endif

#ifdef RPL_OF1_CONF_WEIGHT_AGE
#define WEIGHT_AGE    RPL_OF1_CONF_WEIGHT_AGE
#else
#define WEIGHT_AGE    100 /* REDUIT (10%) : L'âge bloque moins les nouveaux liens */
#endif

/* Bornes RSSI (dBm) - Toujours utilisées pour le score, mais pas pour le rejet */
#define RSSI_MIN      -100 /* Élargi pour capter les signaux très faibles */
#define RSSI_MAX      -30

/* 2. Référence Age réduite : Le nœud atteint le score max plus vite */
#define AGE_REF_SECONDS 15 /* Au lieu de 60 : mature après 15 secondes seulement */

/* 3. Seuils d'acceptabilité RELÂCHÉS */
/* On tolère jusqu'à 16 transmissions par paquet (lien très mauvais mais existant) */
#define MAX_LINK_METRIC     (16 * RPL_DAG_MC_ETX_DIVISOR) 
/* On tolère un coût de chemin énorme */
#define MAX_PATH_COST       65534 

/* Hystérésis réduit pour faciliter le changement vers un meilleur parent */
#define SCORE_THRESHOLD     20

/*---------------------------------------------------------------------------*/
/* Prototypes internes */
static void reset(rpl_dag_t *dag);
static int parent_is_acceptable(rpl_parent_t *p);

/*---------------------------------------------------------------------------*/
/* Helpers */
static int8_t get_parent_rssi(rpl_parent_t *p) {
  return p->rssi; 
}

static clock_time_t get_parent_age_seconds(rpl_parent_t *p) {
  return (clock_time() - p->first_seen) / CLOCK_SECOND;
}

/*---------------------------------------------------------------------------*/
/* * CALCUL DU SCORE (Toujours sur 1000) */
static uint16_t
calculate_stability_score(rpl_parent_t *p)
{
  int32_t score = 0;
  
  /* 1. RSSI Normalisé */
  int8_t rssi = get_parent_rssi(p);
  int32_t rssi_norm = 0;
  
  if(rssi <= RSSI_MIN) rssi_norm = 0; // Mauvais signal = 0 pts (mais pas rejeté)
  else if(rssi >= RSSI_MAX) rssi_norm = 1000;
  else {
    rssi_norm = (int32_t)(rssi - RSSI_MIN) * 1000 / (RSSI_MAX - RSSI_MIN);
  }

  /* 2. ETX Inversé */
  const struct link_stats *stats = rpl_get_parent_link_stats(p);
  uint16_t etx = (stats != NULL) ? stats->etx : 0xFFFF;
  int32_t etx_inv_norm = 0;

  uint16_t etx_ref = RPL_DAG_MC_ETX_DIVISOR; // ETX 1.0
  
  if(etx < etx_ref) etx = etx_ref; 
  
  /* Formule : Plus ETX est grand, plus le score baisse, mais ne devient jamais négatif */
  etx_inv_norm = ((int32_t)etx_ref * 1000) / etx;
  
  /* 3. Age Normalisé */
  clock_time_t age_sec = get_parent_age_seconds(p);
  int32_t age_norm = 0;

  if(age_sec >= AGE_REF_SECONDS) {
    age_norm = 1000;
  } else {
    age_norm = (int32_t)age_sec * 1000 / AGE_REF_SECONDS;
  }

  /* Calcul pondéré */
  score = (WEIGHT_RSSI * rssi_norm) + 
          (WEIGHT_ETX * etx_inv_norm) + 
          (WEIGHT_AGE * age_norm);
          
  score = score / 1000; 

  return (uint16_t)score;
}

/*---------------------------------------------------------------------------*/
static void
reset(rpl_dag_t *dag)
{
  PRINTF("RPL: Reset OF1 Best Effort\n");
}

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
  if(p == NULL || p->dag == NULL || p->dag->instance == NULL) {
    return 0xffff;
  }
  uint16_t base = p->rank;
  /* Rank classique basé sur ETX pour éviter les boucles de routage */
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

  return MAX(MIN((uint32_t)p->rank + min_hoprankinc, 0xffff), path_cost);
}

/*---------------------------------------------------------------------------*/
/* MODIFICATION CRUCIALE : Filtre très permissif */
static int
parent_is_acceptable(rpl_parent_t *p)
{
  uint16_t link_metric = parent_link_metric(p);
  uint16_t path_cost = parent_path_cost(p);

  /* On rejette UNIQUEMENT si le lien est techniquement mort (infini) */
  if (link_metric == 0xFFFF || path_cost == 0xFFFF) {
    return 0;
  }

  /* On accepte tout le reste, même si c'est médiocre */
  return 1;
}

/*---------------------------------------------------------------------------*/
static int
parent_has_usable_link(rpl_parent_t *p)
{
  return parent_is_acceptable(p);
}

/*---------------------------------------------------------------------------*/
static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  rpl_dag_t *dag;
  uint16_t score_p1, score_p2;
  int p1_ok, p2_ok;

  p1_ok = p1 != NULL && parent_is_acceptable(p1);
  p2_ok = p2 != NULL && parent_is_acceptable(p2);

  if(!p1_ok) return p2_ok ? p2 : NULL;
  if(!p2_ok) return p1_ok ? p1 : NULL;

  dag = p1->dag;

  /* On calcule les scores pour départager */
  score_p1 = calculate_stability_score(p1);
  score_p2 = calculate_stability_score(p2);

  /* Si p1 est le parent actuel préféré */
  if(p1 == dag->preferred_parent) {
    /* On garde p1 sauf si p2 est VRAIMENT meilleur (+seuil) */
    if(score_p2 > score_p1 + SCORE_THRESHOLD) {
      return p2;
    }
    return p1;
  }

  /* Si p2 est le parent actuel préféré */
  if(p2 == dag->preferred_parent) {
    if(score_p1 > score_p2 + SCORE_THRESHOLD) {
      return p1;
    }
    return p2;
  }

  /* Si égalité de score, on prend le chemin le plus court (ETX) */
  if(score_p1 == score_p2) {
      if(parent_path_cost(p1) != parent_path_cost(p2)) {
          return parent_path_cost(p1) < parent_path_cost(p2) ? p1 : p2;
      }
  }

  /* Sinon, celui avec le meilleur score gagne */
  return score_p1 > score_p2 ? p1 : p2;
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
