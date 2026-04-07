#include "contiki.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/link-stats.h"
#include <limits.h>

/* Si ton port n'a pas ça, on fixe */
#ifndef RPL_DAG_MC_ETX_DIVISOR
#define RPL_DAG_MC_ETX_DIVISOR 128
#endif

/* Un seuil simple (à ajuster) */
#ifndef RPL_OF1_MAX_ETX
#define RPL_OF1_MAX_ETX (8 * RPL_DAG_MC_ETX_DIVISOR) /* ETX > 8 => lien mauvais */
#endif

/* ----------------- Hooks DRL (à remplacer par ton module) ----------------- */
/* Score plus grand = meilleur parent */
__attribute__((weak)) int32_t rpl_drl_score_parent(const rpl_parent_t *p)
{
  /* Fallback simple si ton DRL n’est pas encore branché :
   * - préfère tau_cand élevé
   * - préfère ETX faible
   */
  const struct link_stats *ls = rpl_get_parent_link_stats((rpl_parent_t *)p);
  uint16_t etx = (ls != NULL) ? ls->etx : 0xFFFF;

  /* tau_cand est 0..1000 chez toi */
  int32_t score = (int32_t)p->tau_cand * 1000 - (int32_t)etx;
  return score;
}
/* ------------------------------------------------------------------------- */

static void
reset(rpl_dag_t *dag)
{
  (void)dag;
}

static uint16_t
parent_link_metric(rpl_parent_t *p)
{
  const struct link_stats *ls = rpl_get_parent_link_stats(p);
  if(ls == NULL) {
    return 0xFFFF;
  }
  return ls->etx; /* déjà en unités RPL_DAG_MC_ETX_DIVISOR */
}

static int
parent_has_usable_link(rpl_parent_t *p)
{
  const struct link_stats *ls = rpl_get_parent_link_stats(p);
  if(ls == NULL) {
    return 0;
  }
  return ls->etx > 0 && ls->etx <= RPL_OF1_MAX_ETX;
}

static uint16_t
parent_path_cost(rpl_parent_t *p)
{
  /* Coût “upward” simple : ETX du lien + (optionnel) pénalité tau */
  uint16_t etx = parent_link_metric(p);
  if(etx == 0xFFFF) return 0xFFFF;

  /* tau_cand: plus petit ou plus grand selon ton interprétation.
     Chez toi, tau_cand est “qualité” 0..1000 (plus grand = mieux).
     Donc on convertit en pénalité: (1000 - tau_cand). */
  uint16_t tau_pen = (p->tau_cand > 1000) ? 0 : (uint16_t)(1000 - p->tau_cand);

  /* pondération légère */
  return etx + (tau_pen / 10);
}

static rpl_rank_t
rank_via_parent(rpl_parent_t *p)
{
  if(p == NULL || p->dag == NULL || p->dag->instance == NULL) {
    return INFINITE_RANK;
  }

  rpl_instance_t *instance = p->dag->instance;
  uint16_t etx = parent_link_metric(p);
  if(etx == 0xFFFF) {
    return INFINITE_RANK;
  }

  /* Rank increment basé ETX (comme MRHOF “spirit”), garanti >= min_hoprankinc */
  uint32_t inc = ((uint32_t)instance->min_hoprankinc * (uint32_t)etx) / RPL_DAG_MC_ETX_DIVISOR;
  if(inc < instance->min_hoprankinc) {
    inc = instance->min_hoprankinc;
  }

  /* Respect max_rankinc si défini */
  if(instance->max_rankinc != 0 && inc > instance->max_rankinc) {
    inc = instance->max_rankinc;
  }

  /* Rank monotone */
  if(p->rank == INFINITE_RANK) {
    return INFINITE_RANK;
  }
  if((uint32_t)p->rank + inc >= 0xFFFF) {
    return INFINITE_RANK;
  }
  return (rpl_rank_t)(p->rank + inc);
}

static rpl_parent_t *
best_parent(rpl_parent_t *p1, rpl_parent_t *p2)
{
  if(p1 == NULL) return p2;
  if(p2 == NULL) return p1;

  /* Écarter les liens non utilisables */
  if(!parent_has_usable_link(p1)) return p2;
  if(!parent_has_usable_link(p2)) return p1;

  /* Décision DRL */
  int32_t s1 = rpl_drl_score_parent(p1);
  int32_t s2 = rpl_drl_score_parent(p2);

  if(s2 > s1) return p2;
  if(s1 > s2) return p1;

  /* Tie-breaker : plus petit coût de chemin */
  uint16_t c1 = parent_path_cost(p1);
  uint16_t c2 = parent_path_cost(p2);
  return (c2 < c1) ? p2 : p1;
}

static rpl_dag_t *
best_dag(rpl_dag_t *d1, rpl_dag_t *d2)
{
  if(d1 == NULL) return d2;
  if(d2 == NULL) return d1;

  /* Classique : grounded > preference > rank */
  if(d1->grounded != d2->grounded) {
    return d1->grounded ? d1 : d2;
  }
  if(d1->preference != d2->preference) {
    return d1->preference > d2->preference ? d1 : d2;
  }
  return d1->rank <= d2->rank ? d1 : d2;
}

static void
update_metric_container(rpl_instance_t *instance)
{
#if RPL_WITH_MC
  /* Option 1 : publier ETX */
  instance->mc.type = RPL_DAG_MC_ETX;
  /* La valeur est calculée côté DIO (souvent via parent_link_metric) */
#else
  (void)instance;
#endif
}

/* --------- Export OF --------- */
rpl_of_t rpl_of1 = {
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
  .ocp = RPL_OCP_OF1 /* 0xF1 */
};

