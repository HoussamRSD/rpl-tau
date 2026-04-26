/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 * Logic for Directed Acyclic Graphs in RPL.
 *
 * \author Joakim Eriksson <joakime@sics.se>, Nicolas Tsiftes <nvt@sics.se>
 * Contributors: George Oikonomou <oikonomou@users.sourceforge.net> (multicast)
 */

/**
 * \addtogroup uip6
 * @{
 */

#include "contiki.h"
#include "net/link-stats.h"
#include "net/rpl/rpl-private.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-nd6.h"
#include "net/ipv6/uip-ds6-nbr.h"
#include "net/nbr-table.h"
#include "net/ipv6/multicast/uip-mcast6.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "sys/ctimer.h"
#include "rpl-timers.c"
#include <limits.h>
#include <string.h>
#include "net/packetbuf.h"
#include "sys/energest.h"
#include "net/rpl/rpl-rl-agent.h"



#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"
#include "net/ipv6/uip-ds6.h"

/* ===================================================================
 *  OF-TAU custom metrics (Strict)
 *
 *  DIO_u = { PE_u , τ_u }    PE_u = [ τ_u ]
 *
 *  τ_cand = (w_etx*ETX_n + w_rssi*RSSI_n + w_tau*τ_u) / W_SUM
 *
 *  τ_local (diffusé DIO) = f(τ_cand_path, Deg, NPC, RE, QL)
 *   RE = Residual Energy  (état local → utile aux enfants)
 *   QL = Queue Load       (état local → utile aux enfants)
 * =================================================================== */

/* ===== Weights (override via project-conf.h) ===== */
#ifndef W_PATH
#define W_PATH 5
#endif

/* ===== Constantes énergie résiduelle (override via project-conf.h) ===== */
/* Capacité initiale de la batterie simulée (en milliJoules) */
#ifndef RPL_ENERGY_INIT_MJ
#define RPL_ENERGY_INIT_MJ  3000UL  /* 3 Joules = batterie AA typique simulée */
#endif
/* Tension d'alimentation (en milliVolts) */
#ifndef RPL_ENERGY_VOLTAGE_MV
#define RPL_ENERGY_VOLTAGE_MV  3300UL
#endif
/* Courants de consommation (en µA) */
#ifndef RPL_I_CPU_UA
#define RPL_I_CPU_UA     1800UL
#endif
#ifndef RPL_I_LPM_UA
#define RPL_I_LPM_UA       55UL
#endif
#ifndef RPL_I_TX_UA
#define RPL_I_TX_UA      17400UL
#endif
#ifndef RPL_I_RX_UA
#define RPL_I_RX_UA      19700UL
#endif

/* ===== Constante taille maximale de file (override via project-conf.h) ===== */
#ifndef RPL_QUEUE_MAX_PACKETS
#define RPL_QUEUE_MAX_PACKETS  8   /* Taille max de la file CSMA par défaut Contiki */
#endif

/* ===== Globals advertised in DIO ===== */
uint16_t rpl_pe_Tau = 0;

static uint16_t parent_switches = 0;
static uint16_t clamp1000(uint16_t v) { return v > 1000 ? 1000 : v; }

/* We use default_instance (defined later in this file) */
extern rpl_instance_t *default_instance;


#ifndef LINK_STATS_ETX_DIVISOR
#define LINK_STATS_ETX_DIVISOR 128
#endif

/*---------------------------------------------------------------------------*/
/**
 * \brief Calcule le degré de connectivité normalisé (0-1000).
 * 
 * Représente la densité du réseau autour du nœud en comptant ses parents/voisins valides.
 * L'échelle sature à 20 voisins (Score 1000). Favorise les nœuds très connectés qui offrent
 * une forte redondance radio.
 */
static uint16_t degree_norm(void)
{
  uint16_t count = 0;
  rpl_parent_t *p;
  for(p = nbr_table_head(rpl_parents); p != NULL; p = nbr_table_next(rpl_parents, p)) {
    if(p->dag != NULL && p->rank != INFINITE_RANK) count++;
  }
  if(count >= 20) return 1000;
  return (uint16_t)((uint32_t)count * 1000UL / 20UL);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Calcule la valeur normalisée (0-1000) de l'instabilité du nœud (NPC)
 * 
 * Le Nombre de Changements de Parent (NPC) sert de pénalité propagée dans 
 * l'arbre DODAG. Plus un nœud change souvent de parent, plus il devient
 * "instable" et indésirable pour ses propres enfants.
 * 
 * Cette fonction convertit le compteur brut en un score sur 1000 afin de
 * l'intégrer dans la fonction de coût linéaire `tau_cand`.
 */
static uint16_t npc_norm(void)
{
  if(parent_switches >= 25) return 1000;
  return (uint16_t)(parent_switches * 40);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Calcule l'énergie consommée depuis le démarrage (en nanoJoules).
 *
 * Basée sur les compteurs Energest (temps CPU, LPM, TX, RX) et les
 * courants de consommation configurés.
 */
#if ENERGEST_CONF_ON
static uint64_t rpl_energy_used_nJ(void)
{
  energest_flush();
  uint64_t nJ = 0;
  nJ += ((uint64_t)energest_type_time(ENERGEST_TYPE_CPU)     * (uint64_t)RPL_I_CPU_UA * (uint64_t)RPL_ENERGY_VOLTAGE_MV) / (uint64_t)RTIMER_SECOND;
  nJ += ((uint64_t)energest_type_time(ENERGEST_TYPE_LPM)     * (uint64_t)RPL_I_LPM_UA * (uint64_t)RPL_ENERGY_VOLTAGE_MV) / (uint64_t)RTIMER_SECOND;
  nJ += ((uint64_t)energest_type_time(ENERGEST_TYPE_TRANSMIT) * (uint64_t)RPL_I_TX_UA  * (uint64_t)RPL_ENERGY_VOLTAGE_MV) / (uint64_t)RTIMER_SECOND;
  nJ += ((uint64_t)energest_type_time(ENERGEST_TYPE_LISTEN)   * (uint64_t)RPL_I_RX_UA  * (uint64_t)RPL_ENERGY_VOLTAGE_MV) / (uint64_t)RTIMER_SECOND;
  return nJ;
}
#endif /* ENERGEST_CONF_ON */

/*---------------------------------------------------------------------------*/
/**
 * \brief Calcule l'énergie résiduelle normalisée (0-1000).
 *
 * Compare l'énergie consommée à la capacité maximale de la batterie simulée
 * (`RPL_ENERGY_INIT_MJ`).
 * - Score = 1000 : Batterie pleine.
 * - Score = 0    : Batterie vide ou épuisée.
 *
 * Cet état LOCAL est diffusé dans les DIOs afin que les enfants puissent
 * éviter de choisir un parent sur le point de tomber en panne d'énergie.
 */
static uint16_t residual_energy_norm(void)
{
#if !ENERGEST_CONF_ON
  return 1000; /* Si Energest désactivé, on suppose batterie pleine */
#else
  const uint64_t init_nJ = (uint64_t)RPL_ENERGY_INIT_MJ * 1000000ULL;
  uint64_t used = rpl_energy_used_nJ();
  if(used >= init_nJ) return 0;
  uint64_t n = ((init_nJ - used) * 1000ULL) / init_nJ;
  return (uint16_t)(n > 1000 ? 1000 : n);
#endif
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Calcule la charge de la file d'attente normalisée (0-1000).
 *
 * Mesure le nombre de paquets en attente dans la file CSMA locale.
 * - Score = 1000 : File vide (nœud disponible, pas de congestion).
 * - Score = 0    : File pleine (nœud saturé, risque de pertes).
 *
 * Cet état LOCAL est diffusé dans les DIOs pour que les enfants puissent
 * éviter les parents congestionnnés.
 */
static uint16_t queue_load_norm(void)
{
  /* packetqueue_len() renvoie le nombre réel de paquets en attente */
  extern int packetqueue_len(void *q);
  extern void *csma_queue; /* file CSMA globale */

  /* Si l'API n'est pas disponible au link, on retourne 1000 (neutre/optimal) */
#if defined(CSMA_MAX_PACKET_QUEUES) || defined(RPL_QUEUE_MAX_PACKETS)
  /* Fallback simple : queuebuf occupancy via packetbuf */
  uint8_t used = queuebuf_numfree() >= 0
                 ? (uint8_t)(RPL_QUEUE_MAX_PACKETS - (uint8_t)MIN(queuebuf_numfree(), RPL_QUEUE_MAX_PACKETS))
                 : 0;
  if(used >= RPL_QUEUE_MAX_PACKETS) return 0;   /* file pleine */
  return (uint16_t)(((uint32_t)(RPL_QUEUE_MAX_PACKETS - used) * 1000UL) / (uint32_t)RPL_QUEUE_MAX_PACKETS);
#else
  return 1000; /* inconnu → neutre */
#endif
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Calcule la valeur normalisée (0-1000) de l'ETX (Expected Transmission Count).
 * 
 * Repose sur les statistiques MAC natives (link-stats).
 * - Score = 1000 : Lien parfait (ETX = 1.0, 100% de succès).
 * - Score = 0 : Lien exécrable (ETX >= 8.0, 8 tentatives requises).
 */
uint16_t rpl_etx_norm(rpl_parent_t *p)
{
  if(p == NULL) return 0;
  const struct link_stats *ls = rpl_get_parent_link_stats(p);
  if(ls == NULL || ls->etx == 0) return 500;
  uint16_t etx = ls->etx;
  if(etx <= LINK_STATS_ETX_DIVISOR) return 1000;
  if(etx >= 8 * LINK_STATS_ETX_DIVISOR) return 0;
  uint32_t range = (8 * LINK_STATS_ETX_DIVISOR) - LINK_STATS_ETX_DIVISOR;
  return (uint16_t)(1000UL - ((etx - LINK_STATS_ETX_DIVISOR) * 1000UL / range));
}

#ifndef RPL_RSSI_MIN
#define RPL_RSSI_MIN  (-100)
#endif
#ifndef RPL_RSSI_MAX
#define RPL_RSSI_MAX  (-40)
#endif

/*---------------------------------------------------------------------------*/
/**
 * \brief Calcule la valeur normalisée (0-1000) du Signal Radio RSSI (Entrant).
 * 
 * Extrait la puissance du signal depuis link-stats (sur la base des ACKs reçus).
 * - Score = 1000 : Signal excellent (>= -40 dBm).
 * - Score = 0 : Signal très faible (<= -100 dBm).
 */
uint16_t rpl_rssi_norm(rpl_parent_t *p)
{
  if(p == NULL) return 0;
  const struct link_stats *ls = rpl_get_parent_link_stats(p);
  if(ls == NULL) return 500;
  int16_t r = ls->rssi;
  if(r <= RPL_RSSI_MIN) return 0;
  if(r >= RPL_RSSI_MAX) return 1000;
  return (uint16_t)(((int32_t)(r - RPL_RSSI_MIN) * 1000L) / (int32_t)(RPL_RSSI_MAX - RPL_RSSI_MIN));
}

/*---------------------------------------------------------------------------*/
/**
 * \brief MOTEUR OF-TAU : Détermine le Score Global d'un Parent Candidat
 * 
 * Calcule la désirabilité d'un voisin à devenir le "Preferred Parent" en réalisant
 * une somme pondérée linéaire de ses capacités nodales (RE, QL, Deg, NPC) recues par DIO
 * et des capacités locales du lien physique (ETX, RSSI).
 * 
 * @return Valeur `tau_cand` sur une échelle de 0 à 1000 (1000 étant le parent idéal).
 */
uint16_t rpl_tau_compute_cand(uint16_t ETX_n, uint16_t RSSI_n, uint16_t tau_parent)
{
  ETX_n = clamp1000(ETX_n);
  RSSI_n = clamp1000(RSSI_n);
  tau_parent = clamp1000(tau_parent);

  uint32_t wsum = (W_ETX + W_RSSI + W_TAU);
  uint32_t num =
    (uint32_t)W_ETX  * ETX_n +
    (uint32_t)W_RSSI * RSSI_n +
    (uint32_t)W_TAU  * tau_parent;

  return (uint16_t)(num / wsum);
}

/* --- Dynamic Period Mechanism for NPC Reset --- */
#ifndef RPL_TAU_NPC_PERIOD
#define RPL_TAU_NPC_PERIOD (300 * CLOCK_SECOND) /* Reset frequency for NPC (3 minutes) */
#endif

/*---------------------------------------------------------------------------*/
/**
 * \brief Amnésie Temporelle pour l'Instabilité
 * 
 * Remet à zéro la pénalité d'instabilité du nœud (NPC). Permet au réseau
 * de ne pas garder une rancune éternelle contre un nœud qui a été instable par le
 * passé mais qui s'est stabilisé.
 */
void rpl_pe_npc_reset(void) { 
  parent_switches = 0; 
}

static struct ctimer npc_reset_timer;
static uint8_t npc_timer_started = 0;

static void handle_npc_reset_timer(void *ptr) {
  rpl_pe_npc_reset();
  /* Reschedule */
  printf("[TAU] NPC Reset (Amnesia) - Timer Rescheduled\n");
  ctimer_set(&npc_reset_timer, RPL_TAU_NPC_PERIOD, handle_npc_reset_timer, NULL);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief Callback interne appelé lors d'un "Parent Switch"
 * 
 * S'assure de démarrer le minuteur dynamique d'expiration au premier faux pas.
 */
void rpl_pe_on_parent_switch(void) { 
  parent_switches++;
  /* Start the dynamic period timer on the very first parent switch */
  printf("[TAU] Parent Switch detected. NPC=%u\n", parent_switches);
  if(!npc_timer_started) {
    npc_timer_started = 1;
    printf("[TAU] Starting NPC Reset Timer (Period: %d ticks)\n", RPL_TAU_NPC_PERIOD);
    ctimer_set(&npc_reset_timer, RPL_TAU_NPC_PERIOD, handle_npc_reset_timer, NULL);
  }
}

void rpl_pe_update_local(rpl_instance_t *instance)
{
  rpl_dag_t *dag = instance->current_dag;
  
  if(dag != NULL && dag->rank == ROOT_RANK(instance)) {
    /* Le Sink (Root) n'a pas de parent. Il envoie un Tau de 1000 (route parfaite) */
    rpl_pe_Tau = 1000;
    printf("[TAU] Root generating DIO. Tau=1000\n");
    return;
  }

  /* --- Métriques locales du nœud (décrivent MON état aux enfants) --- */
  uint16_t deg_n = clamp1000(degree_norm());       /* Degré de connectivité      */
  uint16_t npc_n = clamp1000(npc_norm());           /* Instabilité (switches)     */
  uint16_t re_n  = clamp1000(residual_energy_norm()); /* Énergie résiduelle [0..1000] */
  uint16_t ql_n  = clamp1000(queue_load_norm());    /* File vide=1000, pleine=0   */

  if(dag != NULL && dag->preferred_parent != NULL) {
    /*
     * τ_local = somme pondérée de :
     *   W_PATH : score cumulé du chemin vers la racine (via le meilleur parent)
     *   W_DEG  : connectivité locale (bonnes alternatives disponibles)
     *   W_NPC  : stabilité locale (inverse des changements de parent)
     *   W_RE   : énergie résiduelle (je peux continuer à fonctionner longtemps)
     *   W_QL   : charge de la file (je peux absorber du trafic sans congestion)
     *
     * NB : RE et QL décrivent l'état LOCAL du nœud → informent uniquement les
     *      enfants pour qu'ils évitent les parents épuisés ou saturés.
     *      Ils ne participent PAS au calcul de tau_cand (choix du parent).
     */
    uint16_t candidate_path_score = clamp1000(dag->preferred_parent->tau_cand);

    uint32_t wsum = ((uint32_t)W_PATH + W_DEG + W_NPC + W_RE + W_QL);
    uint32_t num =
      (uint32_t)W_PATH * candidate_path_score +
      (uint32_t)W_DEG  * deg_n +
      (uint32_t)W_NPC  * (1000 - npc_n) +   /* pénalité = instabilité */
      (uint32_t)W_RE   * re_n +              /* bonus   = énergie restante */
      (uint32_t)W_QL   * ql_n;              /* bonus   = file disponible  */

    rpl_pe_Tau = clamp1000((uint16_t)(num / wsum));
  } else {
    /* Aucun parent sélectionné : τ basé uniquement sur l'état local */
    uint32_t wsum = ((uint32_t)W_DEG + W_NPC + W_RE + W_QL);
    uint32_t num =
      (uint32_t)W_DEG  * deg_n +
      (uint32_t)W_NPC  * (1000 - npc_n) +
      (uint32_t)W_RE   * re_n +
      (uint32_t)W_QL   * ql_n;
    rpl_pe_Tau = clamp1000((uint16_t)(num / wsum));
  }

  printf("[TAU] SendingDIO To Child with Tau=%u. Deg=%u NPC=%u RE=%u QL=%u \n",
         rpl_pe_Tau, deg_n, npc_n, re_n, ql_n);
}









/* A configurable function called after every RPL parent switch */
#ifdef RPL_CALLBACK_PARENT_SWITCH
void RPL_CALLBACK_PARENT_SWITCH(rpl_parent_t *old, rpl_parent_t *new);
#endif /* RPL_CALLBACK_PARENT_SWITCH */

/*---------------------------------------------------------------------------*/
extern rpl_of_t rpl_of0, rpl_mrhof, rpl_of1, rpl_of_tau;
/*.............................................................................................................................OF1*/
static rpl_of_t * const objective_functions[] = RPL_SUPPORTED_OFS;

/*---------------------------------------------------------------------------*/
/* RPL definitions. */

#ifndef RPL_CONF_GROUNDED
#define RPL_GROUNDED                    0
#else
#define RPL_GROUNDED                    RPL_CONF_GROUNDED
#endif /* !RPL_CONF_GROUNDED */

/*---------------------------------------------------------------------------*/
/* Per-parent RPL information */
NBR_TABLE_GLOBAL(rpl_parent_t, rpl_parents);
/*---------------------------------------------------------------------------*/
/* Allocate instance table. */
rpl_instance_t instance_table[RPL_MAX_INSTANCES];
rpl_instance_t *default_instance;

/*---------------------------------------------------------------------------*/
void
rpl_print_neighbor_list(void)
{
  if(default_instance != NULL && default_instance->current_dag != NULL &&
      default_instance->of != NULL) {
    int curr_dio_interval = default_instance->dio_intcurrent;
    int curr_rank = default_instance->current_dag->rank;
    rpl_parent_t *p = nbr_table_head(rpl_parents);
    clock_time_t clock_now = clock_time();

    PRINTF("RPL: MOP %u OCP %u rank %u dioint %u, nbr count %u\n",
        default_instance->mop, default_instance->of->ocp, curr_rank, curr_dio_interval, uip_ds6_nbr_num());
    while(p != NULL) {
      const struct link_stats *stats = rpl_get_parent_link_stats(p);
      PRINTF("RPL: nbr %3u %5u, %5u => %5u -- %2u %c%c (last tx %u min ago)\n",
          rpl_get_parent_ipaddr(p)->u8[15],
          p->rank,
          rpl_get_parent_link_metric(p),
          rpl_rank_via_parent(p),
          stats != NULL ? stats->freshness : 0,
          link_stats_is_fresh(stats) ? 'f' : ' ',
          p == default_instance->current_dag->preferred_parent ? 'p' : ' ',
          (unsigned)((clock_now - stats->last_tx_time) / (60 * CLOCK_SECOND))
      );
      p = nbr_table_next(rpl_parents, p);
    }
    PRINTF("RPL: end of list\n");
  }
}
/*---------------------------------------------------------------------------*/
uip_ds6_nbr_t *
rpl_get_nbr(rpl_parent_t *parent)
{
  const linkaddr_t *lladdr = rpl_get_parent_lladdr(parent);
  if(lladdr != NULL) {
    return nbr_table_get_from_lladdr(ds6_neighbors, lladdr);
  } else {
    return NULL;
  }
}
/*---------------------------------------------------------------------------*/
static void
nbr_callback(void *ptr)
{
  rpl_remove_parent(ptr);
}

void
rpl_dag_init(void)
{
  PRINTF("test\n");
  nbr_table_register(rpl_parents, (nbr_table_callback *)nbr_callback);
}
/*---------------------------------------------------------------------------*/
rpl_parent_t *
rpl_get_parent(uip_lladdr_t *addr)
{
  rpl_parent_t *p = nbr_table_get_from_lladdr(rpl_parents, (linkaddr_t *)addr);
  return p;
}
/*---------------------------------------------------------------------------*/
rpl_rank_t
rpl_get_parent_rank(uip_lladdr_t *addr)
{
  rpl_parent_t *p = nbr_table_get_from_lladdr(rpl_parents, (linkaddr_t *)addr);
  if(p != NULL) {
    return p->rank;
  } else {
    return INFINITE_RANK;
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
rpl_get_parent_link_metric(rpl_parent_t *p)
{
  if(p != NULL && p->dag != NULL) {
    rpl_instance_t *instance = p->dag->instance;
    if(instance != NULL && instance->of != NULL && instance->of->parent_link_metric != NULL) {
      return instance->of->parent_link_metric(p);
    }
  }
  return 0xffff;
}
/*---------------------------------------------------------------------------*/
rpl_rank_t
rpl_rank_via_parent(rpl_parent_t *p)
{
  if(p != NULL && p->dag != NULL) {
    rpl_instance_t *instance = p->dag->instance;
    if(instance != NULL && instance->of != NULL && instance->of->rank_via_parent != NULL) {
      return instance->of->rank_via_parent(p);
    }
  }
  return INFINITE_RANK;
}
/*---------------------------------------------------------------------------*/
const linkaddr_t *
rpl_get_parent_lladdr(rpl_parent_t *p)
{
  return nbr_table_get_lladdr(rpl_parents, p);
}
/*---------------------------------------------------------------------------*/
uip_ipaddr_t *
rpl_get_parent_ipaddr(rpl_parent_t *p)
{
  const linkaddr_t *lladdr = rpl_get_parent_lladdr(p);
  return uip_ds6_nbr_ipaddr_from_lladdr((uip_lladdr_t *)lladdr);
}
/*---------------------------------------------------------------------------*/
const struct link_stats *
rpl_get_parent_link_stats(rpl_parent_t *p)
{
  const linkaddr_t *lladdr = rpl_get_parent_lladdr(p);
  return link_stats_from_lladdr(lladdr);
}
/*---------------------------------------------------------------------------*/
int
rpl_parent_is_fresh(rpl_parent_t *p)
{
  const struct link_stats *stats = rpl_get_parent_link_stats(p);
  return link_stats_is_fresh(stats);
}
/*---------------------------------------------------------------------------*/
int
rpl_parent_is_reachable(rpl_parent_t *p) {
  if(p == NULL || p->dag == NULL || p->dag->instance == NULL || p->dag->instance->of == NULL) {
    return 0;
  } else {
#if UIP_ND6_SEND_NS
    uip_ds6_nbr_t *nbr = rpl_get_nbr(p);
    /* Exclude links to a neighbor that is not reachable at a NUD level */
    if(nbr == NULL || nbr->state != NBR_REACHABLE) {
      return 0;
    }
#endif /* UIP_ND6_SEND_NS */
    /* If we don't have fresh link information, assume the parent is reachable. */
    return !rpl_parent_is_fresh(p) || p->dag->instance->of->parent_has_usable_link(p);
  }
}
/*---------------------------------------------------------------------------*/
void
rpl_set_preferred_parent(rpl_dag_t *dag, rpl_parent_t *p)
{
  if(dag != NULL && dag->preferred_parent != p) {
    PRINTF("RPL: rpl_set_preferred_parent ");
    if(p != NULL) {
      PRINT6ADDR(rpl_get_parent_ipaddr(p));
    } else {
      PRINTF("NULL");
    }
    PRINTF(" used to be ");
    if(dag->preferred_parent != NULL) {
      PRINT6ADDR(rpl_get_parent_ipaddr(dag->preferred_parent));
    } else {
      PRINTF("NULL");
    }
    PRINTF("\n");

#ifdef RPL_CALLBACK_PARENT_SWITCH
    RPL_CALLBACK_PARENT_SWITCH(dag->preferred_parent, p);
#endif /* RPL_CALLBACK_PARENT_SWITCH */

    /* Always keep the preferred parent locked, so it remains in the
     * neighbor table. */
    nbr_table_unlock(rpl_parents, dag->preferred_parent);
    nbr_table_lock(rpl_parents, p);

/*changement version 0*/
    /* Count parent changes for stability (ignore initial join) */
    if(dag->preferred_parent != NULL && p != NULL && dag->preferred_parent != p) {
      rpl_pe_on_parent_switch();
      printf("#A Parent Switch!\n");
    }
/*changement version 0*/
    dag->preferred_parent = p;
  }
}
/*---------------------------------------------------------------------------*/
/* Greater-than function for the lollipop counter.                      */
/*---------------------------------------------------------------------------*/
static int
lollipop_greater_than(int a, int b)
{
  /* Check if we are comparing an initial value with an old value */
  if(a > RPL_LOLLIPOP_CIRCULAR_REGION && b <= RPL_LOLLIPOP_CIRCULAR_REGION) {
    return (RPL_LOLLIPOP_MAX_VALUE + 1 + b - a) > RPL_LOLLIPOP_SEQUENCE_WINDOWS;
  }
  /* Otherwise check if a > b and comparable => ok, or
      if they have wrapped and are still comparable */
  return (a > b && (a - b) < RPL_LOLLIPOP_SEQUENCE_WINDOWS) ||
    (a < b && (b - a) > (RPL_LOLLIPOP_CIRCULAR_REGION + 1-
			 RPL_LOLLIPOP_SEQUENCE_WINDOWS));
}
/*---------------------------------------------------------------------------*/
/* Remove DAG parents with a rank that is at least the same as minimum_rank. */
static void
remove_parents(rpl_dag_t *dag, rpl_rank_t minimum_rank)
{
  rpl_parent_t *p;

  PRINTF("RPL: Removing parents (minimum rank %u)\n",
	minimum_rank);

  p = nbr_table_head(rpl_parents);
  while(p != NULL) {
    if(dag == p->dag && p->rank >= minimum_rank) {
      rpl_remove_parent(p);
    }
    p = nbr_table_next(rpl_parents, p);
  }
}
/*---------------------------------------------------------------------------*/
static void
nullify_parents(rpl_dag_t *dag, rpl_rank_t minimum_rank)
{
  rpl_parent_t *p;

  PRINTF("RPL: Nullifying parents (minimum rank %u)\n",
	minimum_rank);

  p = nbr_table_head(rpl_parents);
  while(p != NULL) {
    if(dag == p->dag && p->rank >= minimum_rank) {
      rpl_nullify_parent(p);
    }
    p = nbr_table_next(rpl_parents, p);
  }
}
/*---------------------------------------------------------------------------*/
static int
should_refresh_routes(rpl_instance_t *instance, rpl_dio_t *dio, rpl_parent_t *p)
{
  /* if MOP is set to no downward routes no DAO should be sent */
  if(instance->mop == RPL_MOP_NO_DOWNWARD_ROUTES) {
    return 0;
  }
  /* check if the new DTSN is more recent */
  return p == instance->current_dag->preferred_parent &&
    (lollipop_greater_than(dio->dtsn, p->dtsn));
}
/*---------------------------------------------------------------------------*/
static int
acceptable_rank(rpl_dag_t *dag, rpl_rank_t rank)
{
  return rank != INFINITE_RANK &&
    ((dag->instance->max_rankinc == 0) ||
      DAG_RANK(rank, dag->instance) <= DAG_RANK(dag->min_rank + dag->instance->max_rankinc, dag->instance));
}
/*---------------------------------------------------------------------------*/
static rpl_dag_t *
get_dag(uint8_t instance_id, uip_ipaddr_t *dag_id)
{
  rpl_instance_t *instance;
  rpl_dag_t *dag;
  int i;

  instance = rpl_get_instance(instance_id);
  if(instance == NULL) {
    return NULL;
  }

  for(i = 0; i < RPL_MAX_DAG_PER_INSTANCE; ++i) {
    dag = &instance->dag_table[i];
    if(dag->used && uip_ipaddr_cmp(&dag->dag_id, dag_id)) {
      return dag;
    }
  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_set_root(uint8_t instance_id, uip_ipaddr_t *dag_id)
{
  rpl_dag_t *dag;
  rpl_instance_t *instance;
  uint8_t version;
  int i;

  version = RPL_LOLLIPOP_INIT;
  instance = rpl_get_instance(instance_id);
  if(instance != NULL) {
    for(i = 0; i < RPL_MAX_DAG_PER_INSTANCE; ++i) {
      dag = &instance->dag_table[i];
      if(dag->used) {
        if(uip_ipaddr_cmp(&dag->dag_id, dag_id)) {
          version = dag->version;
          RPL_LOLLIPOP_INCREMENT(version);
        }
        if(dag == dag->instance->current_dag) {
          PRINTF("RPL: Dropping a joined DAG when setting this node as root");
          dag->instance->current_dag = NULL;
        } else {
          PRINTF("RPL: Dropping a DAG when setting this node as root");
        }
        rpl_free_dag(dag);
      }
    }
  }

  dag = rpl_alloc_dag(instance_id, dag_id);
  if(dag == NULL) {
    PRINTF("RPL: Failed to allocate a DAG\n");
    return NULL;
  }

  instance = dag->instance;

  dag->version = version;
  dag->joined = 1;
  dag->grounded = RPL_GROUNDED;
  dag->preference = RPL_PREFERENCE;
  instance->mop = RPL_MOP_DEFAULT;
  instance->of = rpl_find_of(RPL_OF_OCP);
  if(instance->of == NULL) {
    PRINTF("RPL: OF with OCP %u not supported\n", RPL_OF_OCP);
    return NULL;
  }

  rpl_set_preferred_parent(dag, NULL);

  memcpy(&dag->dag_id, dag_id, sizeof(dag->dag_id));

  instance->dio_intdoubl = RPL_DIO_INTERVAL_DOUBLINGS;
  instance->dio_intmin = RPL_DIO_INTERVAL_MIN;
  /* The current interval must differ from the minimum interval in order to
      trigger a DIO timer reset. */
  instance->dio_intcurrent = RPL_DIO_INTERVAL_MIN +
    RPL_DIO_INTERVAL_DOUBLINGS;
  instance->dio_redundancy = RPL_DIO_REDUNDANCY;
  instance->max_rankinc = RPL_MAX_RANKINC;
  instance->min_hoprankinc = RPL_MIN_HOPRANKINC;
  instance->default_lifetime = RPL_DEFAULT_LIFETIME;
  instance->lifetime_unit = RPL_DEFAULT_LIFETIME_UNIT;

  dag->rank = ROOT_RANK(instance);

  if(instance->current_dag != dag && instance->current_dag != NULL) {
    /* Remove routes installed by DAOs. */
    if(RPL_IS_STORING(instance)) {
      rpl_remove_routes(instance->current_dag);
    }

    instance->current_dag->joined = 0;
  }

  instance->current_dag = dag;
  instance->dtsn_out = RPL_LOLLIPOP_INIT;
  instance->of->update_metric_container(instance);
  default_instance = instance;

  PRINTF("RPL: Node set to be a DAG root with DAG ID ");
  PRINT6ADDR(&dag->dag_id);
  PRINTF("\n");

  ANNOTATE("#A root=%u\n", dag->dag_id.u8[sizeof(dag->dag_id) - 1]);

  rpl_reset_dio_timer(instance);

  return dag;
}
/*---------------------------------------------------------------------------*/
int
rpl_repair_root(uint8_t instance_id)
{
  rpl_instance_t *instance;

  instance = rpl_get_instance(instance_id);
  if(instance == NULL ||
      instance->current_dag->rank != ROOT_RANK(instance)) {
    PRINTF("RPL: rpl_repair_root triggered but not root\n");
    return 0;
  }
  RPL_STAT(rpl_stats.root_repairs++);

  RPL_LOLLIPOP_INCREMENT(instance->current_dag->version);
  RPL_LOLLIPOP_INCREMENT(instance->dtsn_out);
  PRINTF("RPL: rpl_repair_root initiating global repair with version %d\n", instance->current_dag->version);
  rpl_reset_dio_timer(instance);

  return 1;
}
/*---------------------------------------------------------------------------*/
static void
set_ip_from_prefix(uip_ipaddr_t *ipaddr, rpl_prefix_t *prefix)
{
  memset(ipaddr, 0, sizeof(uip_ipaddr_t));
  memcpy(ipaddr, &prefix->prefix, (prefix->length + 7) / 8);
  uip_ds6_set_addr_iid(ipaddr, &uip_lladdr);
}
/*---------------------------------------------------------------------------*/
static void
check_prefix(rpl_prefix_t *last_prefix, rpl_prefix_t *new_prefix)
{
  uip_ipaddr_t ipaddr;
  uip_ds6_addr_t *rep;

  if(last_prefix != NULL && new_prefix != NULL &&
      last_prefix->length == new_prefix->length &&
      uip_ipaddr_prefixcmp(&last_prefix->prefix, &new_prefix->prefix, new_prefix->length) &&
      last_prefix->flags == new_prefix->flags) {
    /* Nothing has changed. */
    return;
  }

  if(last_prefix != NULL) {
    set_ip_from_prefix(&ipaddr, last_prefix);
    rep = uip_ds6_addr_lookup(&ipaddr);
    if(rep != NULL) {
      PRINTF("RPL: removing global IP address ");
      PRINT6ADDR(&ipaddr);
      PRINTF("\n");
      uip_ds6_addr_rm(rep);
    }
  }

  if(new_prefix != NULL) {
    set_ip_from_prefix(&ipaddr, new_prefix);
    if(uip_ds6_addr_lookup(&ipaddr) == NULL) {
      PRINTF("RPL: adding global IP address ");
      PRINT6ADDR(&ipaddr);
      PRINTF("\n");
      uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
    }
  }
}
/*---------------------------------------------------------------------------*/
int
rpl_set_prefix(rpl_dag_t *dag, uip_ipaddr_t *prefix, unsigned len)
{
  rpl_prefix_t last_prefix;
  uint8_t last_len = dag->prefix_info.length;

  if(len > 128) {
    return 0;
  }
  if(dag->prefix_info.length != 0) {
    memcpy(&last_prefix, &dag->prefix_info, sizeof(rpl_prefix_t));
  }
  memset(&dag->prefix_info.prefix, 0, sizeof(dag->prefix_info.prefix));
  memcpy(&dag->prefix_info.prefix, prefix, (len + 7) / 8);
  dag->prefix_info.length = len;
  dag->prefix_info.flags = UIP_ND6_RA_FLAG_AUTONOMOUS;
  PRINTF("RPL: Prefix set - will announce this in DIOs\n");
  /* Autoconfigure an address if this node does not already have an address
      with this prefix. Otherwise, update the prefix */
  if(last_len == 0) {
    PRINTF("RPL: rpl_set_prefix - prefix NULL\n");
    check_prefix(NULL, &dag->prefix_info);
  } else {
    PRINTF("RPL: rpl_set_prefix - prefix NON-NULL\n");
    check_prefix(&last_prefix, &dag->prefix_info);
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
int
rpl_set_default_route(rpl_instance_t *instance, uip_ipaddr_t *from)
{
  if(instance->def_route != NULL) {
    PRINTF("RPL: Removing default route through ");
    PRINT6ADDR(&instance->def_route->ipaddr);
    PRINTF("\n");
    uip_ds6_defrt_rm(instance->def_route);
    instance->def_route = NULL;
  }

  if(from != NULL) {
    PRINTF("RPL: Adding default route through ");
    PRINT6ADDR(from);
    PRINTF("\n");
    instance->def_route = uip_ds6_defrt_add(from,
        RPL_DEFAULT_ROUTE_INFINITE_LIFETIME ? 0 : RPL_LIFETIME(instance, instance->default_lifetime));
    if(instance->def_route == NULL) {
      return 0;
    }
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
rpl_instance_t *
rpl_alloc_instance(uint8_t instance_id)
{
  rpl_instance_t *instance, *end;

  for(instance = &instance_table[0], end = instance + RPL_MAX_INSTANCES;
      instance < end; ++instance) {
    if(instance->used == 0) {
      memset(instance, 0, sizeof(*instance));
      instance->instance_id = instance_id;
      instance->def_route = NULL;
      instance->used = 1;
#if RPL_WITH_PROBING
      rpl_schedule_probing(instance);
#endif /* RPL_WITH_PROBING */
      return instance;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_alloc_dag(uint8_t instance_id, uip_ipaddr_t *dag_id)
{
  rpl_dag_t *dag, *end;
  rpl_instance_t *instance;

  instance = rpl_get_instance(instance_id);
  if(instance == NULL) {
    instance = rpl_alloc_instance(instance_id);
    if(instance == NULL) {
      RPL_STAT(rpl_stats.mem_overflows++);
      return NULL;
    }
  }

  for(dag = &instance->dag_table[0], end = dag + RPL_MAX_DAG_PER_INSTANCE; dag < end; ++dag) {
    if(!dag->used) {
      memset(dag, 0, sizeof(*dag));
      dag->used = 1;
      dag->rank = INFINITE_RANK;
      dag->min_rank = INFINITE_RANK;
      dag->instance = instance;
      return dag;
    }
  }

  RPL_STAT(rpl_stats.mem_overflows++);
  return NULL;
}
/*---------------------------------------------------------------------------*/
void
rpl_set_default_instance(rpl_instance_t *instance)
{
  default_instance = instance;
}
/*---------------------------------------------------------------------------*/
rpl_instance_t *
rpl_get_default_instance(void)
{
  return default_instance;
}
/*---------------------------------------------------------------------------*/
void
rpl_free_instance(rpl_instance_t *instance)
{
  rpl_dag_t *dag;
  rpl_dag_t *end;

  PRINTF("RPL: Leaving the instance %u\n", instance->instance_id);

  /* Remove any DAG inside this instance */
  for(dag = &instance->dag_table[0], end = dag + RPL_MAX_DAG_PER_INSTANCE; dag < end; ++dag) {
    if(dag->used) {
      rpl_free_dag(dag);
    }
  }

  rpl_set_default_route(instance, NULL);

#if RPL_WITH_PROBING
  ctimer_stop(&instance->probing_timer);
#endif /* RPL_WITH_PROBING */
  ctimer_stop(&instance->dio_timer);
  ctimer_stop(&instance->dao_timer);
  ctimer_stop(&instance->dao_lifetime_timer);

  if(default_instance == instance) {
    default_instance = NULL;
  }

  instance->used = 0;
}
/*---------------------------------------------------------------------------*/
void
rpl_free_dag(rpl_dag_t *dag)
{
  if(dag->joined) {
    PRINTF("RPL: Leaving the DAG ");
    PRINT6ADDR(&dag->dag_id);
    PRINTF("\n");
    dag->joined = 0;

    /* Remove routes installed by DAOs. */
    if(RPL_IS_STORING(dag->instance)) {
      rpl_remove_routes(dag);
    }

   /* Remove autoconfigured address */
    if((dag->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS)) {
      check_prefix(&dag->prefix_info, NULL);
    }

    remove_parents(dag, 0);
  }
  dag->used = 0;
}
/*---------------------------------------------------------------------------*/
rpl_parent_t *
rpl_add_parent(rpl_dag_t *dag, rpl_dio_t *dio, uip_ipaddr_t *addr)
{
  rpl_parent_t *p = NULL;
  /* Is the parent known by ds6? Drop this request if not.
   * Typically, the parent is added upon receiving a DIO. */
  const uip_lladdr_t *lladdr = uip_ds6_nbr_lladdr_from_ipaddr(addr);

  PRINTF("RPL: rpl_add_parent lladdr %p ", lladdr);
  PRINT6ADDR(addr);
  PRINTF("\n");
  if(lladdr != NULL) {
    /* Add parent in rpl_parents - again this is due to DIO */
    p = nbr_table_add_lladdr(rpl_parents, (linkaddr_t *)lladdr,
                             NBR_TABLE_REASON_RPL_DIO, dio);
    if(p == NULL) {
      PRINTF("RPL: rpl_add_parent p NULL\n");
    } else {
      p->dag = dag;
      p->rank = dio->rank;
      p->dtsn = dio->dtsn;

      /* Copy PE from parent's DIO */
      p->pe_Tau  = dio->pe_Tau;

      /* τ_cand = F(τ_u, P_lien(i,u)) — linear weighted sum */
      p->tau_cand = rpl_tau_compute_cand(
        rpl_etx_norm(p), rpl_rssi_norm(p), p->pe_Tau);

      /* Bootstrap guard: tau_cand can be 0 if PE option not yet received.
       * A zero tau causes parent_has_usable_link() to reject ALL parents.
       * Use a neutral mid-range value until real data arrives. */
      if(p->tau_cand == 0) {
        p->tau_cand = 500;
      }


#if RPL_WITH_MC
      memcpy(&p->mc, &dio->mc, sizeof(p->mc));
#endif /* RPL_WITH_MC */
    }
  }

  return p;
}
/*---------------------------------------------------------------------------*/
static rpl_parent_t *
find_parent_any_dag_any_instance(uip_ipaddr_t *addr)
{
  uip_ds6_nbr_t *ds6_nbr = uip_ds6_nbr_lookup(addr);
  const uip_lladdr_t *lladdr = uip_ds6_nbr_get_ll(ds6_nbr);
  return nbr_table_get_from_lladdr(rpl_parents, (linkaddr_t *)lladdr);
}
/*---------------------------------------------------------------------------*/
rpl_parent_t *
rpl_find_parent(rpl_dag_t *dag, uip_ipaddr_t *addr)
{
  rpl_parent_t *p = find_parent_any_dag_any_instance(addr);
  if(p != NULL && p->dag == dag) {
    return p;
  } else {
    return NULL;
  }
}
/*---------------------------------------------------------------------------*/
static rpl_dag_t *
find_parent_dag(rpl_instance_t *instance, uip_ipaddr_t *addr)
{
  rpl_parent_t *p = find_parent_any_dag_any_instance(addr);
  if(p != NULL) {
    return p->dag;
  } else {
    return NULL;
  }
}
/*---------------------------------------------------------------------------*/
rpl_parent_t *
rpl_find_parent_any_dag(rpl_instance_t *instance, uip_ipaddr_t *addr)
{
  rpl_parent_t *p = find_parent_any_dag_any_instance(addr);
  if(p && p->dag && p->dag->instance == instance) {
    return p;
  } else {
    return NULL;
  }
}
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_select_dag(rpl_instance_t *instance, rpl_parent_t *p)
{
  rpl_parent_t *last_parent;
  rpl_dag_t *dag, *end, *best_dag;
  rpl_rank_t old_rank;

  old_rank = instance->current_dag->rank;
  last_parent = instance->current_dag->preferred_parent;

  best_dag = instance->current_dag;
  if(best_dag->rank != ROOT_RANK(instance)) {
    if(rpl_select_parent(p->dag) != NULL) {
      if(p->dag != best_dag) {
        best_dag = instance->of->best_dag(best_dag, p->dag);
      }
    } else if(p->dag == best_dag) {
      best_dag = NULL;
      for(dag = &instance->dag_table[0], end = dag + RPL_MAX_DAG_PER_INSTANCE; dag < end; ++dag) {
        if(dag->used && dag->preferred_parent != NULL && dag->preferred_parent->rank != INFINITE_RANK) {
          if(best_dag == NULL) {
            best_dag = dag;
          } else {
            best_dag = instance->of->best_dag(best_dag, dag);
          }
        }
      }
    }
  }

  if(best_dag == NULL) {
    /* No parent found: the calling function handle this problem. */
    return NULL;
  }

  if(instance->current_dag != best_dag) {
    /* Remove routes installed by DAOs. */
    if(RPL_IS_STORING(instance)) {
      rpl_remove_routes(instance->current_dag);
    }

    PRINTF("RPL: New preferred DAG: ");
    PRINT6ADDR(&best_dag->dag_id);
    PRINTF("\n");

    if(best_dag->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
      check_prefix(&instance->current_dag->prefix_info, &best_dag->prefix_info);
    } else if(instance->current_dag->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
      check_prefix(&instance->current_dag->prefix_info, NULL);
    }

    best_dag->joined = 1;
    instance->current_dag->joined = 0;
    instance->current_dag = best_dag;
  }

  instance->of->update_metric_container(instance);
  /* Update the DAG rank. */
  best_dag->rank = rpl_rank_via_parent(best_dag->preferred_parent);
  if(last_parent == NULL || best_dag->rank < best_dag->min_rank) {
    /* This is a slight departure from RFC6550: if we had no preferred parent before,
     * reset min_rank. This helps recovering from temporary bad link conditions. */
    best_dag->min_rank = best_dag->rank;
  }

  if(!acceptable_rank(best_dag, best_dag->rank)) {
    PRINTF("RPL: New rank unacceptable!\n");
    rpl_set_preferred_parent(instance->current_dag, NULL);
    if(RPL_IS_STORING(instance) && last_parent != NULL) {
      /* Send a No-Path DAO to the removed preferred parent. */
      dao_output(last_parent, RPL_ZERO_LIFETIME);
    }
    return NULL;
  }

  if(best_dag->preferred_parent != last_parent) {
    rpl_set_default_route(instance, rpl_get_parent_ipaddr(best_dag->preferred_parent));
    PRINTF("RPL: Changed preferred parent, rank changed from %u to %u\n",
  	(unsigned)old_rank, best_dag->rank);
    RPL_STAT(rpl_stats.parent_switch++);
    if(RPL_IS_STORING(instance)) {
      if(last_parent != NULL) {
        /* Send a No-Path DAO to the removed preferred parent. */
        dao_output(last_parent, RPL_ZERO_LIFETIME);
      }
      /* Trigger DAO transmission from immediate children.
       * Only for storing mode, see RFC6550 section 9.6. */
      RPL_LOLLIPOP_INCREMENT(instance->dtsn_out);
    }
    /* The DAO parent set changed - schedule a DAO transmission. */
    rpl_schedule_dao(instance);
    rpl_reset_dio_timer(instance);

#if DEBUG
    rpl_print_neighbor_list();
#endif
  } else if(best_dag->rank != old_rank) {
    PRINTF("RPL: Preferred parent update, rank changed from %u to %u\n",
  	(unsigned)old_rank, best_dag->rank);
  }
  return best_dag;
}
/*---------------------------------------------------------------------------*/
static rpl_parent_t *
best_parent(rpl_dag_t *dag, int fresh_only)
{
  rpl_parent_t *p;
  rpl_of_t *of;
  rpl_parent_t *best = NULL;

  if(dag == NULL || dag->instance == NULL || dag->instance->of == NULL) {
    return NULL;
  }

  of = dag->instance->of;
  /* Search for the best parent according to the OF */
  for(p = nbr_table_head(rpl_parents); p != NULL; p = nbr_table_next(rpl_parents, p)) {

    /* Exclude parents from other DAGs or announcing an infinite rank */
    if(p->dag != dag || p->rank == INFINITE_RANK || p->rank < ROOT_RANK(dag->instance)) {
      if(p->rank < ROOT_RANK(dag->instance)) {
        PRINTF("RPL: Parent has invalid rank\n");
      }
      continue;
    }

    if(fresh_only && !rpl_parent_is_fresh(p)) {
      /* Filter out non-fresh parents if fresh_only is set */
      continue;
    }

#if UIP_ND6_SEND_NS
    {
    uip_ds6_nbr_t *nbr = rpl_get_nbr(p);
    /* Exclude links to a neighbor that is not reachable at a NUD level */
    if(nbr == NULL || nbr->state != NBR_REACHABLE) {
      continue;
    }
    }
#endif /* UIP_ND6_SEND_NS */

    /* Now we have an acceptable parent, check if it is the new best */
    best = of->best_parent(best, p);
  }

  return best;
}
/*---------------------------------------------------------------------------*/
rpl_parent_t *
rpl_select_parent(rpl_dag_t *dag)
{
  /* In the RL-enabled version, the RL agent (via rpl_rl_on_dio_received)
   * and the Panic Monitor are the sole decision makers for parent changes.
   * If we already have a valid preferred parent, DO NOT let best_parent()
   * override it, as that causes massive ping-pong during periodic timer updates.
   * We only use best_parent() to find a parent if we currently have none. */
  rpl_parent_t *best;
  if(dag->preferred_parent != NULL) {
    best = dag->preferred_parent;
  } else {
    best = best_parent(dag, 0);
  }
  

  if(best != NULL) {
#if RPL_WITH_PROBING
    if(rpl_parent_is_fresh(best)) {
      rpl_set_preferred_parent(dag, best);
    } else {
      /* The best is not fresh. Look for the best fresh now. */
      rpl_parent_t *best_fresh = best_parent(dag, 1);
      if(best_fresh == NULL) {
        /* No fresh parent around, use best (non-fresh) */
        rpl_set_preferred_parent(dag, best);
      } else {
        /* Use best fresh */
        rpl_set_preferred_parent(dag, best_fresh);
      }
      /* Probe the best parent shortly in order to get a fresh estimate */
      dag->instance->urgent_probing_target = best;
      rpl_schedule_probing(dag->instance);
    }
#else /* RPL_WITH_PROBING */
    rpl_set_preferred_parent(dag, best);
    dag->rank = rpl_rank_via_parent(dag->preferred_parent);
#endif /* RPL_WITH_PROBING */
  } else {
    rpl_set_preferred_parent(dag, NULL);
  }

  dag->rank = rpl_rank_via_parent(dag->preferred_parent);
  return dag->preferred_parent;
}
/*---------------------------------------------------------------------------*/
void
rpl_remove_parent(rpl_parent_t *parent)
{
  PRINTF("RPL: Removing parent ");
  PRINT6ADDR(rpl_get_parent_ipaddr(parent));
  PRINTF("\n");

  rpl_nullify_parent(parent);

  nbr_table_remove(rpl_parents, parent);
}
/*---------------------------------------------------------------------------*/
void
rpl_nullify_parent(rpl_parent_t *parent)
{
  rpl_dag_t *dag = parent->dag;
  /* This function can be called when the preferred parent is NULL, so we
      need to handle this condition in order to trigger uip_ds6_defrt_rm. */
  if(parent == dag->preferred_parent || dag->preferred_parent == NULL) {
    dag->rank = INFINITE_RANK;
    if(dag->joined) {
      if(dag->instance->def_route != NULL) {
        PRINTF("RPL: Removing default route ");
        PRINT6ADDR(rpl_get_parent_ipaddr(parent));
        PRINTF("\n");
        uip_ds6_defrt_rm(dag->instance->def_route);
        dag->instance->def_route = NULL;
      }
      /* Send No-Path DAO only when nullifying preferred parent */
      if(parent == dag->preferred_parent) {
        if(RPL_IS_STORING(dag->instance)) {
          dao_output(parent, RPL_ZERO_LIFETIME);
        }
        rpl_set_preferred_parent(dag, NULL);
      }
    }
  }

  PRINTF("RPL: Nullifying parent ");
  PRINT6ADDR(rpl_get_parent_ipaddr(parent));
  PRINTF("\n");
}
/*---------------------------------------------------------------------------*/
void
rpl_move_parent(rpl_dag_t *dag_src, rpl_dag_t *dag_dst, rpl_parent_t *parent)
{
  if(parent == dag_src->preferred_parent) {
      rpl_set_preferred_parent(dag_src, NULL);
      dag_src->rank = INFINITE_RANK;
    if(dag_src->joined && dag_src->instance->def_route != NULL) {
      PRINTF("RPL: Removing default route ");
      PRINT6ADDR(rpl_get_parent_ipaddr(parent));
      PRINTF("\n");
      PRINTF("RPL: rpl_move_parent\n");
      uip_ds6_defrt_rm(dag_src->instance->def_route);
      dag_src->instance->def_route = NULL;
    }
  } else if(dag_src->joined) {
    if(RPL_IS_STORING(dag_src->instance)) {
      /* Remove uIPv6 routes that have this parent as the next hop. */
      rpl_remove_routes_by_nexthop(rpl_get_parent_ipaddr(parent), dag_src);
    }
  }

  PRINTF("RPL: Moving parent ");
  PRINT6ADDR(rpl_get_parent_ipaddr(parent));
  PRINTF("\n");

  parent->dag = dag_dst;
}
/*---------------------------------------------------------------------------*/
int
rpl_has_downward_route(void)
{
  int i;
  for(i = 0; i < RPL_MAX_INSTANCES; ++i) {
    if(instance_table[i].used && instance_table[i].has_downward_route) {
      return 1;
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_get_dag(const uip_ipaddr_t *addr)
{
  int i, j;

  for(i = 0; i < RPL_MAX_INSTANCES; ++i) {
    if(instance_table[i].used) {
      for(j = 0; j < RPL_MAX_DAG_PER_INSTANCE; ++j) {
        if(instance_table[i].dag_table[j].joined
            && uip_ipaddr_prefixcmp(&instance_table[i].dag_table[j].dag_id, addr,
                instance_table[i].dag_table[j].prefix_info.length)) {
          return &instance_table[i].dag_table[j];
        }
      }
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_get_any_dag(void)
{
  int i;

  for(i = 0; i < RPL_MAX_INSTANCES; ++i) {
    if(instance_table[i].used && instance_table[i].current_dag->joined) {
      return instance_table[i].current_dag;
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
rpl_instance_t *
rpl_get_instance(uint8_t instance_id)
{
  int i;

  for(i = 0; i < RPL_MAX_INSTANCES; ++i) {
    if(instance_table[i].used && instance_table[i].instance_id == instance_id) {
      return &instance_table[i];
    }
  }
  return NULL;
}
/*---------------------------------------------------------------------------*/
rpl_of_t *
rpl_find_of(rpl_ocp_t ocp)
{
  unsigned int i;

  for(i = 0;
      i < sizeof(objective_functions) / sizeof(objective_functions[0]);
      i++) {
    if(objective_functions[i]->ocp == ocp) {
      return objective_functions[i];
    }
  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
void
rpl_join_instance(uip_ipaddr_t *from, rpl_dio_t *dio)
{
  rpl_instance_t *instance;
  rpl_dag_t *dag;
  rpl_parent_t *p;
  rpl_of_t *of;

  if((!RPL_WITH_NON_STORING && dio->mop == RPL_MOP_NON_STORING)
      || (!RPL_WITH_STORING && (dio->mop == RPL_MOP_STORING_NO_MULTICAST
          || dio->mop == RPL_MOP_STORING_MULTICAST))) {
    PRINTF("RPL: DIO advertising a non-supported MOP %u\n", dio->mop);
    return;
  }

  /* Determine the objective function by using the
      objective code point of the DIO. */
  of = rpl_find_of(dio->ocp);
  if(of == NULL) {
    PRINTF("RPL: DIO for DAG instance %u does not specify a supported OF: %u\n",
           dio->instance_id, dio->ocp);
    return;
  }

  dag = rpl_alloc_dag(dio->instance_id, &dio->dag_id);
  if(dag == NULL) {
    PRINTF("RPL: Failed to allocate a DAG object!\n");
    return;
  }

  instance = dag->instance;

  p = rpl_add_parent(dag, dio, from);
  PRINTF("RPL: Adding ");
  PRINT6ADDR(from);
  PRINTF(" as a parent: ");
  if(p == NULL) {
    PRINTF("failed\n");
    instance->used = 0;
    return;
  }
  p->dtsn = dio->dtsn;
  PRINTF("succeeded\n");

  /* Autoconfigure an address if this node does not already have an address
      with this prefix. */
  if(dio->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
    check_prefix(NULL, &dio->prefix_info);
  }

  dag->joined = 1;
  dag->preference = dio->preference;
  dag->grounded = dio->grounded;
  dag->version = dio->version;

  instance->of = of;
  instance->mop = dio->mop;
  instance->mc.type = dio->mc.type;
  instance->mc.flags = dio->mc.flags;
  instance->mc.aggr = dio->mc.aggr;
  instance->mc.prec = dio->mc.prec;
  instance->current_dag = dag;
  instance->dtsn_out = RPL_LOLLIPOP_INIT;

  instance->max_rankinc = dio->dag_max_rankinc;
  instance->min_hoprankinc = dio->dag_min_hoprankinc;
  instance->dio_intdoubl = dio->dag_intdoubl;
  instance->dio_intmin = dio->dag_intmin;
  instance->dio_intcurrent = instance->dio_intmin + instance->dio_intdoubl;
  instance->dio_redundancy = dio->dag_redund;
  instance->default_lifetime = dio->default_lifetime;
  instance->lifetime_unit = dio->lifetime_unit;

  memcpy(&dag->dag_id, &dio->dag_id, sizeof(dio->dag_id));

  /* Copy prefix information from the DIO into the DAG object. */
  memcpy(&dag->prefix_info, &dio->prefix_info, sizeof(rpl_prefix_t));

  rpl_set_preferred_parent(dag, p);
  instance->of->update_metric_container(instance);
  dag->rank = rpl_rank_via_parent(p);
  /* So far this is the lowest rank we are aware of. */
  dag->min_rank = dag->rank;

  if(default_instance == NULL) {
    default_instance = instance;
  }

  PRINTF("RPL: Joined DAG with instance ID %u, rank %hu, DAG ID ",
         dio->instance_id, dag->rank);
  PRINT6ADDR(&dag->dag_id);
  PRINTF("\n");

  ANNOTATE("#A join=%u\n", dag->dag_id.u8[sizeof(dag->dag_id) - 1]);

  rpl_reset_dio_timer(instance);

  rpl_set_default_route(instance, from);

  if(instance->mop != RPL_MOP_NO_DOWNWARD_ROUTES) {
    rpl_schedule_dao(instance);
  } else {
    PRINTF("RPL: The DIO does not meet the prerequisites for sending a DAO\n");
  }

  instance->of->reset(dag);
}

#if RPL_MAX_DAG_PER_INSTANCE > 1
/*---------------------------------------------------------------------------*/
rpl_dag_t *
rpl_add_dag(uip_ipaddr_t *from, rpl_dio_t *dio)
{
  rpl_instance_t *instance;
  rpl_dag_t *dag, *previous_dag;
  rpl_parent_t *p;
  rpl_of_t *of;

  dag = rpl_alloc_dag(dio->instance_id, &dio->dag_id);
  if(dag == NULL) {
    PRINTF("RPL: Failed to allocate a DAG object!\n");
    return NULL;
  }

  instance = dag->instance;

  previous_dag = find_parent_dag(instance, from);
  if(previous_dag == NULL) {
    PRINTF("RPL: Adding ");
    PRINT6ADDR(from);
    PRINTF(" as a parent: ");
    p = rpl_add_parent(dag, dio, from);
    if(p == NULL) {
      PRINTF("failed\n");
      dag->used = 0;
      return NULL;
    }
    PRINTF("succeeded\n");
  } else {
    p = rpl_find_parent(previous_dag, from);
    if(p != NULL) {
      rpl_move_parent(previous_dag, dag, p);
    }
  }
  p->rank = dio->rank;

  /* Determine the objective function by using the
      objective code point of the DIO. */
  of = rpl_find_of(dio->ocp);
  if(of != instance->of ||
      instance->mop != dio->mop ||
      instance->max_rankinc != dio->dag_max_rankinc ||
      instance->min_hoprankinc != dio->dag_min_hoprankinc ||
      instance->dio_intdoubl != dio->dag_intdoubl ||
      instance->dio_intmin != dio->dag_intmin ||
      instance->dio_redundancy != dio->dag_redund ||
      instance->default_lifetime != dio->default_lifetime ||
      instance->lifetime_unit != dio->lifetime_unit) {
    PRINTF("RPL: DIO for DAG instance %u incompatible with previous DIO\n",
	    dio->instance_id);
    rpl_remove_parent(p);
    dag->used = 0;
    return NULL;
  }

  dag->used = 1;
  dag->grounded = dio->grounded;
  dag->preference = dio->preference;
  dag->version = dio->version;

  memcpy(&dag->dag_id, &dio->dag_id, sizeof(dio->dag_id));

  /* copy prefix information into the dag */
  memcpy(&dag->prefix_info, &dio->prefix_info, sizeof(rpl_prefix_t));

  rpl_set_preferred_parent(dag, p);
  dag->rank = rpl_rank_via_parent(p);
  dag->min_rank = dag->rank; /* So far this is the lowest rank we know of. */

  PRINTF("RPL: Joined DAG with instance ID %u, rank %hu, DAG ID ",
         dio->instance_id, dag->rank);
  PRINT6ADDR(&dag->dag_id);
  PRINTF("\n");

  ANNOTATE("#A join=%u\n", dag->dag_id.u8[sizeof(dag->dag_id) - 1]);

  rpl_process_parent_event(instance, p);
  p->dtsn = dio->dtsn;

  return dag;
}
#endif /* RPL_MAX_DAG_PER_INSTANCE > 1 */

/*---------------------------------------------------------------------------*/
static void
global_repair(uip_ipaddr_t *from, rpl_dag_t *dag, rpl_dio_t *dio)
{
  rpl_parent_t *p;

  remove_parents(dag, 0);
  dag->version = dio->version;

  /* copy parts of the configuration so that it propagates in the network */
  dag->instance->dio_intdoubl = dio->dag_intdoubl;
  dag->instance->dio_intmin = dio->dag_intmin;
  dag->instance->dio_redundancy = dio->dag_redund;
  dag->instance->default_lifetime = dio->default_lifetime;
  dag->instance->lifetime_unit = dio->lifetime_unit;

  dag->instance->of->reset(dag);
  dag->min_rank = INFINITE_RANK;
  RPL_LOLLIPOP_INCREMENT(dag->instance->dtsn_out);

  p = rpl_add_parent(dag, dio, from);
  if(p == NULL) {
    PRINTF("RPL: Failed to add a parent during the global repair\n");
    dag->rank = INFINITE_RANK;
  } else {
    dag->rank = rpl_rank_via_parent(p);
    dag->min_rank = dag->rank;
    PRINTF("RPL: rpl_process_parent_event global repair\n");
    rpl_process_parent_event(dag->instance, p);
  }

  PRINTF("RPL: Participating in a global repair (version=%u, rank=%hu)\n",
         dag->version, dag->rank);

  RPL_STAT(rpl_stats.global_repairs++);
}

/*---------------------------------------------------------------------------*/
void
rpl_local_repair(rpl_instance_t *instance)
{
  int i;

  if(instance == NULL) {
    PRINTF("RPL: local repair requested for instance NULL\n");
    return;
  }
  PRINTF("RPL: Starting a local instance repair\n");
  for(i = 0; i < RPL_MAX_DAG_PER_INSTANCE; i++) {
    if(instance->dag_table[i].used) {
      instance->dag_table[i].rank = INFINITE_RANK;
      nullify_parents(&instance->dag_table[i], 0);
    }
  }

  /* no downward route anymore */
  instance->has_downward_route = 0;

  rpl_reset_dio_timer(instance);

  if(RPL_IS_STORING(instance)) {
    /* Request refresh of DAO registrations next DIO. Only for storing mode. In
     * non-storing mode, non-root nodes increment DTSN only on when their parent do,
     * or on global repair (see RFC6550 section 9.6.) */
    RPL_LOLLIPOP_INCREMENT(instance->dtsn_out);
  }

  RPL_STAT(rpl_stats.local_repairs++);
}
/*---------------------------------------------------------------------------*/
void
rpl_recalculate_ranks(void)
{
  rpl_parent_t *p;

  /*
   * We recalculate ranks when we receive feedback from the system rather
   * than RPL protocol messages. This periodical recalculation is called
   * from a timer in order to keep the stack depth reasonably low.
   */
  p = nbr_table_head(rpl_parents);
  while(p != NULL) {
    if(p->dag != NULL && p->dag->instance && (p->flags & RPL_PARENT_FLAG_UPDATED)) {
      p->flags &= ~RPL_PARENT_FLAG_UPDATED;
      PRINTF("RPL: rpl_process_parent_event recalculate_ranks\n");
      if(!rpl_process_parent_event(p->dag->instance, p)) {
        PRINTF("RPL: A parent was dropped\n");
      }
    }
    p = nbr_table_next(rpl_parents, p);
  }
}
/*---------------------------------------------------------------------------*/
int
rpl_process_parent_event(rpl_instance_t *instance, rpl_parent_t *p)
{
  int return_value;
  rpl_parent_t *last_parent = instance->current_dag->preferred_parent;

#if DEBUG
  rpl_rank_t old_rank;
  old_rank = instance->current_dag->rank;
#endif /* DEBUG */

  return_value = 1;

  if(RPL_IS_STORING(instance)
      && uip_ds6_route_is_nexthop(rpl_get_parent_ipaddr(p))
      && !rpl_parent_is_reachable(p) && instance->mop > RPL_MOP_NON_STORING) {
    PRINTF("RPL: Unacceptable link %u, removing routes via: ", rpl_get_parent_link_metric(p));
    PRINT6ADDR(rpl_get_parent_ipaddr(p));
    PRINTF("\n");
    rpl_remove_routes_by_nexthop(rpl_get_parent_ipaddr(p), p->dag);
  }

  if(!acceptable_rank(p->dag, p->rank)) {
    /* The candidate parent is no longer valid: the rank increase resulting
       from the choice of it as a parent would be too high. */
    PRINTF("RPL: Unacceptable rank %u (Current min %u, MaxRankInc %u)\n", (unsigned)p->rank,
        p->dag->min_rank, p->dag->instance->max_rankinc);
    rpl_nullify_parent(p);
    if(p != instance->current_dag->preferred_parent) {
      return 0;
    } else {
      return_value = 0;
    }
  }

  if(rpl_select_dag(instance, p) == NULL) {
    if(last_parent != NULL) {
      /* No suitable parent anymore; trigger a local repair. */
      PRINTF("RPL: No parents found in any DAG\n");
      rpl_local_repair(instance);
      return 0;
    }
  }

#if DEBUG
  if(DAG_RANK(old_rank, instance) != DAG_RANK(instance->current_dag->rank, instance)) {
    PRINTF("RPL: Moving in the instance from rank %hu to %hu\n",
	    DAG_RANK(old_rank, instance), DAG_RANK(instance->current_dag->rank, instance));
    if(instance->current_dag->rank != INFINITE_RANK) {
      PRINTF("RPL: The preferred parent is ");
      PRINT6ADDR(rpl_get_parent_ipaddr(instance->current_dag->preferred_parent));
      PRINTF(" (rank %u)\n",
            (unsigned)DAG_RANK(instance->current_dag->preferred_parent->rank, instance));
    } else {
      PRINTF("RPL: We don't have any parent");
    }
  }
#endif /* DEBUG */

  return return_value;
}
/*---------------------------------------------------------------------------*/
static int
add_nbr_from_dio(uip_ipaddr_t *from, rpl_dio_t *dio)
{
  /* add this to the neighbor cache if not already there */
  if(rpl_icmp6_update_nbr_table(from, NBR_TABLE_REASON_RPL_DIO, dio) == NULL) {
    PRINTF("RPL: Out of memory, dropping DIO from ");
    PRINT6ADDR(from);
    PRINTF("\n");
    return 0;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
void
rpl_process_dio(uip_ipaddr_t *from, rpl_dio_t *dio)
{
  rpl_instance_t *instance;
  rpl_dag_t *dag, *previous_dag;
  rpl_parent_t *p;

#if RPL_WITH_MULTICAST
  /* If the root is advertising MOP 2 but we support MOP 3 we can still join
   * In that scenario, we suppress DAOs for multicast targets */
  if(dio->mop < RPL_MOP_STORING_NO_MULTICAST) {
#else
  if(dio->mop != RPL_MOP_DEFAULT) {
#endif
    PRINTF("RPL: Ignoring a DIO with an unsupported MOP: %d\n", dio->mop);
    return;
  }

  dag = get_dag(dio->instance_id, &dio->dag_id);
  instance = rpl_get_instance(dio->instance_id);

  if(dag != NULL && instance != NULL) {
    if(lollipop_greater_than(dio->version, dag->version)) {
      if(dag->rank == ROOT_RANK(instance)) {

        PRINTF("RPL: Root received inconsistent DIO version number (current: %u, received: %u)\n", dag->version, dio->version);
        dag->version = dio->version;
        RPL_LOLLIPOP_INCREMENT(dag->version);
        rpl_reset_dio_timer(instance);

      } else {
        PRINTF("RPL: Global repair\n");
        if(dio->prefix_info.length != 0) {
          if(dio->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
            PRINTF("RPL: Prefix announced in DIO\n");
            rpl_set_prefix(dag, &dio->prefix_info.prefix, dio->prefix_info.length);
          }
        }
        global_repair(from, dag, dio);
      }
      return;
    }

    if(lollipop_greater_than(dag->version, dio->version)) {
      /* The DIO sender is on an older version of the DAG. */
      PRINTF("RPL: old version received => inconsistency detected\n");
      if(dag->joined) {
        rpl_reset_dio_timer(instance);

        return;
      }
    }
  }

  if(instance == NULL) {
    PRINTF("RPL: New instance detected (ID=%u): Joining...\n", dio->instance_id);
    if(add_nbr_from_dio(from, dio)) {
      rpl_join_instance(from, dio);
    } else {
      PRINTF("RPL: Not joining since could not add parent\n");
    }
    return;
  }

  if(instance->current_dag->rank == ROOT_RANK(instance) && instance->current_dag != dag) {
    PRINTF("RPL: Root ignored DIO for different DAG\n");
    return;
  }

  if(dag == NULL) {
#if RPL_MAX_DAG_PER_INSTANCE > 1
    PRINTF("RPL: Adding new DAG to known instance.\n");
    if(!add_nbr_from_dio(from, dio)) {
      PRINTF("RPL: Could not add new DAG, could not add parent\n");
      return;
    }
    dag = rpl_add_dag(from, dio);
    if(dag == NULL) {
      PRINTF("RPL: Failed to add DAG.\n");
      return;
    }
#else /* RPL_MAX_DAG_PER_INSTANCE > 1 */
    PRINTF("RPL: Only one instance supported.\n");
    return;
#endif /* RPL_MAX_DAG_PER_INSTANCE > 1 */
  }


  if(dio->rank < ROOT_RANK(instance)) {
    PRINTF("RPL: Ignoring DIO with too low rank: %u\n",
           (unsigned)dio->rank);
    return;
  }

  /* Prefix Information Option treated to add new prefix */
  if(dio->prefix_info.length != 0) {
    if(dio->prefix_info.flags & UIP_ND6_RA_FLAG_AUTONOMOUS) {
      PRINTF("RPL: Prefix announced in DIO\n");
      rpl_set_prefix(dag, &dio->prefix_info.prefix, dio->prefix_info.length);
    }
  }

  if(!add_nbr_from_dio(from, dio)) {
    PRINTF("RPL: Could not add parent based on DIO\n");
    return;
  }

  if(dag->rank == ROOT_RANK(instance)) {
    if(dio->rank != INFINITE_RANK) {
      instance->dio_counter++;
      printf("test rlf consis instance->dio_counter = %u \n",instance->dio_counter);

      
    }
    return;
  }

  /* The DIO comes from a valid DAG, we can refresh its lifetime */
  dag->lifetime = (1UL << (instance->dio_intmin + instance->dio_intdoubl)) * RPL_DAG_LIFETIME / 1000;
  PRINTF("RPL: Set dag ");
  PRINT6ADDR(&dag->dag_id);
  PRINTF(" lifetime to %ld\n", dag->lifetime);

  /*
   * At this point, we know that this DIO pertains to a DAG that
   * we are already part of. We consider the sender of the DIO to be
   * a candidate parent, and let rpl_process_parent_event decide
   * whether to keep it in the set.
   */

  p = rpl_find_parent(dag, from);
  if(p == NULL) {
    previous_dag = find_parent_dag(instance, from);
    if(previous_dag == NULL) {
      /* Add the DIO sender as a candidate parent. */
      p = rpl_add_parent(dag, dio, from);
      if(p == NULL) {
        PRINTF("RPL: Failed to add a new parent (");
        PRINT6ADDR(from);
        PRINTF(")\n");
        return;
      }
      PRINTF("RPL: New candidate parent with rank %u: ", (unsigned)p->rank);
      PRINT6ADDR(from);
      PRINTF("\n");
    } else {
      p = rpl_find_parent(previous_dag, from);
      if(p != NULL) {
        rpl_move_parent(previous_dag, dag, p);
      }
    }
  } else {
    if(p->rank == dio->rank) {
      PRINTF("RPL: Received consistent DIO\n");
      if(dag->joined) {
        instance->dio_counter++; 
        printf("test rlf consis instance->dio_counter = %u \n",instance->dio_counter);


      }
    }
  }
  p->rank = dio->rank;
  /* Refresh PE fields and tau candidate on every DIO */
  p->pe_Tau  = dio->pe_Tau;

  /* τ_cand = F(τ_u, P_lien(i,u)) */
  p->tau_cand = rpl_tau_compute_cand(
    rpl_etx_norm(p), rpl_rssi_norm(p), p->pe_Tau);

  if(p->tau_cand == 0) {
    p->tau_cand = 500;
  }

  /* --- RL: update candidate table + run three-gate decision on every DIO ---
   * Reads RSSI from link-stats (measured locally on DIO arrival).
   * rpl_rl_on_dio_received() implements the full three-gate decision flow:
   *   Gate 1, Gate 2 (Delta_Q), Gate 3 (physical hysteresis), Commit.       */
  {
    const struct link_stats *rl_ls = rpl_get_parent_link_stats(p);
    int16_t rssi_now = (rl_ls != NULL) ? rl_ls->rssi : (int16_t)(-85);
    rpl_rl_on_dio_received(dag, p, rssi_now);
  }


  if(dio->rank == INFINITE_RANK && p == dag->preferred_parent) {
    /* Our preferred parent advertised an infinite rank, reset DIO timer */
    rpl_reset_dio_timer(instance);

  }

  /* Parent info has been updated, trigger rank recalculation */
  p->flags |= RPL_PARENT_FLAG_UPDATED;

  PRINTF("RPL: preferred DAG ");
  PRINT6ADDR(&instance->current_dag->dag_id);
  PRINTF(", rank %u, min_rank %u, ",
	 instance->current_dag->rank, instance->current_dag->min_rank);
  PRINTF("parent rank %u, link metric %u\n",
	 p->rank, rpl_get_parent_link_metric(p));

  /* We have allocated a candidate parent; process the DIO further. */

#if RPL_WITH_MC
  memcpy(&p->mc, &dio->mc, sizeof(p->mc));
#endif /* RPL_WITH_MC */
  /* When the RL agent is active, it has already made the parent selection
   * decision via rpl_rl_on_dio_received() above. Calling
   * rpl_process_parent_event() here would trigger rpl_select_dag() →
   * best_parent() which may override the RL agent's choice, causing
   * a ping-pong effect and massive NPC inflation.
   * We still need to update the rank to keep DODAG consistent. */
  dag->rank = rpl_rank_via_parent(dag->preferred_parent);

  /* We don't use route control, so we can have only one official parent. */
  if(dag->joined && p == dag->preferred_parent) {
    if(should_refresh_routes(instance, dio, p)) {
      /* Our parent is requesting a new DAO. Increment DTSN in turn,
       * in both storing and non-storing mode (see RFC6550 section 9.6.) */
      RPL_LOLLIPOP_INCREMENT(instance->dtsn_out);
      rpl_schedule_dao(instance);
    }
    /* We received a new DIO from our preferred parent.
     * Call uip_ds6_defrt_add to set a fresh value for the lifetime counter */
    uip_ds6_defrt_add(from, RPL_DEFAULT_ROUTE_INFINITE_LIFETIME ? 0 : RPL_LIFETIME(instance, instance->default_lifetime));
  }
  p->dtsn = dio->dtsn;
}
/*---------------------------------------------------------------------------*/
/** @} */
