/*
 * Copyright (c) 2024
 * All rights reserved.
 *
 * \file
 *   RIMS-RPL: Reinforcement Learning-based Intelligent Mobility-Support
 *   Header file defining Q-Learning agent parameters and API.
 *
 * \author PFE Implementation
 */

#ifndef RPL_RIMS_H
#define RPL_RIMS_H

#include "net/rpl/rpl.h"
#include "sys/ctimer.h"

/*---------------------------------------------------------------------------*/
/* Fixed-point arithmetic scale factor (×100, avoids floating point) */
#define RIMS_FP_SCALE             100

/*---------------------------------------------------------------------------*/
/* Q-Learning Parameters (all scaled ×100 for fixed-point) */
#define RIMS_RL_ALPHA              40   /* Learning rate: 0.4 × 100 */
#define RIMS_RL_GAMMA              30   /* Discount factor: 0.3 × 100 */
#define RIMS_RL_EPSILON             1   /* Exploration rate: 0.01 × 100 */

/*---------------------------------------------------------------------------*/
/* States (3 states) */
#define RIMS_NUM_STATES             3
#define RIMS_ST_OUTDATED            0   /* Parent unchanged too long */
#define RIMS_ST_HANDOFF             1   /* Too many recent parent changes */
#define RIMS_ST_STABLE              2   /* Link is stable */

/*---------------------------------------------------------------------------*/
/* Actions (3 actions) */
#define RIMS_NUM_ACTIONS            3
#define RIMS_ACT_REPAIR             0   /* Nullify parent + reset Trickle */
#define RIMS_ACT_DIS                1   /* Send a DIS message */
#define RIMS_ACT_NONE               2   /* Do nothing */

/*---------------------------------------------------------------------------*/
/* Mobility thresholds */
#ifndef RIMS_CONF_THRESHOLD_U
#define RIMS_THRESHOLD_U           10   /* Unchanged count before OUTDATED */
#else
#define RIMS_THRESHOLD_U           RIMS_CONF_THRESHOLD_U
#endif

#ifndef RIMS_CONF_THRESHOLD_C
#define RIMS_THRESHOLD_C            5   /* Changed count before HANDOFF */
#else
#define RIMS_THRESHOLD_C           RIMS_CONF_THRESHOLD_C
#endif

/*---------------------------------------------------------------------------*/
/* Monitoring interval */
#ifndef RIMS_CONF_MONITOR_INTERVAL
#define RIMS_MONITOR_INTERVAL      (5 * CLOCK_SECOND)
#else
#define RIMS_MONITOR_INTERVAL      RIMS_CONF_MONITOR_INTERVAL
#endif

/*---------------------------------------------------------------------------*/
/* Hybrid Mobility Detection / RPL* Parameters */
#ifndef RIMS_CONF_M_TIMER_ALPHA
#define RIMS_M_TIMER_ALPHA         80   /* EWMA Weight (80% historical, 20% fresh) */
#else
#define RIMS_M_TIMER_ALPHA         RIMS_CONF_M_TIMER_ALPHA
#endif

#ifndef RIMS_CONF_MOBILITY_THRESHOLD
#define RIMS_MOBILITY_THRESHOLD    10   /* t_mc < 10 => Mobile */
#else
#define RIMS_MOBILITY_THRESHOLD    RIMS_CONF_MOBILITY_THRESHOLD
#endif

#ifndef RIMS_CONF_MAX_STATIC_TICKS
#define RIMS_MAX_STATIC_TICKS      30   /* Absolute Stability Promotion (approx 150s) */
#else
#define RIMS_MAX_STATIC_TICKS      RIMS_CONF_MAX_STATIC_TICKS
#endif

#ifndef RIMS_CONF_FAST_DROP_TICKS
#define RIMS_FAST_DROP_TICKS       4    /* Fast-Sprint Demotion threshold (approx 20s) */
#else
#define RIMS_FAST_DROP_TICKS       RIMS_CONF_FAST_DROP_TICKS
#endif

/*---------------------------------------------------------------------------*/
/* API */

/**
 * \brief Initialize the RIMS Q-Learning agent.
 *        Zeroes Q-table and counters. Call once at DAG init.
 */
void rims_init(void);

/**
 * \brief Start the periodic RIMS monitoring timer.
 *        Call after the node has joined a DODAG.
 */
void rims_start(void);

/**
 * \brief Stop the RIMS monitoring timer.
 */
void rims_stop(void);

/**
 * \brief Notify RIMS that the preferred parent has changed.
 *        Used to track parent change frequency.
 */
void rims_notify_parent_change(void);

/**
 * \brief Returns the node's dynamically calculated mobility state.
 * \return 1 if Fixed (infrastructure/stable), 0 if Mobile.
 */
uint8_t rims_is_fixed(void);

/*---------------------------------------------------------------------------*/
#endif /* RPL_RIMS_H */
