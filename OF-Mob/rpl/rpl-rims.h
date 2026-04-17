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

/*---------------------------------------------------------------------------*/
#endif /* RPL_RIMS_H */
