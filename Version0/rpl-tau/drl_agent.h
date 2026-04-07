#ifndef DRL_AGENT_H
#define DRL_AGENT_H

#include <stdint.h>
#include "net/rpl/rpl.h"

#ifndef DRL_FEATURES
#define DRL_FEATURES 11
#endif

typedef struct {
  /* Q weights in Q15 fixed point */
  int16_t w[DRL_FEATURES];
} drl_model_t;

void drl_init(void);

/* Return Q score for parent p */
int32_t drl_score_parent(rpl_parent_t *p,
                         uint16_t tau_cand,
                         uint16_t rssi_n,
                         uint16_t etx_n,
                         uint16_t lstab_n,
                         uint16_t hops_n,
                         uint16_t E_i,
                         uint16_t InvV_i,
                         uint16_t Deg_i,
                         uint16_t Stb_i);

/* Training hook (Cooja): update weights from (features, reward) */
void drl_train_step(int32_t q_chosen, int32_t reward_q15);

/* Logging (Cooja) */
void drl_log_decision(rpl_parent_t *p, int32_t q, int32_t reward_q15);

extern drl_model_t g_drl;

#endif

