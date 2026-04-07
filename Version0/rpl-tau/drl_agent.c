#include "drl_agent.h"
#include <stdio.h>

#ifndef DRL_TRAINING
#define DRL_TRAINING 0
#endif

#ifndef DRL_LOGGING
#define DRL_LOGGING  1
#endif

/* learning rate in Q15 (e.g., 0.02 ~ 655) */
#ifndef DRL_LR_Q15
#define DRL_LR_Q15 655
#endif

drl_model_t g_drl;

/* Helpers: map 0..1000 -> Q15 (0..32767) */
static inline int16_t n1000_to_q15(uint16_t v) {
  if(v > 1000) v = 1000;
  return (int16_t)((int32_t)v * 32767 / 1000);
}

/* Dot product w·x (Q15) => Q15 */
static int32_t dot_q15(const int16_t *w, const int16_t *x, int n) {
  int32_t acc = 0;
  int i;
  for(i = 0; i < n; i++) {
    acc += (int32_t)w[i] * (int32_t)x[i]; /* Q15*Q15 => Q30 */
  }
  return acc >> 15; /* back to Q15 */
}

void drl_init(void)
{
  /* Small reasonable defaults (bias + some preference for tau, low ETX/hops) */
  int i;
  for(i = 0; i < DRL_FEATURES; i++) g_drl.w[i] = 0;

  g_drl.w[0] = 1000;   /* bias */
  g_drl.w[1] = 1200;   /* tau_cand (+) */
  g_drl.w[2] = 600;    /* rssi (+) */
  g_drl.w[3] = -1600;  /* etx (-) */
  g_drl.w[4] = 400;    /* link stability (+) */
  g_drl.w[5] = -900;   /* hops (-) */
  /* local Pi terms */
  g_drl.w[6] = 200;
  g_drl.w[7] = 100;
  g_drl.w[8] = 50;
  g_drl.w[9] = 200;
  g_drl.w[10] = 0; /* reserved */
}

int32_t drl_score_parent(rpl_parent_t *p,
                         uint16_t tau_cand,
                         uint16_t rssi_n,
                         uint16_t etx_n,
                         uint16_t lstab_n,
                         uint16_t hops_n,
                         uint16_t E_i,
                         uint16_t InvV_i,
                         uint16_t Deg_i,
                         uint16_t Stb_i)
{
  (void)p;

  int16_t x[DRL_FEATURES];
  x[0] = 32767; /* bias 1.0 in Q15 */
  x[1] = n1000_to_q15(tau_cand);
  x[2] = n1000_to_q15(rssi_n);
  x[3] = n1000_to_q15(etx_n);
  x[4] = n1000_to_q15(lstab_n);
  x[5] = n1000_to_q15(hops_n);
  x[6] = n1000_to_q15(E_i);
  x[7] = n1000_to_q15(InvV_i);
  x[8] = n1000_to_q15(Deg_i);
  x[9] = n1000_to_q15(Stb_i);
  x[10] = 0;

  return dot_q15(g_drl.w, x, DRL_FEATURES);
}

/* Simple bandit-like update: w += lr * reward * x
 * (for real RL, you’d use TD error; this keeps it stable and lightweight)
 */
void drl_train_step(int32_t q_chosen, int32_t reward_q15)
{
#if DRL_TRAINING
  (void)q_chosen;
  /* In this minimal version, training is done in caller with full x if needed. */
  (void)reward_q15;
#else
  (void)q_chosen;
  (void)reward_q15;
#endif
}

void drl_log_decision(rpl_parent_t *p, int32_t q, int32_t reward_q15)
{
#if DRL_LOGGING
  if(p == NULL) return;
  /* rpl_parent_t->mc exists only when RPL_WITH_MC is enabled.
   * Also, rpl_parent_t has no 'link_metric' field. Use the public API. */
  uint16_t lm = rpl_get_parent_link_metric(p);
  /* Log: parent addr + q + reward + key stats */
  printf("DRL_DEC parent=%u q=%ld reward=%ld lm=%u tau_cand=%u\n",
         (unsigned)p->rank, (long)q, (long)reward_q15,
         (unsigned)lm, (unsigned)p->tau_cand);
#else
  (void)p; (void)q; (void)reward_q15;
#endif
}

