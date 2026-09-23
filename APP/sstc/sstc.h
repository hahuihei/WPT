/**
 * sstc.h — Dual-Output Steady-State Target Calculation
 * ======================================================
 * MATLAB: SSTC(yt, ut, A, B, C, D, Q, R, u_min, u_max, y_min, y_max)
 *   with C=[1 0;0 1], yt,ys 2D, Q 2×2
 *
 * DSP: qpOASES QProblem, 4 variables [y1;y2;u;eta], 2 equalities + 2 soft constraints
 *   软约束: |yt2 - y2| <= eta (电流跟踪软化, 论文式16-17)
 */

#ifndef SSTC_H_
#define SSTC_H_

#include "DSP2833x_Device.h"

#define SSTC_NV  4   /* y1, y2, u, eta */
#define SSTC_NC  4   /* 2 equalities (y=K*u) + 2 soft constraints (|yt2-y2|<=eta) */

#ifdef __cplusplus
extern "C" {
#endif

void SSTC_Init(void);
void SSTC_SetRL(float32 newRL);  /* 检测到RL变化时调用, 重算K并重置QP */
void SSTC_Compute(
    const float32 *yt,          /* yt[2]: [voltage_tgt, current_tgt] */
    const float32 *Q_diag,      /* Q_diag[2] */
    float32 Rs,                 /* R scalar */
    float32 ut,                 /* initial duty guess */
    const float32 *y_min,       /* y_min[2] */
    const float32 *y_max,       /* y_max[2] */
    float32 *ys_opt,            /* ys_opt[2] out */
    float32 *us_opt);           /* us_opt out */

float32 SSTC_Gain(Uint16 idx);  /* K[idx]  (0=voltage, 1=current) */

#ifdef __cplusplus
}
#endif

#endif
