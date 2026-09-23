/**
 * qpoases_port.h — C wrapper to call qpOASES QProblem from MPC (Dual-Output)
 * ==========================================================================
 * QP form:
 *   z = [y1(0..4); y2(0..4); u(0..2)],  n=13,  m=10
 *   min  0.5 * z' * H * z + g' * z
 *   s.t.  lb <= z <= ub
 *         lbA <= Aeq * z <= ubA
 *
 *   H = 2*diag(Q11×5, Q22×5, R×3)    13×13
 *   g = -H * [ys1×5; ys2×5; us×3]    13×1
 *   lb/ub = [y1_min×5;y2_min×5;u_min×3] / [...yb]
 *   Aeq = [I_10, g_MP]                10×13
 *   lbA = ubA = g_be                  10×1
 */

#ifndef QPOASES_PORT_H
#define QPOASES_PORT_H

#include "DSP2833x_Device.h"

#define QP_NY   2    /* 2 outputs: voltage + current */
#define QP_NU   1    /* 1 control */
#define QP_NP   5    /* prediction horizon */
#define QP_NM   3    /* control horizon */
#define QP_NV   (QP_NP * QP_NY + QP_NM)  /* 13 */
#define QP_NC   (QP_NP * QP_NY)          /* 10 */

void QPOASES_Init(void);

int QPOASES_Solve(
    const float32 *g_MP_flat,       /* g_MP[10][3] flat row-major */
    const float32 *g_be,            /* g_be[10] */
    const float32 *Q_diag,          /* Q[QP_NY] diagonal */
    float32 Rs,
    const float32 *ys_ref,          /* ys_ref[QP_NY] */
    float32 us_ref,
    const float32 *y_min,           /* y_min[QP_NY] */
    const float32 *y_max,           /* y_max[QP_NY] */
    float32 u_min_val, float32 u_max_val,
    float32 *u_opt,                 /* u_opt[3] out */
    float32 *obj_val,
    int    *iter,
    int    *exit_flag);

#endif
