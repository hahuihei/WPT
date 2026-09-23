/**
 * qpoases_port.cpp — Dual-Output MPC QP via qpOASES
 * ===================================================
 * z = [y1×5; y2×5; u×3],  n=13,  m=10
 */

#include <qpOASES/QProblem.hpp>
#include <qpOASES/Options.hpp>
#include <qpOASES/MessageHandling.hpp>
#include <string.h>

USING_NAMESPACE_QPOASES

extern "C" {
#include "qpoases_port.h"
}

static QProblem *qp = 0;
static Options    qpOptions;
static int        qpInited = 0;

#define NV   QP_NV   /* 13 */
#define NC   QP_NC   /* 10 */
#define Np   QP_NP   /* 5 */
#define Nm   QP_NM   /* 3 */
#define Ny   QP_NY   /* 2 */

/* Δu 惩罚权重 H (论文式18第三项 Σ‖Δu‖²_H, 防止占空比突变) */
#define DU_WEIGHT_H   1.0f

static float H[NV * NV];
static float g_vec[NV];
static float Aeq[NC * NV];
static float lb[NV], ub[NV];
static float lbA[NC], ubA[NC];
static float x_opt[NV];
static float last_u_opt[Nm] = {0.0f};   /* 上一拍成功的控制量, 失败时回退 */
static int   last_valid = 0;            /* 是否有过成功解 */
static float prev_u = 0.0f;             /* 上一拍实际占空比 (Δu 惩罚基准) */

void QPOASES_Init(void)
{
    if (qp) { delete qp; qp = 0; }
    qpInited = 0;
}

int QPOASES_Solve(
    const float *g_MP_flat,
    const float *g_be,
    const float *Q_diag,
    float Rs,
    const float *ys_ref,
    float us_ref,
    const float *y_min,
    const float *y_max,
    float u_min_val, float u_max_val,
    float *u_opt, float *obj_val,
    int *iter, int *exit_flag)
{
    int i, j, nWSR;
    float cputime = 0.0f;
    returnValue retval;

    /* ---- First call: build constant matrices ---- */
    if (qpInited == 0) {
        memset(H, 0, sizeof(H));
        /* H = 2*diag(Q11×Np, Q22×Np, R×Nm)
           Q_diag = [Q11, Q22] from the 2×2 diagonal Q  */
        for (i = 0; i < Np * Ny; i++) {
            int out_idx = i % Ny;     /* 0 or 1 */
            H[i * NV + i] = 2.0f * Q_diag[out_idx];
        }
        for (i = 0; i < Nm; i++) {
            H[(Np * Ny + i) * NV + (Np * Ny + i)] = 2.0f * Rs;
        }

        /* Δu 惩罚二次项 (论文式18第三项): H_du·Σ(Δu)²
           Δu0=u0-prev_u, Δu1=u1-u0, Δu2=u2-u1
           H 的 u 块(3×3)叠加 H_du·[[4,-2,0],[-2,4,-2],[0,-2,2]] */
        {
            int u0 = Np * Ny + 0, u1 = Np * Ny + 1, u2 = Np * Ny + 2;
            H[u0*NV+u0] += 4.0f * DU_WEIGHT_H;
            H[u1*NV+u1] += 4.0f * DU_WEIGHT_H;
            H[u2*NV+u2] += 2.0f * DU_WEIGHT_H;
            H[u0*NV+u1] += -2.0f * DU_WEIGHT_H;
            H[u1*NV+u0] += -2.0f * DU_WEIGHT_H;
            H[u1*NV+u2] += -2.0f * DU_WEIGHT_H;
            H[u2*NV+u1] += -2.0f * DU_WEIGHT_H;
        }

        /* Aeq = [I_10, g_MP]  (g_MP_flat is 10×3 row-major) */
        memset(Aeq, 0, sizeof(Aeq));
        for (i = 0; i < NC; i++) {
            Aeq[i * NV + i] = 1.0f;        /* I_10 */
            for (j = 0; j < Nm; j++)
                Aeq[i * NV + Np * Ny + j] = g_MP_flat[i * Nm + j];
        }

        qpOptions.setToMPC();
        qpOptions.printLevel = PL_NONE;

        qp = new QProblem(NV, NC);
        if (qp == 0) return -1;
        qp->setOptions(qpOptions);
        qpInited = 1;
    }

    /* ---- Every cycle: bounds & linear term ---- */
    for (i = 0; i < Np; i++) {
        lb[i*Ny+0] = y_min[0];   ub[i*Ny+0] = y_max[0];
        lb[i*Ny+1] = y_min[1];   ub[i*Ny+1] = y_max[1];
    }
    for (i = 0; i < Nm; i++) {
        lb[Np*Ny+i] = u_min_val;
        ub[Np*Ny+i] = u_max_val;
    }

    /* g = -H * [ys1×Np; ys2×Np; us×Nm] */
    for (i = 0; i < Np; i++) {
        g_vec[i*Ny+0] = -2.0f * Q_diag[0] * ys_ref[0];
        g_vec[i*Ny+1] = -2.0f * Q_diag[1] * ys_ref[1];
    }
    for (i = 0; i < Nm; i++) {
        g_vec[Np*Ny+i] = -2.0f * Rs * us_ref;
    }
    /* Δu 惩罚一次项: Δu0=u0-prev_u 的展开 -2·H_du·prev_u·u0 */
    g_vec[Np*Ny+0] += -2.0f * DU_WEIGHT_H * prev_u;

    for (i = 0; i < NC; i++) lbA[i] = ubA[i] = g_be[i];

    if (qp == 0) return -1;

    nWSR = 80;

    retval = (qpInited == 1)
        ? qp->init(H, g_vec, Aeq, lb, ub, lbA, ubA, nWSR, &cputime)
        : qp->hotstart(g_vec, lb, ub, lbA, ubA, nWSR, &cputime);

    qpInited = 2;

    /* QP 失败时回退: 统一用 us_ref 兜底 (SSTC 针对当前负载+目标的稳态值,
       比上一拍的 last_u_opt 更安全, 尤其负载切换后) */
    if (retval == SUCCESSFUL_RETURN) {
        qp->getPrimalSolution(x_opt);
        for (j = 0; j < Nm; j++) {
            u_opt[j] = x_opt[Np * Ny + j];
            last_u_opt[j] = u_opt[j];
        }
        last_valid = 1;
        *obj_val = qp->getObjVal();
        prev_u = u_opt[0];          /* 更新上一拍实际占空比 */
    } else {
        for (j = 0; j < Nm; j++) u_opt[j] = us_ref;
        *obj_val = 0.0f;
        prev_u = us_ref;            /* 失败时也用 us_ref 作为上一拍基准 */
    }

    *iter      = nWSR;
    *exit_flag = getSimpleStatus(retval);
    return (*exit_flag == 0) ? 0 : 1;
}
