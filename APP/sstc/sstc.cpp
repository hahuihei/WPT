/**
 * sstc.cpp — Dual-Output SSTC via qpOASES QProblem
 * ==================================================
 * MATLAB: quadprog(Gs,gs,[],[],Aes,bes,lbs,ubs)
 *   3 variables [y1; y2; u], 2 equalities y=K*u
 *
 *   Gs = 2*[Q    0; 0 R]   (3×3),  gs = -Gs*[yt; ut]
 *   Aes = [1 0 -K1; 0 1 -K2],  bes = [0;0]
 *   lbs = [y_min; u_min], ubs = [y_max; u_max]
 *
 * K = C*(I-A)^-1*B,  C=I => K = [UF; UF/RL] = [20; 2]
 */

#include <qpOASES/QProblem.hpp>
#include <qpOASES/Options.hpp>
#include <qpOASES/MessageHandling.hpp>

USING_NAMESPACE_QPOASES

extern "C" {
#include "sstc.h"
#include "../mpcc/mpcc.h"
}

/* DC gains: K[0]=UF (voltage), K[1]=UF/RL (current) */
static float32 K[2] = {0.0f, 0.0f};
static int     sstc_inited = 0;

/* 松弛变量 η (软约束, 论文式16-17, Rao1999): η = [η_v, η_i] 二维
   软约束 |yt - yss| ≤ η 对所有输出(电压+电流)软化
   J += q_v·η_v + q_i·η_i + Q_v·η_v² + Q_i·η_i²
   注: 占空比变化率惩罚(Δu)在 MPCC 动态层(式18第三项), 不在此层 */
#define SSTC_ETA_QV   0.5f   /* 电压 η_v 二次惩罚权重 */
#define SSTC_ETA_QI   0.5f   /* 电流 η_i 二次惩罚权重 */
#define SSTC_ETA_LINV 0.8f   /* 电压 η_v 线性惩罚权重 q_v (论文 q=0.8) */
#define SSTC_ETA_LINI 0.8f   /* 电流 η_i 线性惩罚权重 q_i (论文 q=0.8) */

/* SSTC 开关: 1=启用 SSTC 软约束优化, 0=直接用理想占空比(调试用) */
#define SSTC_ENABLE  0

/* η 软约束激活判定阈值: 大于此值视为激活(远大于数值毛刺 1e-4) */
#define ETA_ACTIVE_TH  0.01f
#define ETA_HIST_LEN   20     /* 历史记录长度(记录最近20次激活的峰值) */

/* Monitor */
volatile float32 monitor_sstc_Kv      = 0.0f;
volatile float32 monitor_sstc_Ki      = 0.0f;
volatile float32 monitor_sstc_ys_v    = 0.0f;
volatile float32 monitor_sstc_ys_i    = 0.0f;
volatile float32 monitor_sstc_us      = 0.0f;
volatile float32 monitor_sstc_obj     = 0.0f;
volatile float32 monitor_sstc_eta_v   = 0.0f;   /* 电压松弛 η_v */
volatile float32 monitor_sstc_eta_i   = 0.0f;   /* 电流松弛 η_i */

/* η 松弛激活监控: 记录每次激活的峰值(历史数组) */
volatile Uint32  monitor_eta_v_active_cnt = 0;   /* η_v 激活累计次数 */
volatile Uint32  monitor_eta_i_active_cnt = 0;   /* η_i 激活累计次数 */
volatile float32 monitor_eta_v_hist[ETA_HIST_LEN] = {0};  /* η_v 每次激活峰值 */
volatile float32 monitor_eta_i_hist[ETA_HIST_LEN] = {0};  /* η_i 每次激活峰值 */
volatile Uint16  monitor_eta_v_hist_idx = 0;   /* 下一个写入位置 */
volatile Uint16  monitor_eta_i_hist_idx = 0;
static float32   eta_v_prev = 0.0f;              /* 上一拍 η_v (边沿检测) */
static float32   eta_i_prev = 0.0f;              /* 上一拍 η_i (边沿检测) */
static float32   eta_v_peak_cur = 0.0f;          /* 当前激活期间的峰值 */
static float32   eta_i_peak_cur = 0.0f;

/* Independent QProblem: 5 vars [y1,y2,u,eta_v,eta_i], 2 equalities + 4 soft constraints */
static QProblem     *sstc_qp = 0;
static Options       sstc_opt;
static float         sstc_H[25];     /* 5×5 = 25 */
static float         sstc_A[30];     /* 6×5 = 30 */
static float         sstc_lb[5], sstc_ub[5];
static float         sstc_lbA[6], sstc_ubA[6];
static float         sstc_x[5];
static int           sstc_phase = 0; /* 0=need alloc, 1=need init, 2=hotstart */

/* ================================================================
 *  SSTC_Init / SSTC_SetRL — compute K = C*(I-A)^-1*B
 * ================================================================ */
static void SSTC_CalcK(float32 RL)
{
    float32 T  = MPCC_SAMPLING_TIME;
    float32 Ci = BUCK_C;
    float32 L  = BUCK_L;
    float32 Uf = BUCK_UF;

    float32 A1[2][2] = {{-1.0f/(RL*Ci), 1.0f/Ci}, {-1.0f/L, 0.0f}};
    float32 A[2][2]  = {{1.0f + A1[0][0]*T, A1[0][1]*T},
                         {A1[1][0]*T,       1.0f + A1[1][1]*T}};
    float32 B_vec[2] = {0.0f, T * Uf / L};

    float32 a = 1.0f - A[0][0];   float32 bv = -A[0][1];
    float32 c = -A[1][0];          float32 d = 1.0f - A[1][1];
    float32 det = a*d - bv*c;
    if (det < 1e-12f && det > -1e-12f) det = 1e-12f;

    K[0] = ( d*B_vec[0] + (-bv)*B_vec[1]) / det;
    K[1] = ((-c)*B_vec[0] +  a*B_vec[1]) / det;
}

extern "C" void SSTC_Init(void)
{
    SSTC_CalcK(BUCK_RL);
    sstc_inited = 1;
    monitor_sstc_Kv = K[0];
    monitor_sstc_Ki = K[1];
}

/* 负载电阻变化时调用: 重算K, 丢弃旧QP, 下次自动重建 */
extern "C" void SSTC_SetRL(float32 newRL)
{
    SSTC_CalcK(newRL);
    if (sstc_qp) { delete sstc_qp; sstc_qp = 0; }
    sstc_phase = 0;
    monitor_sstc_Kv = K[0];
    monitor_sstc_Ki = K[1];
}

/* ================================================================
 *  SSTC_Compute — QP: 3 vars, 2 equalities
 * ================================================================ */
extern "C" void SSTC_Compute(
    const float32 *yt, const float32 *Q_d, float32 Rs,
    float32 ut, const float32 *y_min, const float32 *y_max,
    float32 *ys, float32 *us)
{
    float32 u_min_v = 0.0f, u_max_v = 1.0f;

#if !SSTC_ENABLE
    /* 关掉 SSTC: 直接用理想占空比 ut 作为前馈(调试用) */
    *us = ut;
    if (*us > 1.0f) *us = 1.0f;  if (*us < 0.0f) *us = 0.0f;
    ys[0] = K[0] * (*us);
    ys[1] = K[1] * (*us);
    return;
#endif

    if (!sstc_inited) {
        *us = yt[1] * BUCK_RL / BUCK_UF;
        if (*us > 1.0f) *us = 1.0f;  if (*us < 0.0f) *us = 0.0f;
        ys[0] = K[0] * (*us);
        ys[1] = K[1] * (*us);
        return;
    }

    /* g = [0; 0; -2*R*ut; q_v; q_i]
       y1,y2 无二次跟踪项(论文式16), 输出跟踪完全由 η 软约束实现 */
    float g_vec[5];
    g_vec[0] = 0.0f;
    g_vec[1] = 0.0f;
    g_vec[2] = -2.0f * Rs * ut;
    g_vec[3] = SSTC_ETA_LINV;                       /* q_v·η_v */
    g_vec[4] = SSTC_ETA_LINI;                       /* q_i·η_i */

    if (sstc_qp == 0) {
        sstc_qp = new QProblem(5, 6);
        if (sstc_qp == 0) { *us = 0.5f; ys[0]=K[0]*0.5f; ys[1]=K[1]*0.5f; return; }
        sstc_opt.setToMPC();
        sstc_opt.printLevel = PL_NONE;
        sstc_qp->setOptions(sstc_opt);

        /* H = 2*diag(0, 0, R, Q_v, Q_i) */
        memset(sstc_H, 0, sizeof(sstc_H));
        sstc_H[0*5+0] = 0.0f;
        sstc_H[1*5+1] = 0.0f;
        sstc_H[2*5+2] = 2.0f * Rs;
        sstc_H[3*5+3] = 2.0f * SSTC_ETA_QV;
        sstc_H[4*5+4] = 2.0f * SSTC_ETA_QI;

        /* A: 行0-1 等式(y=K*u), 行2-5 软约束 |yt-y|<=eta (电压+电流) */
        memset(sstc_A, 0, sizeof(sstc_A));
        sstc_A[0*5+0] = 1.0f;  sstc_A[0*5+2] = -K[0];   /* y1 - K0*u = 0 */
        sstc_A[1*5+1] = 1.0f;  sstc_A[1*5+2] = -K[1];   /* y2 - K1*u = 0 */
        sstc_A[2*5+0] = -1.0f; sstc_A[2*5+3] = -1.0f;   /* -y1 - eta_v <= -yt1 */
        sstc_A[3*5+0] = 1.0f;  sstc_A[3*5+3] = -1.0f;   /*  y1 - eta_v <=  yt1 */
        sstc_A[4*5+1] = -1.0f; sstc_A[4*5+4] = -1.0f;   /* -y2 - eta_i <= -yt2 */
        sstc_A[5*5+1] = 1.0f;  sstc_A[5*5+4] = -1.0f;   /*  y2 - eta_i <=  yt2 */

        /* bounds: y1,y2,u 硬约束, eta_v,eta_i >= 0 */
        sstc_lb[0] = y_min[0];  sstc_ub[0] = y_max[0];
        sstc_lb[1] = y_min[1];  sstc_ub[1] = y_max[1];
        sstc_lb[2] = u_min_v;    sstc_ub[2] = u_max_v;
        sstc_lb[3] = 0.0f;       sstc_ub[3] = 1e10f;
        sstc_lb[4] = 0.0f;       sstc_ub[4] = 1e10f;

        /* lbA/ubA: 等式=0, 软约束上界随目标变化 */
        sstc_lbA[0] = sstc_ubA[0] = 0.0f;
        sstc_lbA[1] = sstc_ubA[1] = 0.0f;
        sstc_lbA[2] = -1e10f;  sstc_ubA[2] = -yt[0];
        sstc_lbA[3] = -1e10f;  sstc_ubA[3] =  yt[0];
        sstc_lbA[4] = -1e10f;  sstc_ubA[4] = -yt[1];
        sstc_lbA[5] = -1e10f;  sstc_ubA[5] =  yt[1];

        int nWSR = 10; float cpu = 0.0f;
        sstc_qp->init(sstc_H, g_vec, sstc_A,
                      sstc_lb, sstc_ub, sstc_lbA, sstc_ubA, nWSR, &cpu);
        sstc_phase = 2;
        sstc_qp->getPrimalSolution(sstc_x);
    } else if (sstc_phase == 2) {
        /* 每拍更新软约束上界 (目标变化时) */
        sstc_ubA[2] = -yt[0];
        sstc_ubA[3] =  yt[0];
        sstc_ubA[4] = -yt[1];
        sstc_ubA[5] =  yt[1];
        int nWSR = 50; float cpu = 0.0f;
        sstc_qp->hotstart(g_vec, sstc_lb, sstc_ub, sstc_lbA, sstc_ubA, nWSR, &cpu);
        sstc_qp->getPrimalSolution(sstc_x);
    }

    ys[0] = sstc_x[0];
    ys[1] = sstc_x[1];
    *us   = sstc_x[2];

    if (*us > u_max_v) *us = u_max_v;  if (*us < u_min_v) *us = u_min_v;

    monitor_sstc_ys_v = ys[0];  monitor_sstc_ys_i = ys[1];
    monitor_sstc_us   = *us;    monitor_sstc_obj = sstc_qp->getObjVal();
    monitor_sstc_eta_v = sstc_x[3];
    monitor_sstc_eta_i = sstc_x[4];
    if (monitor_sstc_eta_v < 0.0f) monitor_sstc_eta_v = 0.0f;  /* η≥0, 消除数值毛刺 */
    if (monitor_sstc_eta_i < 0.0f) monitor_sstc_eta_i = 0.0f;

    /* 检测 η 激活边沿, 记录每次激活的峰值到历史数组 */
    if (monitor_sstc_eta_v > ETA_ACTIVE_TH) {
        if (eta_v_prev <= ETA_ACTIVE_TH) {
            monitor_eta_v_active_cnt++;                     /* 上升沿: 新一次激活 */
            eta_v_peak_cur = monitor_sstc_eta_v;
        } else if (monitor_sstc_eta_v > eta_v_peak_cur) {
            eta_v_peak_cur = monitor_sstc_eta_v;            /* 更新峰值 */
        }
    } else if (eta_v_prev > ETA_ACTIVE_TH) {
        /* 下降沿: 激活结束, 峰值存入历史(循环写入) */
        monitor_eta_v_hist[monitor_eta_v_hist_idx % ETA_HIST_LEN] = eta_v_peak_cur;
        monitor_eta_v_hist_idx++;
    }
    eta_v_prev = monitor_sstc_eta_v;

    if (monitor_sstc_eta_i > ETA_ACTIVE_TH) {
        if (eta_i_prev <= ETA_ACTIVE_TH) {
            monitor_eta_i_active_cnt++;                     /* 上升沿: 新一次激活 */
            eta_i_peak_cur = monitor_sstc_eta_i;
        } else if (monitor_sstc_eta_i > eta_i_peak_cur) {
            eta_i_peak_cur = monitor_sstc_eta_i;            /* 更新峰值 */
        }
    } else if (eta_i_prev > ETA_ACTIVE_TH) {
        monitor_eta_i_hist[monitor_eta_i_hist_idx % ETA_HIST_LEN] = eta_i_peak_cur;
        monitor_eta_i_hist_idx++;
    }
    eta_i_prev = monitor_sstc_eta_i;
}

extern "C" float32 SSTC_Gain(Uint16 idx) { return (idx < 2) ? K[idx] : 0.0f; }
