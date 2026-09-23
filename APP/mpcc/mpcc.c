/**
 * mpcc.c — Dual-Output MPC with SSTC
 * ====================================
 * MATLAB: test5/test9 series  (C=[1 0;0 1], Q=0.5·I₂, R=1)
 * Model: 预测矩阵随负载RL动态更新, 通过MPCC_RecomputeModel()触发
 */

#include "mpcc.h"
#include "../qp/qpoases_port.h"
#include "../sstc/sstc.h"

#define Np  MPCC_PREDICTION_HORIZON   /* 5 */
#define Nm  MPCC_CONTROL_HORIZON      /* 3 */
#define Ny  MPCC_NY                   /* 2 */

static float32 x[2];
static float32 A[2][2];
static float32 B[2];
static float32 C[2][2] = {{1.0f,0.0f},{0.0f,1.0f}};
static float32 D[2]   = {0.0f, 0.0f};

static float32 Q_diag[2] = {0.0f, 1.0f};   /* 电压Q=0仅约束, 只控电流 */
static float32 R_scalar  = 20.0f;

static float32 y_min[2] = {0.0f, 0.0f};
static float32 y_max[2] = {40.0f, 3.5f};
static float32 u_min    = 0.0f;
static float32 u_max    = 1.0f;

static float32 g_MP[Np*Ny][Nm];
static float32 g_MM[Np*Ny][2];
static float32 g_be[Np*Ny];

volatile float32 monitor_mpcc_target      = 0.0f;
volatile float32 monitor_mpcc_error       = 0.0f;
volatile float32 monitor_mpcc_duty_cycle  = 0.0f;
volatile float32 monitor_mpcc_us_ref      = 0.0f;
volatile float32 monitor_mpcc_delta_duty  = 0.0f;
volatile float32 monitor_mpcc_qp_status   = 0.0f;
volatile float32 monitor_state_voltage    = 0.0f;
volatile float32 monitor_state_current    = 0.0f;

volatile float32 monitor_be[10]    = {0};
volatile float32 monitor_mp_row0[3] = {0};
volatile float32 monitor_ff_u[3]   = {0};
volatile float32 monitor_xu_raw[3] = {0};
volatile float32 monitor_qp_iter   = 0.0f;
volatile float32 monitor_qp_exit   = 0.0f;

static float32 ys_ref[2];
static float32 us_ref;


static void PreCompute(float32 RL)
{
    Uint16 i, j, k, p;
    float32 T  = MPCC_SAMPLING_TIME;
    float32 Ci = BUCK_C;
    float32 L  = BUCK_L;
    float32 Uf = BUCK_UF;

    float32 A1[2][2] = {{-1.0f/(RL*Ci), 1.0f/Ci}, {-1.0f/L, 0.0f}};
    A[0][0] = 1.0f + A1[0][0]*T;  A[0][1] = A1[0][1]*T;
    A[1][0] = A1[1][0]*T;          A[1][1] = 1.0f + A1[1][1]*T;
    B[0] = 0.0f;  B[1] = T * Uf / L;

    float32 Ap[Np][2][2];
    Ap[0][0][0]=1.0f; Ap[0][0][1]=0.0f; Ap[0][1][0]=0.0f; Ap[0][1][1]=1.0f;
    for (i=1; i<Np; i++)
        for (j=0; j<2; j++) for (k=0; k<2; k++) {
            Ap[i][j][k] = 0.0f;
            for (p=0; p<2; p++) Ap[i][j][k] += A[j][p] * Ap[i-1][p][k];
        }

    for (i=0; i<Np; i++) for (j=0; j<2; j++) {
        g_MM[i*Ny+0][j] = Ap[i][0][j];
        g_MM[i*Ny+1][j] = Ap[i][1][j];
    }

    float32 ABAB[Np][2];
    for (i=0; i<Np; i++) {
        ABAB[i][0] = 0.0f;  ABAB[i][1] = 0.0f;
        for (j=0; j<2; j++) for (k=0; k<2; k++) ABAB[i][j] += Ap[i][j][k] * B[k];
    }

    float32 AB1[2*Np][Np];
    for (i=0; i<2*Np; i++) for (j=0; j<Np; j++) AB1[i][j] = 0.0f;
    for (p=0; p<Np; p++) for (i=p; i<Np; i++) {
        AB1[i*2+0][i-p] = ABAB[p][0];
        AB1[i*2+1][i-p] = ABAB[p][1];
    }

    float32 AB2[2*Np][Nm];
    for (i=0; i<2*Np; i++) for (j=0; j<Nm; j++) AB2[i][j] = AB1[i][j];
    if (Np > Nm) {
        float32 v[2*Np];
        for (i=0; i<2*Np; i++) v[i] = 0.0f;
        for (j=Nm; j<Np; j++) for (k=0; k<2*Np; k++) v[k] += AB1[k][j];
        for (k=0; k<2*Np; k++) v[k] += AB1[k][Nm-1];
        for (k=0; k<2*Np; k++) AB2[k][Nm-1] = v[k];
    }
    for (i=0; i<2*Np; i++) for (j=0; j<Nm; j++) g_MP[i][j] = -AB2[i][j];
}


void MPCC_Init(void) { PreCompute(BUCK_RL); QPOASES_Init(); SSTC_Init(); }

/* RL变化时调用: 重算预测矩阵并重置QP, 使MPC模型与实际负载匹配 */
void MPCC_RecomputeModel(float32 newRL)
{
    PreCompute(newRL);
    QPOASES_Init();  /* 重置内部QP标志, 下次solve时重建H/Aeq */
}


static void MPC_SolveFullQP(float32 *u_opt, int *status)
{
    Uint16 i; int iter, exit_flag; float32 obj_val;
    int ret = QPOASES_Solve(
        (const float32 *)g_MP, g_be, Q_diag, R_scalar,
        ys_ref, us_ref, y_min, y_max, u_min, u_max,
        u_opt, &obj_val, &iter, &exit_flag);
    *status = (exit_flag == 0) ? 0 : 1;
    for (i=0; i<Np*Ny; i++) monitor_be[i] = g_be[i];
    for (i=0; i<Nm; i++)   monitor_mp_row0[i] = g_MP[0][i];
    monitor_ff_u[0] = u_opt[0];
    monitor_ff_u[1] = obj_val;
    monitor_ff_u[2] = (float32)iter;
    for (i=0; i<Nm; i++)   monitor_xu_raw[i] = u_opt[i];
    monitor_qp_iter = (float32)iter;
    monitor_qp_exit = (float32)exit_flag;
}


void MPCC_Controller(float32 current_tgt, float32 voltage_tgt,
                     float32 voltage, float32 current, float32 *duty_cycle)
{
    Uint16 i; int qp_status; float32 u_opt[Nm];
    x[0] = voltage;  x[1] = current;

    float32 yt[2];  yt[0] = voltage_tgt;  yt[1] = current_tgt;
    /* ut = 期望稳态占空比 = 目标电压/输入电压, 用 voltage_tgt 而非固定 BUCK_RL,
       否则负载切换后 ut 错误会把 SSTC 的电流推到约束边界 */
    SSTC_Compute(yt, Q_diag, R_scalar,
                 voltage_tgt/BUCK_UF,
                 y_min, y_max, ys_ref, &us_ref);

    for (i=0; i<Np*Ny; i++) g_be[i] = g_MM[i][0]*x[0] + g_MM[i][1]*x[1];
    MPC_SolveFullQP(u_opt, &qp_status);

    *duty_cycle = u_opt[0];
    if (*duty_cycle < u_min) *duty_cycle = u_min;
    if (*duty_cycle > u_max) *duty_cycle = u_max;

    monitor_mpcc_target     = ys_ref[1];
    monitor_mpcc_error      = ys_ref[1] - current;
    monitor_state_voltage   = voltage;
    monitor_state_current   = current;
    monitor_mpcc_duty_cycle = *duty_cycle;
    monitor_mpcc_us_ref     = us_ref;
    monitor_mpcc_delta_duty = *duty_cycle - us_ref;
    monitor_mpcc_qp_status  = (float32)qp_status;
}


void MPCC_SetControlMode(Uint16 mode)
{
    control_mode = mode;
    Q_diag[0] = 0.0f;  Q_diag[1] = 1.0f;  R_scalar =10.0f;
}

Uint16 MPCC_GetControlMode(void) { return control_mode; }
