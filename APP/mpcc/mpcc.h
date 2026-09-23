/**
 * mpcc.h — Dual-output MPC with SSTC
 * ====================================
 * Paper:  "Constrained Model Predictive Control for Current Regulation
 *          of DC-DC Converter for Wireless Power Transfer System"
 * MATLAB: test5/test9 series  (C=[1 0;0 1], Q=0.5·I₂, R=1)
 *
 * Outputs:  y1 = U_L (voltage),  y2 = I_L (current)
 * State:    x = [U_L, I_L]^T
 * Np=5, Nc=3  →  z = [y1×5; y2×5; u×3] = 13 variables
 */

#ifndef MPCC_H_
#define MPCC_H_

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

#define CONTROL_MODE_CURRENT        0
#define CONTROL_MODE_VOLTAGE        1

#define MPCC_PREDICTION_HORIZON     5     /* Np */
#define MPCC_CONTROL_HORIZON        3     /* Nc */
#define MPCC_SAMPLING_TIME          0.00001f    /* 100µs, 一阶欧拉离散化保留阻尼(λ实部=0.5) */

#define BUCK_RL                     10.0f
#define BUCK_C                      10e-6f
#define BUCK_L                      3e-3f
#define BUCK_UF                     20.0f   /* 输入侧母线电压 */

/* Output dimension: 2 (voltage + current) */
#define MPCC_NY                     2

extern volatile Uint16 control_mode;

/* Target inputs from main */
extern float32 current_target;
extern float32 voltage_target;

/* Monitor variables */
extern volatile float32 monitor_mpcc_target;
extern volatile float32 monitor_mpcc_error;
extern volatile float32 monitor_mpcc_duty_cycle;
extern volatile float32 monitor_mpcc_qp_status;
extern volatile float32 monitor_state_voltage;
extern volatile float32 monitor_state_current;

extern volatile float32 monitor_be[10];
extern volatile float32 monitor_mp_row0[3];
extern volatile float32 monitor_ff_u[3];
extern volatile float32 monitor_xu_raw[3];
extern volatile float32 monitor_qp_iter;
extern volatile float32 monitor_qp_exit;

void MPCC_Init(void);
void MPCC_RecomputeModel(float32 newRL);  /* RL变化时重算预测矩阵 */
void MPCC_Controller(float32 current_tgt, float32 voltage_tgt,
                     float32 voltage, float32 current, float32 *duty_cycle);
void MPCC_SetControlMode(Uint16 mode);
Uint16 MPCC_GetControlMode(void);

#endif
