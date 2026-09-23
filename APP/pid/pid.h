/**
 * pid.h — 增量式 PID 电流控制器
 *
 * 参考 MATLAB: 资料/buck/控buck电流/usbdaq_v12.m
 *   Kp=2.0, Ki=0.5, Kd=0.005 (增量式)
 *
 * 用法:
 *   PID_Init();
 *   PID_Controller(target, current, &duty);
 *
 * 模式: 在 main.c 中通过 CONTROLLER_TYPE 切换 MPC/PID。
 */

#ifndef PID_H_
#define PID_H_

#include "DSP2833x_Device.h"

/* ---- PID 参数 (电流环, 采样 4ms, 按 BUCK 对象整定) ----
 * 对象: L·di/dt ≈ UF·duty → 离散 G(z) = (UF·T/L)/(z-1), K≈40
 * 整定: PI 闭环极点配置, 带宽 30Hz, 临界阻尼(ζ=1)
 *   p  = exp(-2π·30·0.004) ≈ 0.47
 *   Kp = (1-p²)/K ≈ 0.02   Ki = (1-p)²/K ≈ 0.008
 * 微分: 纯积分对象不需要, 置 0 (避免放大电流采样噪声)
 */
#define PID_KP_DEFAULT   0.02f
#define PID_KI_DEFAULT   0.008f
#define PID_KD_DEFAULT   0.0f
#define PID_OUT_MIN      0.0f
#define PID_OUT_MAX      0.99f

void PID_Init(void);
void PID_Controller(float32 target, float32 measured, float32 *duty);
void PID_Reset(void);

#endif /* PID_H_ */
