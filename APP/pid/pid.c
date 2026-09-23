/**
 * pid.c — 增量式 PID 电流控制
 *
 * 公式:  Δu = Kp·(e_k − e_{k−1}) + Ki·e_k + Kd·(e_k − 2·e_{k−1} + e_{k−2})
 *        u_k = u_{k−1} + Δu   (clamped to [OUT_MIN, OUT_MAX])
 *
 * 参考: 资料/buck/控buck电流/usbdaq_v12.m
 */

#include "pid.h"

static float32 Kp, Ki, Kd;
static float32 prev_error;     /* e_{k-1} */
static float32 prev_error2;    /* e_{k-2} */
static float32 prev_output;    /* u_{k-1} (duty cycle) */

void PID_Init(void)
{
    Kp = PID_KP_DEFAULT;
    Ki = PID_KI_DEFAULT;
    Kd = PID_KD_DEFAULT;
    prev_error  = 0.0f;
    prev_error2 = 0.0f;
    prev_output = 0.0f;
}

void PID_Reset(void)
{
    prev_error  = 0.0f;
    prev_error2 = 0.0f;
    prev_output = 0.1f;   /* restart from 10% duty */
}

void PID_Controller(float32 target, float32 measured, float32 *duty)
{
    float32 error = target - measured;

    /* 抗积分饱和(条件积分): 输出已饱和且误差仍朝饱和方向时, 冻结积分项,
       避免切换负载/长时间饱和后积分累积, 回程时产生大的反向超调 */
    Uint16 freeze_integral =
        ((prev_output >= PID_OUT_MAX) && (error > 0.0f)) ||
        ((prev_output <= PID_OUT_MIN) && (error < 0.0f));

    /* 增量式 PID */
    float32 delta_u = Kp * (error - prev_error)
                    + Kd * (error - 2.0f * prev_error + prev_error2);
    if (!freeze_integral)
        delta_u += Ki * error;

    prev_output += delta_u;
    if (prev_output < PID_OUT_MIN) prev_output = PID_OUT_MIN;
    if (prev_output > PID_OUT_MAX) prev_output = PID_OUT_MAX;

    prev_error2 = prev_error;
    prev_error  = error;

    *duty = prev_output;
}
