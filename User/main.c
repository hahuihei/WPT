#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "adc.h"
#include "pwm.h"
#include "mpcc.h"
#include "../APP/pid/pid.h"
#include <string.h>

#define CTL_MPC   1

/* PI 周期对齐 MPC: 置 1 时 PI 每拍结束后忙等补齐到 TARGET_CYCLE_US, 使控制周期与 MPC 一致;
   置 0 时立即关闭延迟(恢复快速 PI) */
#define PI_MATCH_MPC       0
#define TARGET_CYCLE_US    1370   /* 目标周期(µs), 取 MPC 模式下 monitor_control_cycle 的值 */

/* 负载检测去抖: 连续 RL_VOTE_N 拍一致才切换, 切换后锁存 RL_LOCK_M 拍不再检测 */
#define RL_VOTE_N   2     /* 连续一致拍数 */
#define RL_LOCK_M   20    /* 切换后锁存拍数 */

#define EXPERIMENT_STEP_CURRENT  0

#if EXPERIMENT_STEP_CURRENT
#define STEP_LOW         2.0f
#define STEP_HIGH        1.5f
#define STEP_DURATION    1.0f
static float32 step_sequence[] = { STEP_LOW, STEP_HIGH };
#define STEP_NUM  (sizeof(step_sequence) / sizeof(step_sequence[0]))
#endif

#define SYSTEM_READY    0
#define SYSTEM_RUNNING  1

volatile Uint16 system_state = SYSTEM_READY;
volatile Uint16 control_mode = CONTROL_MODE_CURRENT;

/* === Dual-output targets (matching paper SSTC) === */
float32 current_target = 1.50f;       /* yt[1] — current tracking */
float32 voltage_target = 20.0f;    /* = current * RL, 切负载时同步更新 */

/* ---- monitor ---- */
volatile Uint32  monitor_system_uptime     = 0;
volatile float32 monitor_global_Ub         = 0.0f;
volatile float32 monitor_global_iB         = 0.0f;
volatile float32 monitor_global_iB_filtered= 0.0f;
volatile float32 monitor_global_us         = 0.0f;
volatile float32 monitor_control_cycle     = 0.0f;    /* 整个控制周期耗时(微秒) */
extern volatile float32 monitor_mpcc_duty_cycle;
extern volatile float32 monitor_mpcc_us_ref;
extern volatile float32 monitor_mpcc_delta_duty;

/* === 数据导出用历史数组 (CCS Memory Browser 导出到 Excel) === */
#define LOG_N 100
volatile float32 monitor_sstc_time_us_hist[LOG_N];    /* SSTC 耗时历史(微秒) */
volatile float32 monitor_mpcc_qp_us_hist[LOG_N];      /* MPCC QP 耗时历史(微秒) */
volatile float32 monitor_control_cycle_hist[LOG_N];   /* 控制周期历史(微秒) */
volatile Uint16  log_index = 0;   /* 已记录点数 */
volatile Uint16  log_full  = 0;   /* =1 表示已存满 100 点 */

/* SSTC 运行时 RL 更新 (sstc.h) */
extern void SSTC_SetRL(float32 newRL);

void delay_ms(Uint32 ms)
{
    Uint32 i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 15000; j++)  asm(" NOP");
}

static float32 EstimateCycleTime(void) { return 0.001f; }

void main(void)
{
    float32 voltage, current, duty;
    float32 current_filtered = 0.0f;

#if EXPERIMENT_STEP_CURRENT
    Uint16 step_idx       = 0;
    float32 step_elapsed  = 0.0f;
    float32 cycle_time    = EstimateCycleTime();
    current_target        = step_sequence[0];
#endif

    InitSysCtrl();

    /* 将 ramfuncs 段(含 InitFlash)从 FLASH 拷贝到 RAM, 然后初始化 FLASH 等待周期 */
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadEnd - (Uint32)&RamfuncsLoadStart);
    InitFlash();

    InitGpio();

    /* 启动 CpuTimer2 作自由运行计数器(150MHz), 测量控制周期耗时 */
    CpuTimer2Regs.TPR.all   = 0;
    CpuTimer2Regs.TPRH.all  = 0;
    CpuTimer2Regs.PRD.all   = 0xFFFFFFFF;
    CpuTimer2Regs.TCR.bit.TRB = 1;   /* 重载 */
    CpuTimer2Regs.TCR.bit.TSS = 0;   /* 启动 */

    MPCC_Init();
    MPCC_SetControlMode(control_mode);
    PID_Init();

    PWM_GPIO_Init();
    PWM_Init();
    ADC_Init();

    duty = 0.0f;
    PWM_UpdateDutyCycle(duty);
    PWM_Start();
    delay_ms(500);

    ADC_MeasureCurrentBias();

    duty = 0.1f;
    PWM_UpdateDutyCycle(duty);
    system_state = SYSTEM_RUNNING;

    while (1)
    {
        Uint32 t_start = CpuTimer2Regs.TIM.all;   /* 采样前读计数 */

        ADC_ReadBlocking(&voltage, &current);
        current_filtered = current;

#if EXPERIMENT_STEP_CURRENT
        step_elapsed += cycle_time;
        if (step_elapsed >= STEP_DURATION) {
            step_elapsed = 0.0f;
            step_idx++;
            if (step_idx >= STEP_NUM) step_idx = 0;
            current_target = step_sequence[step_idx];
        }
#endif

        float32 target = current_target;

#if CTL_MPC
        /* 检测负载电阻(带去抖), 变化时更新 SSTC K 值和电压目标 */
        {
            static float32 detected_RL = BUCK_RL;   /* 当前生效负载 */
            static float32 candidate_RL = BUCK_RL;  /* 候选负载 */
            static Uint16  vote_count = 0;          /* 连续一致计数 */
            static Uint16  lock_count = 0;          /* 切换后锁存计数 */
            float32 r_eff;

            if (current_filtered > 0.05f) {
                r_eff = voltage / current_filtered;
                /* 判断阻值区间 */
                if      (r_eff >= 3.5f && r_eff <= 7.5f)  r_eff = 5.0f;
                else if (r_eff >= 7.5f && r_eff <= 12.5f) r_eff = 10.0f;
                else if (r_eff >= 12.5f && r_eff <= 17.5f) r_eff = 15.0f;
                else                                       r_eff = detected_RL; /* 保持 */
            } else {
                r_eff = detected_RL;
            }

            if (lock_count > 0) {
                lock_count--;                        /* 锁存期: 只倒计时, 不检测 */
            } else if (r_eff != detected_RL) {
                if (r_eff == candidate_RL) {
                    if (++vote_count >= RL_VOTE_N) {  /* 连续 N 拍一致才切换 */
                        detected_RL = candidate_RL;
                        SSTC_SetRL(detected_RL);          /* 重算稳态增益 K */
                        MPCC_RecomputeModel(detected_RL); /* 重算预测模型矩阵 */
                        vote_count = 0;
                        lock_count = RL_LOCK_M;           /* 切换后锁存 M 拍 */
                    }
                } else {
                    candidate_RL = r_eff;            /* 换候选, 重新计数 */
                    vote_count = 1;
                }
            } else {
                vote_count = 0;                      /* 回到当前值, 清零投票 */
                candidate_RL = detected_RL;
            }

            voltage_target = current_target * detected_RL;
        }
        MPCC_Controller(current_target, voltage_target, voltage, current_filtered, &duty);
#else
        PID_Controller(target, current_filtered, &duty);
#endif
        PWM_UpdateDutyCycle(duty);

#if !CTL_MPC && PI_MATCH_MPC
        /* PI 模式: 忙等补齐到目标周期, 让控制周期与 MPC 一致(关掉: PI_MATCH_MPC=0) */
        {
            Uint32 target_ticks = (Uint32)(TARGET_CYCLE_US * 150.0f);
            while ((t_start - CpuTimer2Regs.TIM.all) < target_ticks) { asm(" NOP"); }
        }
#endif

        /* 测量本拍控制周期耗时(采样→PWM更新) */
        monitor_control_cycle = (float32)(t_start - CpuTimer2Regs.TIM.all) / 150.0f;   /* 150MHz: 1µs=150ticks */

        /* 记录到导出数组(各100点), 存满后置 log_full */
        if (log_index < LOG_N) {
#if CTL_MPC
            monitor_sstc_time_us_hist[log_index]       = monitor_sstc_time_us;
            monitor_mpcc_qp_us_hist[log_index]    = monitor_mpcc_qp_us;
#endif
            monitor_control_cycle_hist[log_index] = monitor_control_cycle;
            log_index++;
            if (log_index >= LOG_N) log_full = 1;
        }

        monitor_system_uptime++;
        monitor_global_Ub          = voltage;
        monitor_global_iB          = current;
        monitor_global_iB_filtered = current_filtered;
        monitor_global_us          = duty;
    }
}
