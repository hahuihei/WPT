#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "pwm.h"

#define PWM_FREQ       100000
#define PWM_PERIOD     1500
#define PWM_DEAD_TIME  30

volatile float32 pwm_duty = 0.0f;

volatile float32 monitor_pwm_duty_cycle = 0.0f;
volatile Uint32  monitor_pwm_update_count = 0;
volatile float32 monitor_pwm_cmpa_value = 0.0f;

void PWM_Init(void)
{
    EALLOW;

    EPwm1Regs.TBPRD = PWM_PERIOD;
    EPwm1Regs.TBPHS.half.TBPHS = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm1Regs.CMPA.half.CMPA = (Uint16)(PWM_PERIOD * pwm_duty);
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.PRD = AQ_SET;

    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm1Regs.DBRED = PWM_DEAD_TIME;
    EPwm1Regs.DBFED = PWM_DEAD_TIME;

    EPwm1Regs.ETSEL.bit.SOCAEN = 0;

    EDIS;
}

void PWM_UpdateDutyCycle(float32 duty_cycle)
{
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 0.99f) duty_cycle = 0.99f;

    pwm_duty = duty_cycle;

    monitor_pwm_duty_cycle = duty_cycle;
    monitor_pwm_update_count++;
    monitor_pwm_cmpa_value = (float32)(PWM_PERIOD * duty_cycle);

    EALLOW;
    EPwm1Regs.CMPA.half.CMPA = (Uint16)(PWM_PERIOD * duty_cycle);
    EDIS;
}

float32 PWM_GetDutyCycle(void) { return pwm_duty; }

void PWM_Start(void)
{
    EALLOW;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EDIS;
}

void PWM_Stop(void)
{
    EALLOW;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_FREEZE;
    EDIS;
}

void PWM_GPIO_Init(void)
{
    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;

    EDIS;
}
