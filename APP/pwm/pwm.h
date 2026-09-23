#ifndef PWM_H_
#define PWM_H_

#include "DSP2833x_Device.h"

// PWM函数声明
void PWM_Init(void);
void PWM_UpdateDutyCycle(float32 duty_cycle);
float32 PWM_GetDutyCycle(void);
void PWM_Start(void);
void PWM_Stop(void);
void PWM_GPIO_Init(void);

// 外部变量声明
extern volatile float32 pwm_duty;

#endif /* PWM_H_ */
