#ifndef ADC_H_
#define ADC_H_

#include "DSP2833x_Device.h"

/* ADC 参数 */
#define BUF_SIZE      2    /* 通道数：电压 + 电流 */
#define SAMPLE_NUM    40    /* 每通道采集点数（减少点数缩短控制周期，噪声用一阶滤波缓解） */

/* 中断采集后的平均 raw ADC 值 (每次攒满 SAMPLE_NUM 后更新) */
extern volatile float32 ADValue[BUF_SIZE];

/* 转换后的物理量 */
extern volatile float32 adc_voltage;
extern volatile float32 adc_current;

/* ADC 函数 */
void ADC_Init(void);
void ADC_EnableInterrupt(void);  /* bias 测量完成后调用，使能 ISR */
void ADC_GetAveraged(float32 *voltage, float32 *current);  /* 非阻塞读取 */
Uint16 ADC_NewDataReady(void);   /* 有新数据返回 1，同时清零 */
void ADC_ReadBlocking(float32 *voltage, float32 *current);  /* 阻塞式 200 次平均 */
float32 ADC_GetVoltage(void);
float32 ADC_GetCurrent(void);
void ADC_MeasureCurrentBias(void);

/* 监测变量 */
extern volatile float32 monitor_adc_raw_voltage;
extern volatile float32 monitor_adc_raw_current;
extern volatile float32 monitor_adc_voltage;
extern volatile float32 monitor_adc_current;
extern volatile float32 monitor_adc_current_bias;
extern volatile Uint32  monitor_adc_sample_count;
extern volatile Uint32  monitor_adc_timeout_count;
extern volatile Uint16  monitor_adc_st;
extern volatile Uint16  monitor_adc_result0;
extern volatile Uint16  monitor_adc_result1;

#endif /* ADC_H_ */
