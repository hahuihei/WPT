/**
 * adc.c — ADC 中断驱动采集 (后台填 buffer, 200 点滚动平均)
 *
 * 参考: DSP2833x_temp/APP/adc/adc.c
 *
 * 架构:
 *   ADC 连续转换 (CONT_RUN) → 每次 EOS 触发 SEQ1 中断
 *   ISR 将原始值存入 SampleTable, 攒满 SAMPLE_NUM 后求平均更新 ADValue
 *   main 循环调用 ADC_GetAveraged() 非阻塞读取最新的物理量
 */

#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "adc.h"

#define ADC_CH_VOLTAGE  0
#define ADC_CH_CURRENT  1

#define VOLTAGE_SCALE   10.091f
#define CURRENT_SCALE   3.9916f  /* 换新BUCK后重标: 真实I=V/R=13.82/10=1.382A, 采样1.079A → 3.9916×1.382/1.079 */

#define ADC_REF_VOLTAGE 3.0f
#define ADC_MAX_VALUE   4095.0f

/* ---- 中断采集 buffer ---- */
volatile Uint16  SampleTable[BUF_SIZE][SAMPLE_NUM] = {{0}};
volatile float32 ADValue[BUF_SIZE] = {0.0f};
volatile Uint16  adc_data_ready = 0;  /* ISR 置 1，主循环消费后清 0 */

/* ---- 物理量 ---- */
volatile float32 adc_voltage = 0.0f;
volatile float32 adc_current = 0.0f;
volatile float32 adc_current_bias = 0.0f;

/* ---- 监测变量 ---- */
volatile float32 monitor_adc_raw_voltage   = 0.0f;
volatile float32 monitor_adc_raw_current   = 0.0f;
volatile float32 monitor_adc_voltage       = 0.0f;
volatile float32 monitor_adc_current       = 0.0f;
volatile float32 monitor_adc_current_bias  = 0.0f;
volatile Uint32  monitor_adc_sample_count  = 0;
volatile Uint32  monitor_adc_timeout_count = 0;
volatile Uint16  monitor_adc_st            = 0;
volatile Uint16  monitor_adc_result0       = 0;
volatile Uint16  monitor_adc_result1       = 0;

/* ---- ISR 声明 ---- */
__interrupt void adc_isr(void);

void ADC_Init(void)
{
    Uint32 i;

    EALLOW;
    SysCtrlRegs.HISPCP.all = 0x3;
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;

    /* 注册 ADC 中断到 PIE 向量表 (ADCINT = group 1, interrupt 6) */
    PieVectTable.ADCINT = &adc_isr;
    EDIS;

    AdcRegs.ADCTRL3.all = 0x00E0;
    for (i = 0; i < 1000000; i++) { asm(" NOP"); }

    EALLOW;
    AdcRegs.ADCTRL3.bit.ADCCLKPS = 0x4;
    AdcRegs.ADCTRL1.bit.CPS      = 0;
    AdcRegs.ADCTRL1.bit.ACQ_PS   = 0xF;
    AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;
    AdcRegs.ADCTRL1.bit.CONT_RUN = 1;
    AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1;

    AdcRegs.ADCMAXCONV.all = 0x0001;
    AdcRegs.ADCCHSELSEQ1.bit.CONV00 = ADC_CH_VOLTAGE;
    AdcRegs.ADCCHSELSEQ1.bit.CONV01 = ADC_CH_CURRENT;

    /* 中断暂不使能, bias 测量后再开 (ADC_EnableInterrupt) */
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 0;
    AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 0;
    EDIS;

    /* 启动连续转换 */
    AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;
}

void ADC_EnableInterrupt(void)
{
    /* 使能 ADC SEQ1 中断 */
    EALLOW;
    AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;
    EDIS;

    /* 使能 PIE group 1 interrupt 6 (ADCINT) */
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;

    /* 使能 CPU 中断 */
    IER |= M_INT1;
    EINT;
    ERTM;
}

/* ---- ADS 中断服务 ---- */
__interrupt void adc_isr(void)
{
    static Uint16 count = 0;
    Uint16 i, j;

    /* 读本次转换结果 */
    SampleTable[0][count] = (AdcRegs.ADCRESULT0 >> 4) & 0x0FFF;
    SampleTable[1][count] = (AdcRegs.ADCRESULT1 >> 4) & 0x0FFF;

    count++;
    if (count >= SAMPLE_NUM)
    {
        count = 0;

        /* 累加 200 点 */
        for (j = 0; j < BUF_SIZE; j++)
        {
            ADValue[j] = 0.0f;
            for (i = 0; i < SAMPLE_NUM; i++)
                ADValue[j] += (float32)SampleTable[j][i];
        }

        /* 平均 */
        for (j = 0; j < BUF_SIZE; j++)
            ADValue[j] = ADValue[j] / (float32)SAMPLE_NUM;

        adc_data_ready = 1;  /* 通知主循环数据已更新 */
        monitor_adc_sample_count++;
    }

    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

/* ---- 非阻塞读取：将 ADValue 转为物理量 ---- */
void ADC_GetAveraged(float32 *voltage, float32 *current)
{
    float32 raw_v = ADValue[0];
    float32 raw_c = ADValue[1];

    float32 adc_voltage_v = (raw_v / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float32 adc_current_v = (raw_c / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float32 adc_bias_v    = (adc_current_bias / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;

    *voltage = adc_voltage_v * VOLTAGE_SCALE;
    *current = (adc_current_v - adc_bias_v) * CURRENT_SCALE;

    adc_voltage = *voltage;
    adc_current = *current;

    monitor_adc_raw_voltage = raw_v;
    monitor_adc_raw_current = raw_c;
    monitor_adc_voltage     = *voltage;
    monitor_adc_current     = *current;
    monitor_adc_st          = AdcRegs.ADCST.all;
    monitor_adc_result0     = (AdcRegs.ADCRESULT0 >> 4) & 0x0FFF;
    monitor_adc_result1     = (AdcRegs.ADCRESULT1 >> 4) & 0x0FFF;
}

/* 主循环查询：有新数据时返回 1（同时自动清零） */
Uint16 ADC_NewDataReady(void)
{
    if (adc_data_ready) {
        adc_data_ready = 0;
        return 1;
    }
    return 0;
}

/* 阻塞式 ADC 采集：SAMPLE_NUM 次转换取平均 */
void ADC_ReadBlocking(float32 *voltage, float32 *current)
{
    Uint16 n;
    Uint32 v_sum = 0, c_sum = 0;

    for (n = 0; n < SAMPLE_NUM; n++)
    {
        Uint32 timeout = 0;
        while (AdcRegs.ADCST.bit.INT_SEQ1 == 0 && timeout < 100000)
            timeout++;
        if (timeout >= 100000) monitor_adc_timeout_count++;
        AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;

        v_sum += (Uint32)((AdcRegs.ADCRESULT0 >> 4) & 0x0FFF);
        c_sum += (Uint32)((AdcRegs.ADCRESULT1 >> 4) & 0x0FFF);
    }

    float32 avg_v = (float32)v_sum / (float32)SAMPLE_NUM;
    float32 avg_c = (float32)c_sum / (float32)SAMPLE_NUM;
    float32 adc_v = (avg_v / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float32 adc_c = (avg_c / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float32 bias_v = (adc_current_bias / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;

    *voltage = adc_v * VOLTAGE_SCALE;
    *current = (adc_c - bias_v) * CURRENT_SCALE;
    if (*current < 0.0f) *current = 0.0f;

    adc_voltage = *voltage;
    adc_current = *current;

    monitor_adc_raw_voltage = avg_v;
    monitor_adc_raw_current = avg_c;
    monitor_adc_voltage     = *voltage;
    monitor_adc_current     = *current;
    monitor_adc_st          = AdcRegs.ADCST.all;
    monitor_adc_result0     = (AdcRegs.ADCRESULT0 >> 4) & 0x0FFF;
    monitor_adc_result1     = (AdcRegs.ADCRESULT1 >> 4) & 0x0FFF;
    monitor_adc_sample_count++;
}

/* ---- bias 测量 (中断未开, 软件轮询) ---- */
void ADC_MeasureCurrentBias(void)
{
    Uint16 i, j;
    float32 readings[8];
    float32 sum = 0.0f;

    for (i = 0; i < 8; i++)
    {
        /* 等待转换完成 */
        Uint32 timeout = 0;
        while (AdcRegs.ADCST.bit.INT_SEQ1 == 0 && timeout < 100000)
            timeout++;
        if (timeout >= 100000)
            monitor_adc_timeout_count++;
        AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;

        readings[i] = (float32)((AdcRegs.ADCRESULT1 >> 4) & 0x0FFF);
        for (j = 0; j < 15000; j++) { asm(" NOP"); }
    }

    {
        Uint16 k;
        float32 tmp;
        for (i = 0; i < 7; i++)
            for (k = i + 1; k < 8; k++)
                if (readings[i] > readings[k])
                    { tmp = readings[i]; readings[i] = readings[k]; readings[k] = tmp; }
        sum = 0.0f;
        for (i = 1; i < 7; i++) sum += readings[i];
        adc_current_bias = sum / 6.0f;
    }

    monitor_adc_current_bias = adc_current_bias;
}

float32 ADC_GetVoltage(void) { return adc_voltage; }
float32 ADC_GetCurrent(void) { return adc_current; }
