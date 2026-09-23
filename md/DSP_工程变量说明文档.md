# DSP 工程变量和作用说明文档

## 文档概述

本文档详细说明 DSP8233x_BUCK 工程中的所有监测变量及参数，方便在 CCS 仿真中实时监测系统状态。所有监测变量均使用 `volatile` 关键字声明。

**工程用 qpOASES 求解 8 变量完整 QP（5 个输出预测 + 3 个控制量），与论文/MATLAB 数学等价。** 详细证明见 `MPC_QP求解器验证证明.md`。

---

## 在 CCS 中查看监测变量的方法

1. 暂停 DSP → Expressions 窗口 → Add New Expression → 输入变量名
2. 启用实时模式：点工具栏时钟/仪表盘图标变蓝
3. Expressions 窗口右上角下拉 → 勾选 "Continuous Refresh"
4. Resume 运行 DSP

---

## 变量速查表（Quick Reference）

| 变量 | 含义 | 正常值 | QP 验证关键 |
|------|------|--------|:---:|
| `monitor_qp_exit` | QP 退出码 | **0** | ★★★ |
| `monitor_xu_raw[0]` | QP 解 u₀ | 与 duty_cycle 同 | ★★ |
| `monitor_xu_raw[1]` | QP 解 u₁ | **≠ u₀** | ★★★ |
| `monitor_xu_raw[2]` | QP 解 u₂ | **≠ u₀, u₁** | ★★★ |
| `monitor_mpcc_duty_cycle` | 输出占空比 | 0.6~0.9 | ★★ |
| `monitor_mpcc_error` | 电流误差 | ≈0 | ★★ |
| `monitor_state_voltage` | 状态 x[0]→电压 | 10~16V | — |
| `monitor_state_current` | 状态 x[1]→电流 | 0.6~0.8A | — |
| `monitor_be[0]` | 自由响应 | =voltage/20 | ★ |
| `monitor_ff_u[0]` | QP 解的 u₀ | =duty | ★ |
| `monitor_ff_u[1]` | QP 目标函数值 | 负数 | — |
| `monitor_ff_u[2]` | QP 工作集重算 | 0~15 | — |

---

## 1. MPCC 控制性能监测 (`mpcc.c`)

### 控制目标监测

| 变量 | 含义 | 单位 | 更新时机 |
|------|------|------|---------|
| `monitor_mpcc_target` | MPC 内部模型跟踪目标 `ys_ref` | A（电流模式） | 每次 MPCC_Controller |
| `monitor_mpcc_error` | 目标值与实际值的误差 = `ys_ref − voltage/BUCK_RL` | A | 每次 |
| `monitor_mpcc_duty_cycle` | MPC 控制器计算的最优占空比（已 clamp 到 0~1） | — | 每次 |

### QP 求解状态监测（最重要）

| 变量 | 含义 | 取值说明 |
|------|------|---------|
| `monitor_mpcc_qp_status` | QP 求解状态 | 0=成功，非0=失败 |
| `monitor_qp_exit` | qpOASES 退出标志 | **0=SUCCESS**，1=不可行，2=超迭代，-1=回退到闭式解 |
| `monitor_qp_iter` | QP 工作集重算次数 | 正常 2~15，0 表示闭式解或旁路模式 |

**判断 qpOASES 是否在跑的关键逻辑**：
- `monitor_qp_exit == 0` **且** `monitor_xu_raw[0,1,2]` 三点不全相等 → qpOASES 正常
- `monitor_qp_exit == -1` → QP 失败，自动回退到闭式解
- `monitor_qp_iter == 0` 且 `monitor_xu_raw` 全是垃圾 → qpOASES 未初始化

### 状态变量监测

| 变量 | 含义 | 来源 |
|------|------|------|
| `monitor_state_voltage` | 状态 x[0]，即 Buck 输出电压经低通后进 MPC | ADC 采样 + 低通滤波 |
| `monitor_state_current` | 状态 x[1]，即 Buck 输出电流经低通后进 MPC | ADC 采样 + 低通滤波 |

### QP 内部数据监测（调试用）

| 变量 | 大小 | 含义 | 如何验证正确性 |
|------|------|------|--------------|
| `monitor_be[5]` | 5 | 自由响应 g_be（预测步 0~4） | `be[0] = voltage/20` 手算对照 |
| `monitor_mp_row0[3]` | 3 | g_MP 第一行（控制量对第 1 步预测的影响系数） | 应约 -0.003（Uf=20, T=10µs） |
| `monitor_ff_u[3]` | 3 | QP 诊断：`[0]=x_opt[5]`（第一个控制量）, `[1]=目标函数值`, `[2]=工作集重算次数` | `ff_u[0] ≈ duty_cycle` |
| `monitor_xu_raw[3]` | 3 | **QP 解的 3 个控制量** u₀, u₁, u₂ | **三点应有微差异（QP 独立优化三个变量）** |

---

## 2. ADC 采样监测 (`adc.c`)

### 原始采样值

| 变量 | 含义 | 范围 |
|------|------|------|
| `monitor_adc_raw_voltage` | 原始电压 ADC 值（200 次平均） | 0~4095 |
| `monitor_adc_raw_current` | 原始电流 ADC 值（200 次平均） | 0~4095 |

### 转换后物理值

| 变量 | 含义 | 标定公式 |
|------|------|---------|
| `monitor_adc_voltage` | 标定后的实际电压值 | `(avg/4095) × 3.0 × 10.091` |
| `monitor_adc_current` | 标定后的实际电流值 | `((avg/4095)×3.0 − bias_v) × 4.0816` |
| `monitor_adc_current_bias` | 电流零偏值（ADC 原始值） | 启动阶段测量 |

### 采样统计

| 变量 | 含义 | 异常标志 |
|------|------|---------|
| `monitor_adc_sample_count` | 累计采样次数 | 持续增长 = 主循环正常 |
| `monitor_adc_timeout_count` | ADC 超时次数 | >0 = ADC 采集异常，需检查硬件 |

### 硬件状态

| 变量 | 含义 |
|------|------|
| `monitor_adc_st` | ADCST 寄存器（Uint16） |
| `monitor_adc_result0` | ADCRESULT0 最后一次原始值（Uint16，12 位有效） |
| `monitor_adc_result1` | ADCRESULT1 最后一次原始值（Uint16，12 位有效） |

---

## 3. PWM 控制监测 (`pwm.c`)

| 变量 | 含义 | 范围 |
|------|------|------|
| `monitor_pwm_duty_cycle` | 当前 PWM 占空比 | 0.0~0.99 |
| `monitor_pwm_update_count` | PWM 更新次数 | 持续增长 |
| `monitor_pwm_cmpa_value` | CMPA 寄存器值 = PERIOD × duty | 0~1500 |

---

## 4. 系统状态监测 (`main.c`)

| 变量 | 含义 | 取值 |
|------|------|------|
| `monitor_system_uptime` | 主循环执行次数（Uint32） | 持续增长 |
| `monitor_system_state` | 系统运行状态 | 0=READY, 1=RUNNING |
| `monitor_control_mode` | 控制模式 | 0=电流, 1=电压 |
| `monitor_target_value` | 当前目标值 | 电流或电压 |
| `monitor_voltage_target` | 电压目标 | 默认 12.0V |
| `monitor_current_target` | 电流目标 | 默认 0.75A |
| `monitor_control_cycle_count` | 控制循环计数 | 持续增长 |
| `monitor_build_id` | 编译版本标识 | 12345（验证新代码生效用） |

### 全局变量监测

| 变量 | 含义 | 用途 |
|------|------|------|
| `monitor_global_Ub` | 反馈电压值 | 对照 state_voltage |
| `monitor_global_iB` | 反馈电流值（原始 ADC） | 对照 state_current |
| `monitor_global_iB_filtered` | 反馈电流值（低通滤波后，进 MPC） | α=0.3 一阶滤波 |
| `monitor_global_us` | 最终输出 duty | 对照 mpcc_duty_cycle |
| `monitor_global_ys` | 被控输出值 | 电流模式=current，电压模式=voltage |

---

## 5. QP 求解器 (`APP/qp/`)

### 求解器信息
- **名称**: qpOASES 3.2
- **算法**: 在线 active-set 策略 (Online Active Set Strategy)
- **证书**: GNU LGPL v2.1
- **接入**: `qpoases_port.cpp`，extern "C" 接口供 `mpcc.c` 调用

### 调用流程
```
mpcc.c  →  MPC_SolveFullQP()
  →  QPOASES_Solve()  [qpoases_port.h]
    →  qpoases_port.cpp  (C++)
      →  QProblem::init()     // 首次
      →  QProblem::hotstart()  // 后续（热启动加速）
```

### 关键参数

| 参数 | 值 | 说明 |
|------|------|------|
| NV | 8 | 5 个输出预测 y + 3 个控制量 u |
| NC | 5 | 5 个等式约束（预测模型） |
| 箱约束 | y ∈ [-0.01, 2], u ∈ [0, 1] | 电流和占空比硬约束 |
| 等式约束 | Y = g_be + g_MP × U | 预测模型等式 |
| Q 权重 | 1.0 | 输出跟踪 |
| R 权重 | 1.0 | 控制量惩罚 |
| 初始 WSR | 50 | 工作集重算上限 |
| 精度 | 单精度 float | `__USE_SINGLE_PRECISION__` |

---

## 6. 系统参数配置

### Buck 电路参数 (`mpcc.h`)
| 参数 | 值 | 说明 |
|------|------|------|
| `BUCK_RL` | 20.0 | 负载电阻 (Ω) |
| `BUCK_C` | 10e-6 | 输出电容 (F) |
| `BUCK_L` | 3e-3 | 电感 (H) |
| `BUCK_UF` | 20.0 | 等效输入电压 (V) |

### MPCC 算法参数 (`mpcc.h`)
| 参数 | 值 | 说明 |
|------|------|------|
| `MPCC_PREDICTION_HORIZON` | 5 | 预测时域 Np |
| `MPCC_CONTROL_HORIZON` | 3 | 控制时域 Nc |
| `MPCC_SAMPLING_TIME` | 0.00001 | 采样时间 10µs (100kHz) |

### ADC 标定参数 (`adc.c`)
| 参数 | 值 | 说明 |
|------|------|------|
| `VOLTAGE_SCALE` | 10.091 | 电压分压比 |
| `CURRENT_SCALE` | 4.0816 | 电流传感器转换系数 |
| `ADC_REF_VOLTAGE` | 3.0 | ADC 参考电压 (V) |
| `ADC_MAX_VALUE` | 4095.0 | ADC 最大值 (12 位) |

### PWM 参数 (`pwm.c`)
| 参数 | 值 | 说明 |
|------|------|------|
| `PWM_FREQ` | 100000 | PWM 频率 100kHz |
| `PWM_PERIOD` | 1500 | PWM 周期计数值 |
| `PWM_DEAD_TIME` | 30 | 死区时间 ~200ns (30 × 6.67ns) |

---

## 7. 推荐的监测变量组

### QP 验证组（最高优先级）
```
monitor_qp_exit          // 0 = qpOASES 成功
monitor_xu_raw[0]        // u₀
monitor_xu_raw[1]        // u₁（应 ≠ u₀）
monitor_xu_raw[2]        // u₂（应 ≠ u₀,u₁）
```
**用途**：确认 qpOASES 8 变量 QP 在运行

### 控制性能组
```
monitor_mpcc_target      // 目标值
monitor_mpcc_error       // 控制误差
monitor_mpcc_duty_cycle  // 输出占空比
monitor_mpcc_qp_status   // QP 求解状态
```

### 模型验证组
```
monitor_be[0]            // 自由响应 g_be[0]
monitor_mp_row0[0]       // g_MP[0][0]
monitor_ff_u[0]          // QP 解的 u₀
monitor_state_voltage    // 电压状态
```

### 系统状态组
```
monitor_adc_voltage      // ADC 电压
monitor_adc_current      // ADC 电流
monitor_pwm_duty_cycle   // PWM 占空比
monitor_system_state     // 系统状态
```

### 硬件调试组
```
monitor_adc_raw_voltage  // 原始电压 ADC 值
monitor_adc_raw_current  // 原始电流 ADC 值
monitor_adc_st           // ADC 状态寄存器
monitor_adc_timeout_count // ADC 超时计数
```

---

## 8. 工程文件结构

```
DSP8233x_BUCK/
├── User/
│   └── main.c                  // 主程序入口，主控制循环
├── APP/
│   ├── adc/
│   │   ├── adc.c               // ADC 采样驱动（连续模式，200 次平均）
│   │   └── adc.h
│   ├── mpcc/
│   │   ├── mpcc.c              // QP 调用入口（MPC_Controller → qpOASES）
│   │   └── mpcc.h
│   ├── pwm/
│   │   ├── pwm.c               // ePWM1 驱动（100kHz，互补 + 死区）
│   │   └── pwm.h
│   └── qp/
│       ├── qpoases_port.h      // qpOASES C 接口声明
│       ├── qpoases_port.cpp    // qpOASES C++ 调用实现
│       ├── *.cpp               // qpOASES 核心源文件（14 个）
│       └── include/
│           └── qpOASES/        // qpOASES C++ 头文件
├── DSP2833x_Libraries/         // TI C2000 外设库
├── F28335.cmd                  // 链接脚本（Flash + RAM 分配）
├── MPC_QP求解器验证证明.md      // QP 求解器正确性验证文档
├── DSP_工程变量说明文档.md      // 本文档
└── 硬件配置说明.md              // 硬件端口和参数说明
```
