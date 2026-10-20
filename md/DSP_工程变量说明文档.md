# DSP 工程变量与参数说明文档

> 本文档是工程全部**监测变量**和**配置参数**的字典，按模块分组，供 CCS 仿真与工程交接查阅。
> 所有监测变量均用 `volatile` 声明，可在 CCS Expressions 窗口实时查看。
> 配套文档：[项目说明文档.md](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/md/项目说明文档.md)、[监控变量调试指南.md](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/md/监控变量调试指南.md)

---

## 0. 在 CCS 中查看变量的方法

1. 暂停 DSP → Expressions 窗口 → Add New Expression → 输入变量名
2. 启用实时模式：点工具栏时钟/仪表盘图标变蓝
3. Expressions 窗口右上角下拉 → 勾选 "Continuous Refresh"
4. Resume 运行 DSP

数组变量（如 `monitor_xu_raw`）在 Expressions 中展开即可查看各元素。

---

## 1. 变量速查表（Quick Reference）

| 变量 | 含义 | 正常值（10Ω / 1.5A） |
|------|------|----------------------|
| `monitor_qp_exit` | MPCC QP 退出码 | **0** |
| `monitor_xu_raw[0]` | QP 解 u₀ | ≈ 前馈占空比 ≈ 0.6 |
| `monitor_xu_raw[1]` | QP 解 u₁ | 与 u₀ 有微差异 |
| `monitor_xu_raw[2]` | QP 解 u₂ | 与 u₀/u₁ 有微差异 |
| `monitor_mpcc_duty_cycle` | 输出占空比 | ≈ 0.6 |
| `monitor_mpcc_error` | 电流跟踪误差 | ≈ 0 |
| `monitor_state_voltage` | 实测输出电压 | ≈ 15V |
| `monitor_state_current` | 实测电感电流 | ≈ 1.5A |
| `monitor_sstc_ys_i` | SSTC 稳态电流目标 | ≈ 1.5 |
| `monitor_sstc_us` | SSTC 稳态占空比 | ≈ 0.6 |
| `monitor_adc_timeout_count` | ADC 超时次数 | **0** |

> 上表「正常值」基于 `BUCK_UF=25`、`current_target=1.5A`、负载 10Ω。不同负载下的期望值见第 8 节。

---

## 2. MPCC 动态控制层（[mpcc.c](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/mpcc/mpcc.c)）

### 控制目标与输出

| 变量 | 含义 | 说明 |
|------|------|------|
| `monitor_mpcc_target` | 传给 MPCC 的电流跟踪目标 = `ys_ref[1]` | 应 ≈ `current_target` |
| `monitor_mpcc_error` | 电流误差 = `ys_ref[1] − current` | 稳态趋近 0 |
| `monitor_mpcc_duty_cycle` | 最终输出占空比（已 clamp 0~1） | 稳态 ≈ `us_ref` |
| `monitor_mpcc_us_ref` | 前馈占空比 `us_ref` | = `monitor_sstc_us` |
| `monitor_mpcc_delta_duty` | 反馈修正量 = duty − us_ref | 稳态 ≈ 0 |
| `monitor_mpcc_qp_status` | MPCC QP 求解状态 | 0 = 成功 |

### 状态变量

| 变量 | 含义 |
|------|------|
| `monitor_state_voltage` | 状态 x[0]，实测输出电压 (V) |
| `monitor_state_current` | 状态 x[1]，实测电感电流 (A) |

### QP 内部数据（调试用）

| 变量 | 大小 | 含义 |
|------|------|------|
| `monitor_qp_exit` | — | qpOASES 退出码：**0=成功**，非 0=失败（失败时已回退到 us_ref） |
| `monitor_qp_iter` | — | QP 工作集重算次数（nWSR） |
| `monitor_xu_raw[3]` | 3 | QP 解的 3 个控制量 u₀/u₁/u₂，应平滑且有微差异 |
| `monitor_be[10]` | 10 | 自由响应 g_be（5 预测步 × 2 输出） |
| `monitor_mp_row0[3]` | 3 | 脉冲响应矩阵 g_MP 第一行 |
| `monitor_ff_u[3]` | 3 | QP 诊断：`[0]=u_opt[0]`、`[1]=目标函数值`、`[2]=迭代次数` |

---

## 3. SSTC 稳态目标层（[sstc.cpp](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/sstc/sstc.cpp)）

| 变量 | 含义 | 说明 |
|------|------|------|
| `monitor_sstc_Kv` | 电压通道 DC 增益 K[0] | = `BUCK_UF`（恒定） |
| `monitor_sstc_Ki` | 电流通道 DC 增益 K[1] | = `BUCK_UF / RL`，随负载变化 |
| `monitor_sstc_ys_v` | 稳态电压目标 ys[0] | = 电压目标 |
| `monitor_sstc_ys_i` | 稳态电流目标 ys[1] | 应 ≈ `current_target` |
| `monitor_sstc_us` | 稳态占空比 us | = `voltage_tgt / BUCK_UF` |
| `monitor_sstc_obj` | SSTC QP 目标函数值 | 负数，绝对值小 = 偏差小 |

### 松弛变量 η（软约束监控，仅 `SSTC_ENABLE=1` 时有意义）

| 变量 | 含义 |
|------|------|
| `monitor_sstc_eta_v` | 电压软约束松弛量 η_v |
| `monitor_sstc_eta_i` | 电流软约束松弛量 η_i |
| `monitor_eta_v_active_cnt` | η_v 激活累计次数 |
| `monitor_eta_i_active_cnt` | η_i 激活累计次数 |
| `monitor_eta_v_hist[20]` | η_v 每次激活峰值历史 |
| `monitor_eta_i_hist[20]` | η_i 每次激活峰值历史 |
| `monitor_eta_v_hist_idx` | η_v 历史写入位置 |
| `monitor_eta_i_hist_idx` | η_i 历史写入位置 |

---

## 4. ADC 采样层（[adc.c](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/adc/adc.c)）

| 变量 | 含义 | 正常值/判断 |
|------|------|------------|
| `monitor_adc_raw_voltage` | 电压通道原始 ADC 均值（0~4095） | 满量程对应 3.0V |
| `monitor_adc_raw_current` | 电流通道原始 ADC 均值 | 同上 |
| `monitor_adc_voltage` | 标定后输出电压 (V) | = `monitor_state_voltage` |
| `monitor_adc_current` | 标定后输出电流 (A) | = `monitor_state_current` |
| `monitor_adc_current_bias` | 电流零点偏置（上电测量） | 稳定值 |
| `monitor_adc_sample_count` | 采样批次数 | 持续递增 |
| `monitor_adc_timeout_count` | ADC 转换超时次数 | **应为 0** |
| `monitor_adc_st` | ADC 状态寄存器 `ADCST` | 诊断用 |
| `monitor_adc_result0` | 电压通道最新单次转换值 | 诊断用 |
| `monitor_adc_result1` | 电流通道最新单次转换值 | 诊断用 |

---

## 5. PWM 层（[pwm.c](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/pwm/pwm.c)）

| 变量 | 含义 | 范围 |
|------|------|------|
| `monitor_pwm_duty_cycle` | 当前 PWM 占空比 | 0.0~0.99 |
| `monitor_pwm_update_count` | PWM 更新次数 | 持续增长 |
| `monitor_pwm_cmpa_value` | CMPA 寄存器值 = PERIOD × duty | 0~1485 |

---

## 6. 系统与全局层（[main.c](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/User/main.c)）

### 全局反馈量

| 变量 | 含义 |
|------|------|
| `monitor_global_Ub` | 实测输出电压 |
| `monitor_global_iB` | 实测电感电流 |
| `monitor_global_iB_filtered` | 滤波后电流（当前未滤波，= current） |
| `monitor_global_us` | 最终输出占空比 = duty |

### 分模块耗时测量（CpuTimer2，150MHz）

| 变量 | 含义 | 定义位置 |
|------|------|---------|
| `monitor_sstc_time_us` | SSTC 单独耗时（微秒） | mpcc.c |
| `monitor_mpcc_qp_us` | MPCC 动态 QP 单独耗时（微秒） | mpcc.c |
| `monitor_control_cycle` | 整个控制周期耗时（微秒） | main.c |

### 数据导出历史数组

| 变量 | 含义 |
|------|------|
| `monitor_sstc_time_us_hist[100]` | SSTC 耗时历史（微秒），CCS Memory Browser 导出 |
| `monitor_mpcc_qp_us_hist[100]` | MPCC QP 耗时历史（微秒） |
| `monitor_control_cycle_hist[100]` | 控制周期历史（微秒） |
| `log_index` | 已记录点数 |
| `log_full` | =1 表示已存满 100 点 |

> PI 模式（`CTL_MPC=0`）下只记录 `monitor_control_cycle`，不记录 SSTC/QP 耗时。

### 系统状态

| 变量 | 含义 | 取值 |
|------|------|------|
| `monitor_system_uptime` | 主循环执行次数 | 持续增长 |
| `system_state` | 系统运行状态 | 0=READY, 1=RUNNING |
| `control_mode` | 控制模式 | 0=电流, 1=电压 |
| `current_target` | 电流目标 | 默认 1.5A |
| `voltage_target` | 电压目标 | = current_target × RL（动态） |

---

## 7. QP 求解器（[qpoases_port.cpp](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/qp/qpoases_port.cpp)）

| 参数/常量 | 值 | 说明 |
|-----------|-----|------|
| `NV` | 13 | 变量数 = 5×2 输出 + 3 控制量 |
| `NC` | 10 | 等式约束数 = 5×2 |
| `DU_WEIGHT_H` | 1.0 | Δu 占空比变化率惩罚权重 |
| nWSR | 80 | 工作集重算上限 |
| 精度 | float | 单精度（`__USE_SINGLE_PRECISION__`） |

调用流程：

```
mpcc.c → MPC_SolveFullQP()
  → QPOASES_Solve()          [qpoases_port.h]
    → QProblem::init()       首次
    → QProblem::hotstart()   后续（热启动）
    → 失败回退：u_opt = us_ref
```

---

## 8. 关键配置参数汇总

### Buck 电路参数（[mpcc.h](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/mpcc/mpcc.h)）

| 参数 | 值 | 说明 |
|------|-----|------|
| `BUCK_RL` | 10.0 | 默认负载电阻 (Ω) |
| `BUCK_C` | 10e-6 | 输出电容 (F) |
| `BUCK_L` | 3e-3 | 电感 (H) |
| `BUCK_UF` | 25.0 | 输入侧母线电压 (V) |

### MPCC 算法参数（[mpcc.h](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/mpcc/mpcc.h)）

| 参数 | 值 | 说明 |
|------|-----|------|
| `MPCC_PREDICTION_HORIZON` | 5 | 预测时域 Np |
| `MPCC_CONTROL_HORIZON` | 3 | 控制时域 Nc |
| `MPCC_SAMPLING_TIME` | 0.00001 | 10µs |
| `MPCC_NY` | 2 | 输出维度 |

### 权重与约束（[mpcc.c](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/mpcc/mpcc.c)）

| 参数 | 初始值 | 运行时值 | 含义 |
|------|--------|---------|------|
| `Q_diag[0]` | 0.0 | 0.0 | 电压权重 |
| `Q_diag[1]` | 1.0 | 1.0 | 电流权重 |
| `R_scalar` | 20.0 | 10.0 | 占空比权重 |
| `y_max[0]` | 40.0 | 40.0 | 电压上限 |
| `y_max[1]` | 3.5 | 3.5 | 电流上限 |
| `u_min/u_max` | 0/1 | 0/1 | 占空比限幅 |

### ADC 标定参数（[adc.c](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/adc/adc.c)）

| 参数 | 值 | 说明 |
|------|-----|------|
| `VOLTAGE_SCALE` | 10.091 | 电压分压比 |
| `CURRENT_SCALE` | 3.9916 | 电流采样放大系数 |
| `SAMPLE_NUM` | 40 | 每通道采样点数 |
| `ADC_REF_VOLTAGE` | 3.0 | ADC 参考电压 (V) |
| `ADC_MAX_VALUE` | 4095.0 | ADC 最大值（12 位） |

### PWM 参数（[pwm.c](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/pwm/pwm.c)）

| 参数 | 值 | 说明 |
|------|-----|------|
| `PWM_FREQ` | 100000 | PWM 频率 100kHz |
| `PWM_PERIOD` | 1500 | PWM 周期计数值 |
| `PWM_DEAD_TIME` | 30 | 死区 ~200ns（30 × 6.67ns） |

### PID 参数（[pid.h](file:///d:/wenjian/OneDrive/workspace_v6_1_3/DSP8233x_BUCKSSTC/APP/pid/pid.h)）

| 参数 | 值 |
|------|-----|
| `PID_KP_DEFAULT` | 0.02 |
| `PID_KI_DEFAULT` | 0.008 |
| `PID_KD_DEFAULT` | 0.0 |
| `PID_OUT_MIN/MAX` | 0.0 / 0.99 |

---

## 9. 不同负载下的期望值速查（`BUCK_UF=25`、`current_target=1.5A`）

| 变量 | 5Ω | 10Ω | 15Ω |
|------|-----|------|------|
| `voltage_target` | 7.5V | 15V | 22.5V |
| `monitor_sstc_Kv` | 25 | 25 | 25 |
| `monitor_sstc_Ki` | 5.0 | 2.5 | 1.67 |
| `monitor_sstc_us` | 0.3 | 0.6 | 0.9 |
| `monitor_sstc_ys_v` | 7.5V | 15V | 22.5V |
| `monitor_sstc_ys_i` | 1.5A | 1.5A | 1.5A |
| `monitor_mpcc_duty_cycle` | ≈ 0.3 | ≈ 0.6 | ≈ 0.9 |

> `monitor_sstc_ys_i` 始终 ≈ `current_target`（1.5A），这是「只控电流、电压仅约束」设计（`Q_diag[0]=0`）的直接体现。

---

## 10. 推荐的监测变量组

### QP 验证组（最高优先级）

```
monitor_qp_exit          // 0 = qpOASES 成功
monitor_xu_raw[0]        // u₀
monitor_xu_raw[1]        // u₁（应 ≠ u₀）
monitor_xu_raw[2]        // u₂（应 ≠ u₀,u₁）
```

### 控制性能组

```
monitor_mpcc_target      // 电流目标
monitor_mpcc_error       // 电流误差
monitor_mpcc_duty_cycle  // 输出占空比
monitor_mpcc_us_ref      // 前馈占空比
monitor_mpcc_delta_duty  // 反馈修正量
```

### SSTC 稳态组

```
monitor_sstc_Ki          // 电流增益（随负载变化）
monitor_sstc_us          // 稳态占空比
monitor_sstc_ys_i        // 稳态电流目标
monitor_sstc_ys_v        // 稳态电压目标
```

### 系统状态组

```
monitor_adc_voltage      // ADC 电压
monitor_adc_current      // ADC 电流
monitor_control_cycle    // 整个控制周期耗时（µs）
monitor_sstc_time_us    // SSTC 耗时（µs）
monitor_mpcc_qp_us       // MPCC QP 耗时（µs）
```

### 硬件调试组

```
monitor_adc_raw_voltage  // 原始电压 ADC 值
monitor_adc_raw_current  // 原始电流 ADC 值
monitor_adc_st           // ADC 状态寄存器
monitor_adc_timeout_count // ADC 超时计数（应为 0）
```
