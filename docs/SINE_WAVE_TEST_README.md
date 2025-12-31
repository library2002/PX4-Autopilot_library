# PX4 正弦波扫频测试功能说明

## 1. 功能描述

为 PX4 控制分配模块增加独立的正弦波测试输出功能，用于飞行试验中的舵面动态响应测试和频率特性分析。测试信号通过专用 uORB 消息传递，不影响正常飞控输出。

**工作原理**：
- Control Allocator 模块根据参数生成正弦波信号（公式：output = offset + amplitude × sin(2πft)）
- 通过 `actuator_test_sine` uORB 话题发布测试信号
- PWM Output 驱动订阅该信号并转换为 PWM 输出（[−1, 1] → [1000, 2000]μs）
- 支持 8 个独立通道，可单独或同时测试

**两种工作模式**：
- **固定频率模式**：输出恒定频率的正弦波
- **扫频模式**：自动从起始频率逐步递增到结束频率，每个频率保持设定时间

## 2. 为什么使用正弦波测试？

### 2.1 核心原理

**系统辨识的本质**：输入已知正弦信号 → 测量系统输出 → 对比分析 → 建立系统模型

```
你的操作：发送正弦指令 input(t) = 0.3 × sin(2π × 1Hz × t)
                    ↓
系统处理：[舵机 + 机械传动 + 空气动力 + 传感器]
                    ↓
系统输出：测量信号 output(t) = 0.27 × sin(2π × 1Hz × t - 30°)
                    ↓
数据分析：对比两个正弦波的幅值和相位
                    ↓
得到模型：增益 = 0.9, 相位延迟 = -30° (在1Hz频率下)
```

**这就是经典的频域系统辨识方法**！

### 2.2 线性系统的特殊性质

**核心原理**：线性系统对正弦波输入会产生**相同频率**的正弦波输出，只是幅值和相位会改变。

**测试过程**：
```
输入：input = 0.3 × sin(2π × 1Hz × t)     ← 你发送的控制指令（已知幅值 0.3）
      ↓
系统：舵机 + 舵面 + 空气动力 + 传感器
      ↓
输出：测量信号（频率相同 1Hz，幅值 A 和相位 φ 待测）
```

**输出测量的三种方式**：

| 测量对象 | 数据源 | 物理意义 | 测试场景 |
| -------- | ------ | -------- | -------- |
| **PWM输出** | `actuator_outputs.output[]` | 实际发送给舵机的PWM信号 | 地面测试，验证信号链路 |
| **舵面角度** | 舵面角度传感器（如需外接） | 舵面实际偏转角度 | 地面机械测试，评估舵机响应 |
| **姿态响应** | IMU数据（角速度/角度） | 飞机姿态的实际变化 | 飞行测试，评估整体气动响应 |

**最常用的分析方法**：
- **地面测试**：对比 `actuator_test_sine` 输入指令 vs `actuator_outputs` PWM输出
- **飞行测试**：对比 `actuator_test_sine` 输入指令 vs IMU姿态数据（如滚转角速度）

**操作流程**：
1. **设定输入**：你发送正弦指令（幅值 0.3, 频率 1Hz）
2. **记录输出**：传感器测量系统实际响应（幅值 A, 频率 1Hz, 相位 φ）
3. **对比分析**：比较两个正弦波的差异
4. **提取模型**：得到系统在该频率的传递函数特性

**提取的关键参数**：
- **增益（Gain）**：`G = A / 0.3` - 系统响应能力（输出幅值 / 输入幅值）
- **相位延迟（Phase）**：`φ` - 输出滞后于输入的相位角（反映时间延迟）

**具体例子**：
```
步骤1 - 输入正弦信号：
  actuator_test_sine.output[0] = 0.3 × sin(2π × 1Hz × t)

步骤2 - 测量输出信号（三种方式）：
  方式A：actuator_outputs.output[0] = 1620μs 波动 (对应 0.24 归一化值)
  方式B：舵面角度传感器 = 12° × sin(2π × 1Hz × t - 30°)
  方式C：IMU角速度 = 15°/s × sin(2π × 1Hz × t - 45°)

步骤3 - 对比分析（以方式A为例）：
  ├─ 输入幅值 = 0.3
  ├─ 输出幅值 = 0.24 (从PWM换算: (1620-1500)/500 = 0.24)
  ├─ 相位差 ≈ 0° (PWM输出几乎无延迟)

步骤4 - 得到模型参数：
  ├─ 增益 = 0.24 / 0.3 = 0.8 (PWM输出达到指令的80%)
  ├─ 相位 ≈ 0° (电子信号几乎无延迟)
  └─ 结论：信号链路正常，但可能存在饱和或限幅
```

**飞行测试例子**（测量姿态响应）：
```
输入：副翼指令 = 0.2 × sin(2π × 0.5Hz × t)
输出：滚转角速度 = 18°/s × sin(2π × 0.5Hz × t - 60°)

分析：
  → 增益 = 18 / 0.2 = 90 (°/s per 归一化单位)
  → 相位 = -60° (滞后 0.33秒)
  → 结论：低频响应良好，但有明显的气动延迟
```

### 2.3 为什么不用其他信号？

| 信号类型 | 问题 | 正弦波的优势 |
| -------- | ---- | ------------ |
| 恒定值 | 无法测试动态响应，无法知道系统在运动时的特性 | ✅ 持续振荡，完整评估动态性能 |
| 阶跃信号 | 包含所有频率成分，难以分离特定频率的响应 | ✅ 单一频率，精确测量该频率的特性 |
| 随机信号 | 需要复杂的频谱分析，信噪比低 | ✅ 能量集中在测试频率，信噪比高 |

**正弦波的独特优势**：
- 输入输出都是正弦波，易于对比分析
- 每个频率单独测试，建立精确的频率响应模型
- 测试结果直接对应传递函数的频域特性

### 2.4 扫频的意义

不同频率下，系统响应特性不同：

| 频率 | 输入指令幅值 | 测量输出幅值 | 增益 | 相位延迟 | 物理意义 |
| ---- | ------------ | ------------ | ---- | -------- | -------- |
| **低频（0.5Hz）** | 0.3 | 0.30 | 1.0 | 10° | 舵面完全跟随指令，响应充分 |
| **中频（2Hz）** | 0.3 | 0.24 | 0.8 | 45° | 开始出现滞后，幅度衰减20% |
| **高频（5Hz）** | 0.3 | 0.09 | 0.3 | 90° | 舵面跟不上，大幅衰减70% |

**扫频测试**自动遍历所有频率，获得完整的**波特图（Bode Plot）**：
- **幅频曲线**：增益 vs 频率 → 找到系统带宽（-3dB 频率）
- **相频曲线**：相位 vs 频率 → 评估稳定裕度

**建立系统模型**：
- 通过多个频率点的增益和相位数据
- 可以拟合出系统的**传递函数模型** G(s)
- 用于控制器设计和性能预测

**实际应用**：
- 识别控制系统的频率特性
- 优化 PID 参数（避免在相位裕度不足的频率激进调参）
- 检测机械共振频率
- 验证执行器带宽是否满足设计要求

### 2.5 数据记录与分析

测试时，系统会同步记录：
- `actuator_test_sine.output[]` - 输入指令（已知的正弦波）
- `actuator_outputs.output[]` - 实际 PWM 输出
- 姿态传感器数据 - 实际舵面响应效果

**离线分析示例**：
```python
# 提取 1Hz 测试段的数据
input_signal = log['actuator_test_sine.output[0]']     # 输入指令
pwm_output = log['actuator_outputs.output[0]']         # PWM输出
roll_rate = log['vehicle_angular_velocity.xyz[0]']    # 滚转角速度（输出响应）

# 计算增益和相位
gain = amplitude(roll_rate) / amplitude(input_signal)
phase_delay = cross_correlation(input_signal, roll_rate)

# 绘制波特图
plot_bode(frequencies, gains, phases)
```

## 3. 主要代码文件

| 文件 | 功能 |
| ---- | ---- |
| `msg/ActuatorTestSine.msg` | 定义 uORB 消息格式（通道、频率、幅值、偏置、8 通道输出数组） |
| `msg/CMakeLists.txt` | 注册消息到编译系统 |
| `src/modules/control_allocator/ControlAllocator.hpp` | 添加发布者、参数声明、状态变量 |
| `src/modules/control_allocator/ControlAllocator.cpp` | 实现 `publish_sine_test_output()` 函数，周期性生成并发布正弦波 |
| `src/modules/control_allocator/module.yaml` | 定义 5 个参数（CA_SINE_TST_*）的类型、范围、默认值 |
| `src/drivers/pwm_out/PWMOut.hpp` | 添加 `actuator_test_sine` 订阅者声明 |
| `src/drivers/pwm_out/PWMOut.cpp` | 订阅测试信号，转换为 PWM 并输出到硬件 |

## 4. 参数配置

### 4.1 基础参数

| 参数 | 类型 | 范围 | 默认值 | 说明 |
| ---- | ---- | ---- | ------ | ---- |
| `CA_SINE_TST_EN` | INT32 | 0/1 | 0 | 测试使能开关，1=启用，0=禁用 |
| `CA_SINE_TST_CH` | INT32 | 0-7 | 0 | 输出通道号 |
| `CA_SINE_TST_MODE` | INT32 | 0/1 | 0 | 测试模式：0=固定频率，1=扫频 |
| `CA_SINE_TST_AMP` | FLOAT | 0.0-1.0 | 0.2 | 振幅（0.0-1.0） |
| `CA_SINE_TST_OFS` | FLOAT | -1.0-1.0 | 0.0 | 直流偏置（0.0 为中立位置） |

### 3.2 固定频率模式参数

| 参数 | 类型 | 范围 | 默认值 | 说明 |
| ---- | ---- | ---- | ------ | ---- |
| `CA_SINE_TST_FREQ` | FLOAT | 0.1-20.0 | 0.5 | 正弦波频率 (Hz) |

### 3.3 扫频模式参数

| 参数 | 类型 | 范围 | 默认值 | 说明 |
| ---- | ---- | ---- | ------ | ---- |
| `CA_SINE_TST_F_MIN` | FLOAT | 0.1-20.0 | 0.5 | 起始频率 (Hz) |
| `CA_SINE_TST_F_MAX` | FLOAT | 0.1-20.0 | 5.0 | 结束频率 (Hz) |
| `CA_SINE_TST_STEP` | FLOAT | 0.1-5.0 | 0.5 | 频率步进 (Hz) |
| `CA_SINE_TST_TIME` | FLOAT | 1.0-120.0 | 10.0 | 每个频率持续时间 (秒) |

### 3.4 配置示例

**固定频率模式**：
```bash
param set CA_SINE_TST_EN 1          # 启用测试
param set CA_SINE_TST_CH 0          # 通道 0
param set CA_SINE_TST_MODE 0        # 固定频率模式
param set CA_SINE_TST_FREQ 1.0      # 1Hz
param set CA_SINE_TST_AMP 0.3       # 30% 幅度
param set CA_SINE_TST_OFS 0.0       # 中立位置
param save
```

**扫频模式**：
```bash
param set CA_SINE_TST_EN 1          # 启用测试
param set CA_SINE_TST_CH 0          # 通道 0
param set CA_SINE_TST_MODE 1        # 扫频模式
param set CA_SINE_TST_F_MIN 0.5     # 从 0.5Hz 开始
param set CA_SINE_TST_F_MAX 5.0     # 到 5.0Hz 结束
param set CA_SINE_TST_STEP 0.5      # 每次增加 0.5Hz
param set CA_SINE_TST_TIME 10.0     # 每个频率持续 10 秒
param set CA_SINE_TST_AMP 0.3       # 30% 幅度
param save
```

停止测试：`param set CA_SINE_TST_EN 0`

## 5. 编译与使用


**编译固件**：
```bash
make px4_sitl_default          # 仿真版本
make px4_fmu-v5_default        # 硬件版本（根据飞控板型号调整）
```

**验证方法**：
```bash
# 检查参数
param show CA_SINE_TST_*

# 监听测试话题
listener actuator_test_sine

# 预期输出（实时变化）
# timestamp: 123456789
# enabled: 1
# channel: 0
# frequency: 1.00
# amplitude: 0.30
# offset: 0.00
# output[0]: 0.234  (正弦波值)
# output[1]: nan
```

**PWM 输出验证**：通过舵机测试器或示波器观察指定通道，应为 1000-2000μs 范围的正弦波形（映射公式：PWM = 1500 + output × 500）

## 6. 测试场景

**固定频率低频特性测试**（测量舵面低频响应精度）：
```bash
param set CA_SINE_TST_MODE 0        # 固定频率模式
param set CA_SINE_TST_FREQ 0.2      # 5秒/周期
param set CA_SINE_TST_AMP 0.3
```

**自动扫频测试**（获取完整频率响应曲线）：
```bash
param set CA_SINE_TST_MODE 1        # 扫频模式
param set CA_SINE_TST_F_MIN 0.5     # 起始 0.5Hz
param set CA_SINE_TST_F_MAX 5.0     # 结束 5.0Hz
param set CA_SINE_TST_STEP 0.5      # 步进 0.5Hz
param set CA_SINE_TST_TIME 15.0     # 每个频率 15 秒
param set CA_SINE_TST_AMP 0.2       # 20% 幅度
```
扫频序列：0.5Hz (15s) → 1.0Hz (15s) → 1.5Hz (15s) → 2.0Hz (15s) → 2.5Hz (15s) → 3.0Hz (15s) → 3.5Hz (15s) → 4.0Hz (15s) → 4.5Hz (15s) → 5.0Hz (15s) → 循环

**快速扫频测试**（粗略评估）：
```bash
param set CA_SINE_TST_F_MIN 0.5
param set CA_SINE_TST_F_MAX 10.0
param set CA_SINE_TST_STEP 1.0      # 大步进
param set CA_SINE_TST_TIME 5.0      # 短时间
```

**偏置位置测试**（测试不同修整位置的响应）：
```bash
param set CA_SINE_TST_MODE 0
param set CA_SINE_TST_FREQ 0.5
param set CA_SINE_TST_OFS 0.2       # +20% 偏置
param set CA_SINE_TST_AMP 0.1       # 小幅振荡
```

## 7. 数据分析


**ULog 记录**：测试数据自动记录在 `.ulg` 日志文件中，包含字段：
- `actuator_test_sine.output[0-7]` - 各通道输出值
- `actuator_test_sine.frequency/amplitude/offset` - 参数设置

**分析工具**：
- **FlightPlot** / **PlotJuggler**：查看时域波形（`plotjuggler logfile.ulg`）
- **pyulog + Python**：FFT 频谱分析、波特图绘制

**数据质量检查**：频率准确性（FFT 峰值位置）、谐波失真（THD < 5%）、幅值一致性（标准差 < 2%）

## 8. 故障排除

| 问题 | 可能原因 | 解决方法 |
| ---- | -------- | -------- |
| 参数不存在 | 未完整编译固件 | `make clean && make px4_sitl_default` |
| 话题无数据 | 测试未启用 | 检查 `param show CA_SINE_TST_EN` |
| PWM 无输出 | 通道未初始化或被占用 | 检查 `dmesg` 日志，确认通道配置 |
| 波形失真 | 频率过高或采样率不足 | 降低频率到 1Hz 以下 |

## 9. 技术细节

**信号生成公式**：
```
output(t) = offset + amplitude × sin(2π × frequency × t)
```
其中 t 为从启动测试开始的经过时间（秒）

**扫频逻辑**：
- 初始频率 = `CA_SINE_TST_F_MIN`
- 每隔 `CA_SINE_TST_TIME` 秒，频率增加 `CA_SINE_TST_STEP`
- 当频率超过 `CA_SINE_TST_F_MAX` 时，重置为 `CA_SINE_TST_F_MIN` 并循环
- 每次频率变化时，正弦波相位重置（从 0 开始）

**扫频时间计算**：
```
总时间 = (F_MAX - F_MIN) / STEP × TIME
例如：(5.0 - 0.5) / 0.5 × 10 = 90 秒
```

**PWM 映射**：`PWM(μs) = 1500 + output × 500`，将归一化值 [-1, 1] 转换为 [1000, 2000]μs 脉宽

**数据流**：Control Allocator（~250Hz）→ uORB `actuator_test_sine` → PWM Output Driver → 硬件输出

**建议测试参数**：

| 测试类型 | 模式 | 频率范围 (Hz) | 步进/时间 | 幅值 | 备注 |
| -------- | ---- | ------------- | --------- | ---- | ---- |
| 地面测试 | 固定 | 0.5-2.0 | - | 0.3-0.5 | 观察舵面动作 |
| 首次飞行 | 固定 | 0.2-0.5 | - | 0.1-0.2 | 极度保守 |
| 精细扫频 | 扫频 | 0.5-5.0 | 0.5Hz / 15s | 0.2-0.3 | 标准频率响应分析 |
| 快速扫频 | 扫频 | 0.5-10.0 | 1.0Hz / 5s | 0.1-0.2 | 快速评估 |
| 高频测试 | 固定 | 5.0-20.0 | - | 0.1-0.2 | 需谨慎，可能激发振动 |

**QGroundControl 操作**：
1. **Vehicle Setup** → **Parameters** → 搜索 "SINE" 或 "CA_SINE"
2. 在 **Control Allocation** 分组下修改参数，自动生效无需重启
3. 快速命令（MAVLink Console）：
   ```bash
   # 固定频率快速启动
   param set CA_SINE_TST_EN 1; param set CA_SINE_TST_MODE 0; param set CA_SINE_TST_FREQ 1.0

   # 扫频快速启动
   param set CA_SINE_TST_EN 1; param set CA_SINE_TST_MODE 1

   # 停止测试
   param set CA_SINE_TST_EN 0
   ```
---

**完整实现包含**：
- 正弦波生成：`ControlAllocator::publish_sine_test_output()`
- 扫频逻辑：自动频率递增和相位重置
- PWM 转换输出：`PWMOut::Run()` 中订阅并处理测试信号
- 参数系统集成：通过 `module.yaml` 定义的 10 个参数控制（5 个基础参数 + 5 个扫频参数）
