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

## 2. 主要代码文件

| 文件 | 功能 |
| ---- | ---- |
| `msg/ActuatorTestSine.msg` | 定义 uORB 消息格式（通道、频率、幅值、偏置、8 通道输出数组） |
| `msg/CMakeLists.txt` | 注册消息到编译系统 |
| `src/modules/control_allocator/ControlAllocator.hpp` | 添加发布者、参数声明、状态变量 |
| `src/modules/control_allocator/ControlAllocator.cpp` | 实现 `publish_sine_test_output()` 函数，周期性生成并发布正弦波 |
| `src/modules/control_allocator/module.yaml` | 定义 5 个参数（CA_SINE_TST_*）的类型、范围、默认值 |
| `src/drivers/pwm_out/PWMOut.hpp` | 添加 `actuator_test_sine` 订阅者声明 |
| `src/drivers/pwm_out/PWMOut.cpp` | 订阅测试信号，转换为 PWM 并输出到硬件 |

## 3. 参数配置

### 3.1 基础参数

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

## 4. 编译与使用


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

## 5. 测试场景

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

## 6. 数据分析


**ULog 记录**：测试数据自动记录在 `.ulg` 日志文件中，包含字段：
- `actuator_test_sine.output[0-7]` - 各通道输出值
- `actuator_test_sine.frequency/amplitude/offset` - 参数设置

**分析工具**：
- **FlightPlot** / **PlotJuggler**：查看时域波形（`plotjuggler logfile.ulg`）
- **pyulog + Python**：FFT 频谱分析、波特图绘制

**数据质量检查**：频率准确性（FFT 峰值位置）、谐波失真（THD < 5%）、幅值一致性（标准差 < 2%）

## 7. 故障排除

| 问题 | 可能原因 | 解决方法 |
| ---- | -------- | -------- |
| 参数不存在 | 未完整编译固件 | `make clean && make px4_sitl_default` |
| 话题无数据 | 测试未启用 | 检查 `param show CA_SINE_TST_EN` |
| PWM 无输出 | 通道未初始化或被占用 | 检查 `dmesg` 日志，确认通道配置 |
| 波形失真 | 频率过高或采样率不足 | 降低频率到 1Hz 以下 |

## 8. 技术细节

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
