# UAVCAN 控制指令传输功能实现

## 功能概述

本实现在PX4 UAVCAN驱动中新增了**控制指令传输功能**，用于链翼组合飞行器系统。中央主飞控可以通过UAVCAN总线向左右从飞控发送控制指令（PWM值），实现协同控制。

## 系统架构

### 数据流向
```
中央飞控（主）:
  姿态控制器 → ControlAllocator → actuator_outputs → UAVCAN发送器 → CAN总线

左/右飞控（从）:
  CAN总线 → UAVCAN接收器 → uavcan_control_command → 应用到舵机
```

### 核心文件

| 角色 | 文件 | 说明 |
| ---- | ---- | ---- |
| 消息定义 | `msg/UavcanControlCommand.msg` | 定义控制指令uORB消息，包含时间戳、节点ID、PWM输出数组 |
| 发送器（主） | `src/drivers/uavcan/control_command_sender.{hpp,cpp}` | 订阅本机actuator_outputs，转换并通过UAVCAN广播PWM控制指令 |
| 接收器（从） | `src/drivers/uavcan/sensors/control_command.{hpp,cpp}` | 订阅UAVCAN控制指令，发布到本地uORB话题 |
| 参数配置 | `src/drivers/uavcan/uavcan_params.c` | 定义UAVCAN_PUB_CTRL和UAVCAN_SUB_CTRL参数 |

## 传输的数据

### UAVCAN消息类型
使用标准的 `uavcan::equipment::actuator::ArrayCommand` 消息，包含：
- **命令数组**：最多15个执行器命令
- 每个命令包含：
  - `actuator_id`: 执行器ID（0-15）
  - `command_type`: 命令类型（PWM = 4）
  - `command_value`: PWM值（1000-2000微秒）

### uORB消息内容
`UavcanControlCommand` 包含：
- `timestamp`: 接收时间戳（微秒）
- `timestamp_sample`: 数据采样时间戳（微秒）
- `source_node_id`: 源节点ID（发送方）
- `target_node_id`: 目标节点ID（接收方识别用）
- `num_outputs`: 有效输出数量（1-16）
- `outputs[16]`: PWM值数组（1000-2000微秒，0表示未使用）

## 参数配置

### 必需参数

| 参数 | 类型 | 默认值 | 说明 |
| ---- | ---- | ------ | ---- |
| `UAVCAN_ENABLE` | INT32 | 0 | UAVCAN使能，设为2或3 |
| `UAVCAN_NODE_ID` | INT32 | 1 | 本机节点ID（1-125），**必须唯一** |
| `UAVCAN_BITRATE` | INT32 | 1000000 | CAN总线波特率（bps） |
| `UAVCAN_PUB_CTRL` | INT32 | 0 | 控制指令发布开关（中央飞控设为1） |
| `UAVCAN_SUB_CTRL` | INT32 | 0 | 控制指令订阅开关（左右飞控设为1） |

### 配置示例

#### 中央飞控（节点ID=1，发送控制指令）
```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 1
param set UAVCAN_PUB_CTRL 1      # 发送控制指令
param set UAVCAN_SUB_ATT 1       # 接收左右飞控的姿态反馈
param save
reboot
```

#### 左侧飞控（节点ID=2，接收控制指令）
```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 2
param set UAVCAN_SUB_CTRL 1      # 接收控制指令
param set UAVCAN_PUB_ATT 1       # 发送本机姿态给中央飞控
param save
reboot
```

#### 右侧飞控（节点ID=3，接收控制指令）
```bash
param set UAVCAN_ENABLE 3
param set UAVCAN_NODE_ID 3
param set UAVCAN_SUB_CTRL 1      # 接收控制指令
param set UAVCAN_PUB_ATT 1       # 发送本机姿态给中央飞控
param save
reboot
```

## 测试验证

### 1. 检查UAVCAN总线状态
```bash
uavcan status
```
预期输出：
```
Online nodes (Node ID, Health, Mode):
    1 OK         OPERATIONAL
    2 OK         OPERATIONAL
    3 OK         OPERATIONAL
```

### 2. 中央飞控端：检查控制指令是否发送
```bash
# 查看执行器输出（发送源数据）
listener actuator_outputs
```

### 3. 左/右飞控端：监听接收到的控制指令
```bash
listener uavcan_control_command
```
**预期输出示例：**
```
TOPIC: uavcan_control_command
    timestamp: 123456789
    timestamp_sample: 123456000
    source_node_id: 1          # 来自中央飞控
    target_node_id: 0
    num_outputs: 4
    outputs: [1523, 1456, 1500, 1200, 0, 0, ...]
```

其中：
- `outputs[0]`: 副翼通道PWM值
- `outputs[1]`: 升降舵通道PWM值
- `outputs[2]`: 方向舵通道PWM值
- `outputs[3]`: 油门通道PWM值

### 4. 手动控制测试

在QGroundControl中：
1. 连接中央飞控
2. 切换到手动模式
3. 移动遥控器摇杆
4. 观察：
   - 中央飞控的舵机响应
   - 左右飞控通过`listener uavcan_control_command`看到相应的PWM值变化

### 5. 副翼控制测试（向左翻滚）

**预期行为：**
- 中央飞控：
  - 左副翼：PWM > 1500（上扬）
  - 右副翼：PWM < 1500（下偏）

- 左侧飞控接收：
  - `outputs[0]` > 1500（副翼全部上扬）

- 右侧飞控接收：
  - `outputs[0]` < 1500（副翼全部下偏）

**测试命令：**
```bash
# 在左侧飞控上
listener uavcan_control_command

# 在中央飞控上手动输入横滚指令，观察左侧飞控的输出变化
```

## 数据转换说明

### PWM范围映射
- **输入**：`actuator_outputs` 使用归一化值（-1.0 到 +1.0）
- **输出**：UAVCAN使用PWM微秒值（1000-2000μs）
- **转换公式**：
  ```
  PWM = 1500 + (normalized_value × 500)

  例如：
  -1.0 → 1000μs（最小）
   0.0 → 1500μs（中点）
  +1.0 → 2000μs（最大）
  ```

### 安全限制
- PWM值限制在800-2200μs范围内
- 超出范围的值会被设为安全中点（1500μs）
- 通讯中断时，左右飞控应实施故障保护

## 当前实现特点

### 已实现
✅ 控制指令广播（100Hz高频率）
✅ PWM值精确传输（1000-2000μs范围）
✅ 时间戳同步
✅ 源节点识别
✅ 前4个通道传输（副翼、升降舵、方向舵、油门）

### 待扩展
⏹ 针对特定节点的单播（当前是广播）
⏹ 更多通道支持（当前前4个，可扩展到16个）
⏹ 通讯超时保护
⏹ 控制模式切换（完全从控/混合模式）

## 下一步工作

1. **编译测试**
   ```bash
   cd /home/shiori/PX4-Autopilot
   make cuav_7-nano_default
   ```

2. **地面通讯测试**
   - 连接三个飞控到CAN总线
   - 配置参数
   - 验证通讯

3. **舵机响应测试**
   - 在左右飞控添加代码，将接收到的PWM指令应用到舵机
   - 测试实际舵面响应

4. **飞行测试**
   - 先进行地面滑行测试
   - 低速飞行测试
   - 完整机动测试

## 故障排除

### 问题：左右飞控收不到控制指令
- 检查UAVCAN总线连接
- 确认所有节点ID唯一
- 确认中央飞控 `UAVCAN_PUB_CTRL=1`
- 确认左右飞控 `UAVCAN_SUB_CTRL=1`

### 问题：PWM值不正确
- 检查中央飞控的 `actuator_outputs` 数据
- 确认归一化到PWM的转换逻辑
- 查看 `listener uavcan_control_command` 的实际接收值

### 问题：延迟过大
- 检查CAN总线波特率（建议1000000）
- 查看CPU负载
- 确认发送频率（当前100Hz）

## 文件清单

### 新增文件
1. `msg/UavcanControlCommand.msg` - uORB消息定义
2. `src/drivers/uavcan/control_command_sender.hpp` - 发送器头文件
3. `src/drivers/uavcan/control_command_sender.cpp` - 发送器实现
4. `src/drivers/uavcan/sensors/control_command.hpp` - 接收器头文件
5. `src/drivers/uavcan/sensors/control_command.cpp` - 接收器实现

### 修改文件
1. `src/drivers/uavcan/Kconfig` - 添加配置选项
2. `src/drivers/uavcan/uavcan_main.hpp` - 添加发送器声明
3. `src/drivers/uavcan/uavcan_main.cpp` - 添加发送器初始化
4. `src/drivers/uavcan/sensors/sensor_bridge.cpp` - 添加接收器初始化
5. `src/drivers/uavcan/uavcan_params.c` - 添加参数定义
6. `src/drivers/uavcan/CMakeLists.txt` - 添加编译配置
7. `msg/CMakeLists.txt` - 添加消息编译配置

