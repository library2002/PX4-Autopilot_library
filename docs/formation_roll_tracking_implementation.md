# 编队从机滚转角跟踪功能实现记录

## 1. 功能概述

为编队控制系统新增**从机滚转角跟踪功能**，从机通过订阅本机姿态，使用 P 控制器跟踪主机发送的目标滚转角，生成滚转速率指令。

**核心机制：**
```
主机滚转角 ──UAVCAN──► 从机接收 ──► P控制器 ──► 滚转速率指令
                           ↑
                      从机当前滚转角
```

**控制公式：**
```
roll_rate = Kp × (target_roll - self_roll)
          = FORM_R2P_GAIN × (主机滚转角 - 从机当前滚转角)
```

## 2. 修改的文件

| 文件 | 修改内容 |
| ---- | -------- |
| `src/drivers/uavcan/sensors/formation_rates.hpp` | 添加姿态订阅、matrix 库、FORM_R2P_GAIN 参数 |
| `src/drivers/uavcan/sensors/formation_rates.cpp` | 实现滚转角跟踪 P 控制器 |
| `src/drivers/uavcan/formation_rates_sender.cpp` | 发送主机实际滚转角（而非常量增益） |

## 3. 详细修改

### 3.1 formation_rates.hpp

**新增头文件：**
```cpp
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/math.hpp>
```

**新增成员变量：**
```cpp
// 姿态订阅
uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
vehicle_attitude_s _vehicle_attitude{};

// 参数句柄
param_t _param_roll_to_pitch_gain_h;

// 参数缓存值
float _roll_to_pitch_gain{1.0f};
```

### 3.2 formation_rates.cpp

**构造函数中查找参数：**
```cpp
_param_roll_to_pitch_gain_h = param_find("FORM_R2P_GAIN");
```

**init() 中加载参数：**
```cpp
if (_param_roll_to_pitch_gain_h != PARAM_INVALID) {
    param_get(_param_roll_to_pitch_gain_h, &_roll_to_pitch_gain);
}
```

**回调函数中实现滚转角跟踪：**
```cpp
// 获取当前飞机姿态
if (_vehicle_attitude_sub.updated()) {
    _vehicle_attitude_sub.copy(&_vehicle_attitude);
}

// 四元数转欧拉角
matrix::Quatf q(_vehicle_attitude.q);
matrix::Eulerf euler(q);
float self_roll = euler.phi();    // 从机自身的滚转角 (rad)

// ... 解码 UAVCAN 消息获取 roll (目标滚转角) ...

// P 控制器计算滚转速率
rates_sp.roll = _roll_to_pitch_gain * (roll - self_roll);
```

### 3.3 formation_rates_sender.cpp

**修改 actuator_id=100 的发送内容：**

修改前（发送常量增益）：
```cpp
left_roll.command_value = _roll_to_pitch_gain;
```

修改后（发送主机实际滚转角）：
```cpp
// 需要订阅 vehicle_attitude 获取主机滚转角
left_roll.command_value = master_roll_angle;  // 主机当前滚转角 (rad)
```

## 4. 数据流

```
主机端:
┌─────────────────────────────────────────────────────┐
│  vehicle_attitude                                   │
│       │                                             │
│       ▼                                             │
│  主机滚转角 ──► actuator_id=100 ──► UAVCAN 广播    │
└─────────────────────────────────────────────────────┘
                        │
                        ▼
从机端:
┌─────────────────────────────────────────────────────┐
│  UAVCAN 接收 ──► roll (目标滚转角)                  │
│                       │                             │
│  vehicle_attitude ──► self_roll (从机当前滚转角)    │
│                       │                             │
│                       ▼                             │
│  roll_rate = Kp × (roll - self_roll)               │
│                       │                             │
│                       ▼                             │
│  vehicle_rates_setpoint ──► fw_rate_control        │
└─────────────────────────────────────────────────────┘
```

## 5. 控制效果

| 情况 | 角度误差 | 滚转速率 | 效果 |
| ---- | -------- | -------- | ---- |
| 从机滚转角 < 主机滚转角 | 正 | 正（向右滚转） | 从机追上主机 |
| 从机滚转角 > 主机滚转角 | 负 | 负（向左滚转） | 从机追上主机 |
| 从机滚转角 = 主机滚转角 | 0 | 0 | 同步保持 |

## 6. 参数说明

| 参数 | 类型 | 默认值 | 说明 |
| ---- | ---- | ------ | ---- |
| `FORM_R2P_GAIN` | FLOAT | 2.0 | P 控制器增益 Kp，值越大跟踪响应越快 |

**调参建议：**
- 1.0-2.0：响应较慢，稳定性好
- 2.0-3.0：响应适中，推荐范围
- 3.0+：响应快，可能产生振荡

## 7. Git 提交记录

```
commit fa924b273b
Author: library2002
Date:   2026-03-11

feat(formation): add roll angle tracking for follower aircraft

- Add vehicle_attitude subscription to formation_rates receiver
- Implement P controller for roll angle tracking: rates_sp.roll = Kp * (target_roll - self_roll)
- Use FORM_R2P_GAIN parameter as proportional gain
- Modify formation_rates_sender to send actual roll angle instead of constant
- Remove unused control_command_sender files
- Fix code indentation issues
```

## 8. 删除的文件

- `src/drivers/uavcan/control_command_sender.cpp`
- `src/drivers/uavcan/control_command_sender.hpp`

这两个文件是旧的控制指令发送器，功能已被 `formation_rates_sender` 取代。

---
