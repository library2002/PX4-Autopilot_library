# 主机偏航差速功能实现

## 概述

在编队飞行中，主机（Leader）在转弯时需要增加推力以补偿阻力增加。本功能在 `FixedwingRateControl` 模块中实现了基于偏航输入的油门增量。

## 功能描述

当飞行员打方向舵（偏航输入）时，主机的油门会自动增加：

```
油门增量 = 0.5 × |yaw| × FORM_THR_DIFF
```

- **|yaw|**: 偏航输入的绝对值（范围 0~1）
- **0.5**: 固定系数，限制最大增量
- **FORM_THR_DIFF**: 差速增益参数（可通过地面站调整）

**特点**：
- 无论向左还是向右转弯，油门都会增加
- 使用绝对值，避免一侧增加一侧减少的问题

## 代码修改

### 1. FixedwingRateControl.hpp

添加参数句柄和成员变量：

```cpp
param_t _handle_param_form_thr_diff{PARAM_INVALID};
float _param_form_thr_diff{0.0f};
```

### 2. FixedwingRateControl.cpp

#### 构造函数
```cpp
_handle_param_form_thr_diff = param_find("FORM_THR_DIFF");
```

#### parameters_update()
```cpp
if (_handle_param_form_thr_diff != PARAM_INVALID) {
    param_get(_handle_param_form_thr_diff, &_param_form_thr_diff);
}
```

#### 速率控制模式 (ACRO)
```cpp
_rates_sp.thrust_body[0] = (_manual_control_setpoint.throttle + 1.f) * .5f
                          + 0.5f * fabsf(_manual_control_setpoint.yaw) * _param_form_thr_diff;
```

#### 手动/稳定模式
```cpp
_vehicle_thrust_setpoint.xyz[0] = math::constrain(
    (_manual_control_setpoint.throttle + 1.f) * .5f
    + 0.5f * fabsf(_manual_control_setpoint.yaw) * _param_form_thr_diff,
    0.f, 1.f);
```

## 参数说明

| 参数名 | 默认值 | 范围 | 说明 |
|--------|--------|------|------|
| FORM_THR_DIFF | 0.0 | 0.0~1.0 | 偏航差速增益。设为0禁用此功能 |

## 适用模式

- ACRO 模式（特技飞行）
- Manual 模式（手动）
- Stabilized 模式（自稳）

## 与从机的配合

主机和从机使用相同的 `FORM_THR_DIFF` 参数：

| 飞机 | 油门差速逻辑 |
|------|-------------|
| 主机 | `+0.5 × |yaw| × FORM_THR_DIFF`（任意方向转弯都增加） |
| 左机 | `+max(yaw, 0) × FORM_THR_DIFF`（右转时增加，外侧加速） |
| 右机 | `-min(yaw, 0) × FORM_THR_DIFF`（左转时增加，外侧加速） |

## 编译验证

已通过以下目标编译验证：
- `make px4_fmu-v6x_default` ✓
- `make cuav_7-nano_default` ✓

## 文件列表

- `src/modules/fw_rate_control/FixedwingRateControl.hpp`
- `src/modules/fw_rate_control/FixedwingRateControl.cpp`

## 相关文档

- [UAVCAN 编队速率控制实现](uavcan_formation_rates_implementation.md)
- [编队滚转跟踪实现](formation_roll_tracking_implementation.md)
- [板级配置修改](board_config_fixedwing_changes.md)

## CAN 通讯提速建议（2026-03-15）

### 目标

在不改变当前控制逻辑的前提下，提升编队指令的发送与接收实时性。

### 已确认现状

- `formation_rates_sender` 当前发送频率上限为 `100Hz`（代码常量 `MAX_RATE_HZ`）
- 发送优先级为 `OneLowerThanHighest`
- 当前实现稳定可用

### 提速建议（未实施）

1. 将发送频率从 `100Hz` 提高到 `200Hz`（先做首轮验证）
2. 将 UAVCAN 发布优先级提升到 `Highest`
3. 确认总线波特率统一为 `1Mbps`（主从一致）
4. 接收侧参数读取改为“参数更新触发刷新”，减少每包 `param_get` 开销

### 本轮结论

- 用户决定：**暂时不修改代码**
- 现阶段保持当前实现，后续如需提速再按上述方案逐项落地和验证
