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
