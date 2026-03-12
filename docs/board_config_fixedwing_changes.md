# CUAV FMU-V6X 固定翼配置修改说明

## 修改日期
2026年3月12日

## 修改文件
`boards/cuav/fmu-v6x/default.px4board`

## 修改内容

### 1. 新增编队飞行 UAVCAN 模块配置
```
CONFIG_UAVCAN_FORMATION_RATES_SENDER=y    # 编队主机发送模块
CONFIG_UAVCAN_SENSOR_FORMATION_RATES=y    # 编队从机接收模块
```

### 2. 新增固定翼控制模块
```
CONFIG_MODULES_FW_LATERAL_LONGITUDINAL_CONTROL=y   # 固定翼横向纵向控制
CONFIG_MODULES_FW_MODE_MANAGER=y                    # 固定翼模式管理器
```

### 3. 禁用多旋翼控制模块（固定翼不需要）
```
# CONFIG_MODULES_MC_ATT_CONTROL=y
# CONFIG_MODULES_MC_AUTOTUNE_ATTITUDE_CONTROL=y
# CONFIG_MODULES_MC_HOVER_THRUST_ESTIMATOR=y
# CONFIG_MODULES_MC_POS_CONTROL=y
# CONFIG_MODULES_MC_RATE_CONTROL=y
```

### 4. 禁用 VTOL 相关模块
```
# CONFIG_MODE_NAVIGATOR_VTOL_TAKEOFF=y
# CONFIG_MODULES_VTOL_ATT_CONTROL=y
```

### 5. 禁用其他不需要的模块
```
# CONFIG_DRIVERS_DSHOT=y                    # 多旋翼电调协议
# CONFIG_MODULES_LANDING_TARGET_ESTIMATOR=y # 精确着陆（视觉）
```

## 保留的固定翼核心模块
- `CONFIG_MODULES_FW_ATT_CONTROL=y` - 固定翼姿态控制
- `CONFIG_MODULES_FW_RATE_CONTROL=y` - 固定翼角速率控制
- `CONFIG_MODULES_FW_POS_CONTROL=y` - 固定翼位置控制
- `CONFIG_MODULES_FW_AUTOTUNE_ATTITUDE_CONTROL=y` - 固定翼自调参
- `CONFIG_MODULES_FW_LATERAL_LONGITUDINAL_CONTROL=y` - 横向纵向控制
- `CONFIG_MODULES_FW_MODE_MANAGER=y` - 模式管理器

## 编队飞行参数说明

运行时通过以下参数区分主从机角色：

| 参数 | 值 | 说明 |
|------|-----|------|
| `FORM_FOLLOWER_EN` | 0 | 主机（发送角度） |
| `FORM_FOLLOWER_EN` | 1 | 从机（接收并跟踪） |
| `FORM_POSITION` | 1/2/... | 从机编队位置编号 |
| `FORM_R2P_GAIN` | float | Roll角跟踪P控制器增益 |

## 相关文档
- [编队滚转角跟踪实现](formation_roll_tracking_implementation.md)
