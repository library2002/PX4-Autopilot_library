# UAVCAN Control Command Message
# 用于通过UAVCAN传输控制指令（从中央飞控到左右飞控）

uint64 timestamp                # 指令生成时间戳（微秒，使用 hrt_absolute_time()）
uint64 timestamp_sample         # 控制数据采样时间戳（微秒）

uint8 target_node_id           # 目标飞控节点ID（1-127，0表示广播给所有节点）
uint8 source_node_id           # 源节点ID（发送方的节点ID）

uint8 num_outputs              # 有效输出数量（1-16）
uint16[16] outputs             # PWM输出数组（微秒，1000-2000，0表示该通道未使用）

# 通道定义示例（可根据实际配置调整）：
# outputs[0] = 副翼（Aileron）
# outputs[1] = 升降舵（Elevator）
# outputs[2] = 方向舵（Rudder）
# outputs[3] = 油门（Throttle）
# outputs[4-15] = 保留用于其他舵面或执行器
