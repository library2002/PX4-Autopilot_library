# Formation Rates Setpoint Message
# 用于三机编队控制：主飞控通过UAVCAN向从飞控发送速率指令

uint64 timestamp                # 指令生成时间戳（微秒）
uint64 timestamp_sample         # 控制数据采样时间戳（微秒）

uint8 target_node_id           # 目标飞控节点ID（1=主机, 2=左机, 3=右机, 0=广播）
uint8 source_node_id           # 源节点ID（发送方的节点ID，通常是主机=1）

# 角速率设定值（rad/s）
float32 roll                   # 滚转角速度 (rad/s)
float32 pitch                  # 俯仰角速度 (rad/s) - 从机主要控制通道！
float32 yaw                    # 偏航角速度 (rad/s)

# 推力设定值
float32[3] thrust_body         # 机体坐标系推力 [前/右/下]，归一化 (0-1)

# 控制标志
bool reset_integral            # 复位速率控制器积分器标志

# 编队位置标识（用于从机识别）
uint8 POSITION_CENTER = 0      # 中央主机
uint8 POSITION_LEFT = 1        # 左侧从机
uint8 POSITION_RIGHT = 2       # 右侧从机
uint8 formation_position       # 目标飞机在编队中的位置

# TOPICS formation_rates_setpoint
