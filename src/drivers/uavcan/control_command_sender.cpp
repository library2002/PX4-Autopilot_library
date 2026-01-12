/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "control_command_sender.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

UavcanControlCommandSender::UavcanControlCommandSender(uavcan::INode &node) :
	_publisher(node),
	_timer(node)

{	// 控制指令使用 仅次于最高，确保控制指令及时传输
	_publisher.setPriority(uavcan::TransferPriority::OneLowerThanHighest);
}

int UavcanControlCommandSender::init()
{
	// 定时器周期性调用更新
	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &UavcanControlCommandSender::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}

	return 0;
}

uint16_t UavcanControlCommandSender::normalize_to_pwm(float normalized_value)
{
	// 限制输入范围到 -1.0 ~ +1.0
	float clamped = math::constrain(normalized_value, -1.0f, 1.0f);

	// 转换为PWM范围 (1000-2000微秒)
	// -1.0 -> 1000us, 0.0 -> 1500us, +1.0 -> 2000us
	uint16_t pwm = static_cast<uint16_t>(PWM_CENTER + (clamped * (PWM_MAX - PWM_CENTER)));

	return pwm;
}

void UavcanControlCommandSender::periodic_update(const uavcan::TimerEvent &)
{
	actuator_outputs_s outputs{};

	// 尝试获取最新的执行器输出
	if (!_actuator_outputs_sub.update(&outputs)) {
		// 如果没有新数据，尝试复制最后一次的数据
		if (!_actuator_outputs_sub.copy(&outputs)) {
			return;  // 没有可用数据
		}
	}

	uavcan::equipment::actuator::ArrayCommand msg;

	// 确定有效输出数量
	uint32_t num_outputs = math::min(outputs.noutputs, (uint32_t)MAX_OUTPUTS);

	// 只发送前几个通道（用于测试副翼控制）
	// 后续可以通过参数配置具体发送哪些通道
	// 暂时发送前4个通道：副翼、升降舵、方向舵、油门
	num_outputs = math::min(num_outputs, (uint32_t)4);

	// 构建命令数组
	for (uint32_t i = 0; i < num_outputs; i++) {
		uavcan::equipment::actuator::Command cmd;

		// 设置执行器ID（对应通道索引）
		// 0 = 副翼, 1 = 升降舵, 2 = 方向舵, 3 = 油门
		cmd.actuator_id = i;

		// 设置命令类型为PWM
		// UAVCAN支持多种命令类型：
		// - COMMAND_TYPE_UNITLESS = 0  (无单位, -1到+1)
		// - COMMAND_TYPE_POSITION = 1  (位置, 米或弧度)
		// - COMMAND_TYPE_FORCE = 2     (力, 牛顿或牛顿米)
		// - COMMAND_TYPE_SPEED = 3     (速度, m/s或rad/s)
		// - COMMAND_TYPE_PWM = 4       (PWM微秒值)
		cmd.command_type = uavcan::equipment::actuator::Command::COMMAND_TYPE_PWM;

		// 转换归一化值为PWM微秒值
        	// outputs.output[i]: 归一化值（-1.0到+1.0）
        	// normalize_to_pwm(): 转换为PWM（1000-2000μs）
      		  // static_cast<float>(): 转换为float类型（UAVCAN要求）
		cmd.command_value = static_cast<float>(normalize_to_pwm(outputs.output[i]));

		// 添加到命令数组
		msg.commands.push_back(cmd);
	}

	// 广播UAVCAN消息
	if (msg.commands.size() > 0) {
		(void)_publisher.broadcast(msg);
	}
}
