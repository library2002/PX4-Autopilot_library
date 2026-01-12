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

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_outputs.h>

/**
 * @brief UAVCAN控制指令发送器
 *
 * 订阅本机的actuator_outputs，将其转换为UAVCAN ArrayCommand消息并广播
 * 用于链翼组合飞行器的中央主飞控，向左右从飞控发送控制指令
 */
class UavcanControlCommandSender
{
public:
	UavcanControlCommandSender(uavcan::INode &node);

	/**
	 * 初始化周期性发布器
	 */
	int init();

private:
	static constexpr unsigned MAX_RATE_HZ = 100;  // 最大发布频率：100Hz
	static constexpr unsigned MAX_OUTPUTS = 16;    // 最大输出数量

	// PWM转换常数（假设标准舵机范围）
	static constexpr uint16_t PWM_MIN = 1000;      // 最小PWM值（微秒）
	static constexpr uint16_t PWM_MAX = 2000;      // 最大PWM值（微秒）
	static constexpr uint16_t PWM_CENTER = 1500;   // 中心PWM值（微秒）

	void periodic_update(const uavcan::TimerEvent &);  // 周期性更新函数

	/**
	 * 将归一化的输出值转换为PWM微秒值
	 * @param normalized_value 归一化值（通常-1.0到+1.0或0.0到1.0）
	 * @return PWM微秒值（1000-2000）
	 */
	uint16_t normalize_to_pwm(float normalized_value);

	typedef uavcan::MethodBinder<UavcanControlCommandSender *, void (UavcanControlCommandSender::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand> _publisher;
	uavcan::TimerEventForwarder<TimerCbBinder> _timer;

	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
};
