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

#include "control_command.hpp"

const char *const UavcanControlCommandBridge::NAME = "control_command";

UavcanControlCommandBridge::UavcanControlCommandBridge(uavcan::INode &node, NodeInfoPublisher *node_info_publisher) :
	UavcanSensorBridgeBase("uavcan_control_command", ORB_ID(uavcan_control_command), node_info_publisher, 8),
	_sub_control_command(node)
{ }

int UavcanControlCommandBridge::init()
{
	int res = _sub_control_command.start(ControlCommandCbBinder(this, &UavcanControlCommandBridge::control_command_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start control command sub: %d", res);
		return res;
	}

	return PX4_OK;
}

void UavcanControlCommandBridge::control_command_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::actuator::ArrayCommand> &msg)
{
	uavcan_control_command_s report{};

	report.timestamp = hrt_absolute_time();
	report.timestamp_sample = msg.getMonotonicTimestamp().toUSec();
	report.source_node_id = static_cast<uint8_t>(msg.getSrcNodeID().get());

	// 暂时将target_node_id设为本节点（可以在未来扩展为多目标）
	// 这里假设所有接收到的指令都是发给本节点的
	report.target_node_id = 0; // 0表示接受所有广播指令

	// 解析控制指令数组
	report.num_outputs = msg.commands.size();

	if (report.num_outputs > 16) {
		report.num_outputs = 16; // 限制最大数量
	}

	// 初始化所有输出为0
	for (unsigned i = 0; i < 16; i++) {
		report.outputs[i] = 0;
	}

	// 提取PWM值
	for (unsigned i = 0; i < report.num_outputs; i++) {
		const auto &cmd = msg.commands[i];

		// 检查命令类型是否为PWM
		if (cmd.command_type == uavcan::equipment::actuator::Command::COMMAND_TYPE_PWM) {
			// command_value 包含PWM微秒值（通常1000-2000）
			uint16_t pwm_value = static_cast<uint16_t>(cmd.command_value);

			// 确保PWM值在合理范围内
			if (pwm_value >= 800 && pwm_value <= 2200) {
				report.outputs[i] = pwm_value;
			} else {
				// 如果超出范围，设置为安全值（中立位置1500）
				report.outputs[i] = 1500;
			}
		} else {
			// 如果不是PWM类型，可以根据需要进行转换
			// 暂时设为中立位置
			report.outputs[i] = 1500;
		}
	}

	// 发布到uORB
	publish(msg.getSrcNodeID().get(), &report);
}

int UavcanControlCommandBridge::init_driver(uavcan_bridge::Channel *channel)
{
	(void)channel;
	return PX4_OK;
}
