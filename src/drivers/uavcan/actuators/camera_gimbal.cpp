/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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

/**
 * @file camera_gimbal.cpp
 *
 * @author Gimbal Guys Team <mtprovin@calpoly.edu>
 */

#include "camera_gimbal.hpp"
#include <systemlib/err.h>

UavcanCameraGimbalController::UavcanCameraGimbalController(uavcan::INode &node) :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_node(node),
	_uavcan_pub_raw_cmd(node)
{
	_uavcan_pub_raw_cmd.setPriority(uavcan::TransferPriority::MiddleLower);
}

UavcanCameraGimbalController::~UavcanCameraGimbalController()
{

}

int
UavcanCameraGimbalController::init()
{
	if (!_mount_orientation_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
UavcanCameraGimbalController::Run()
{
	mount_orientation_s mount_orientation;
	if (_mount_orientation_sub.update(&mount_orientation)){
		float quaternion[4];
		float *euler = mount_orientation.attitude_euler_angle;
		matrix::Quatf(matrix::Eulerf(euler[0], euler[1], euler[2])).copyTo(quaternion);

		_cmd.gimbal_id = 0;
		//_cmd.Mode =
		for (int i=0; i < 4; i++)
			_cmd.quaternion_xyzw[i] = quaternion[i];
		_uavcan_pub_raw_cmd.broadcast(_cmd);
	}
}

void
UavcanCameraGimbalController::gimbal_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		auto &ref = _esc_status.esc[msg.esc_index];

		ref.timestamp       = hrt_absolute_time();
		ref.esc_address = msg.getSrcNodeID().get();
		ref.esc_voltage     = msg.voltage;
		ref.esc_current     = msg.current;
		ref.esc_temperature = msg.temperature;
		ref.esc_rpm         = msg.rpm;
		ref.esc_errorcount  = msg.error_count;

		_gimbal_status.esc_count = _rotor_count;
		_gimbal_status.counter += 1;
		_gimbal_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;
		_esc_status.esc_online_flags = check_escs_status();
		_esc_status.esc_armed_flags = (1 << _rotor_count) - 1;
		_gimbal_status.timestamp = hrt_absolute_time();
		_gimbal_status_pub.publish(_gimbal_status);
	}
}
