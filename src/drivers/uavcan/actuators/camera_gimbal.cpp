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
 * @author Matthew Province, Gimbal Guys Team <mtprovin@calpoly.edu>
 */

#include "camera_gimbal.hpp"
#include <systemlib/err.h>
#include <uavcan/equipment/camera_gimbal/Mode.hpp>

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
	static mount_orientation_s mount_orientation = { 0 };
	static matrix::Quatf quat(0.0f, 0.0f, 0.0f, 0.0f);

	if (_mount_orientation_sub.update(&mount_orientation)) {

		// Convert from euler angles to quaternions
		quat = matrix::Quatf(matrix::Eulerf(
					     mount_orientation.attitude_euler_angle[0],
					     mount_orientation.attitude_euler_angle[1],
					     mount_orientation.attitude_euler_angle[2]
				     ));

		// Publish the quaternion to the gimbal
		_cmd.gimbal_id = 0;
		_cmd.quaternion_xyzw[0] = quat(0);
		_cmd.quaternion_xyzw[1] = quat(1);
		_cmd.quaternion_xyzw[2] = quat(2);
		_cmd.quaternion_xyzw[3] = quat(3);

		if (_uavcan_pub_raw_cmd.broadcast(_cmd) < 0) {
			PX4_ERR("UavcanCameraGimbalController: Error publishing uavcan::equipment::camera_gimbal::AngularCommand");
		}
	}
}
