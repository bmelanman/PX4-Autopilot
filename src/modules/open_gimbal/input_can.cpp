/****************************************************************************
*
*   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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


#include "input_can.h"

#include <math.h>
#include <errno.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

namespace open_gimbal
{

InputCAN::InputCAN(Parameters &parameters) :
	InputBase(parameters)
{}

InputCAN::~InputCAN()
{
	if (_gimbal_manager_set_attitude_sub >= 0) {
		orb_unsubscribe(_gimbal_manager_set_attitude_sub);
	}
}

int InputCAN::initialize()
{
	_gimbal_manager_set_attitude_sub = orb_subscribe(ORB_ID(gimbal_manager_set_attitude));

	if (_gimbal_manager_set_attitude_sub < 0) {
		return -errno;
	}

	return 0;
}

InputCAN::UpdateResult InputCAN::update(unsigned int timeout_ms, ControlData &control_data, bool already_active)
{
	px4_pollfd_struct_t polls[1];
	polls[0].fd = 		_gimbal_manager_set_attitude_sub;
	polls[0].events = 	POLLIN;

	int ret = px4_poll(polls, 1, timeout_ms);

	if (ret < 0) {
		return UpdateResult::NoUpdate;
	}

	if (ret == 0) {
		// If we have been active before, we stay active, unless someone steals
		// the control away.
		if (already_active) {
			return UpdateResult::UpdatedActive;
		}
	}

	if (polls[0].revents & POLLIN) {
		return _read_control_data_from_subscription(control_data, already_active);
	}

	return UpdateResult::NoUpdate;
}

InputCAN::UpdateResult InputCAN::_read_control_data_from_subscription(ControlData &control_data, bool already_active)
{
	gimbal_manager_set_attitude_s gimbal_manager_set_attitude{};
	orb_copy(ORB_ID(gimbal_manager_set_attitude), _gimbal_manager_set_attitude_sub, &gimbal_manager_set_attitude);
	control_data.type = ControlData::Type::Angle;

	// If we were already active previously, we just update normally. Otherwise, there needs to be
	// a major stick movement to re-activate manual (or it's running for the very first time).

	if (already_active) {
		//control_data.sysid_primary_control = _parameters.mav_sysid;
		//control_data.compid_primary_control = _parameters.mav_compid;

		// We scale manual input from roll -180..180, pitch -90..90, yaw, -180..180 degrees.

		for(int i = 0; i < 4; i++) {
			control_data.type_data.angle.q[i] = gimbal_manager_set_attitude.q[i];
		}

		control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
		control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
		control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

		control_data.type_data.angle.angular_velocity[0] = NAN;
		control_data.type_data.angle.angular_velocity[1] = NAN;
		control_data.type_data.angle.angular_velocity[2] = NAN;

		return UpdateResult::UpdatedActive;

	} else {
		return UpdateResult::NoUpdate;
	}
}

void InputCAN::print_status() const
{
	PX4_INFO("Input: RC (channels: roll=%" PRIi32 ", pitch=%" PRIi32 ", yaw=%" PRIi32 ")", _parameters.mnt_man_roll,
		 _parameters.mnt_man_pitch, _parameters.mnt_man_yaw);
}

} /* namespace open_gimbal */
