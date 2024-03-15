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

#include <errno.h>

#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include <uORB/topics/gimbal_manager_set_attitude.h>

namespace open_gimbal
{

InputCAN::InputCAN(Parameters &parameters) :
	InputBase(parameters)
{}

InputCAN::~InputCAN()
{
	_gimbal_manager_set_attitude_sub.unsubscribe();
}

int InputCAN::initialize()
{
	_gimbal_manager_set_attitude_sub.subscribe();

	return PX4_OK;
}

InputCAN::UpdateResult InputCAN::update(unsigned int timeout_ms, ControlData &control_data, bool already_active)
{
	// Read the new data from the gimbal_manager_set_attitude topic only if there is new data
	if (_gimbal_manager_set_attitude_sub.update(&_gimbal_manager_set_attitude)
	    && _gimbal_manager_set_attitude.q != control_data.type_data.angle.q) {

		// Update the setpoint
		(matrix::Quatf(_gimbal_manager_set_attitude.q)).copyTo(control_data.type_data.angle.q);

		control_data.type_data.angle.frames[0] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
		control_data.type_data.angle.frames[1] = ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame;
		control_data.type_data.angle.frames[2] = ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame;

		control_data.type_data.angle.angular_velocity[0] = NAN;
		control_data.type_data.angle.angular_velocity[1] = NAN;
		control_data.type_data.angle.angular_velocity[2] = NAN;

		// Debug: Print a status message
		PX4_INFO("Received new gimbal attitude from InputCAN!");

		return UpdateResult::UpdatedActive;
	}

	// No new data
	return UpdateResult::NoUpdate;
}

void InputCAN::print_status() const
{
	PX4_INFO("Input: CAN");
}

} /* namespace open_gimbal */
