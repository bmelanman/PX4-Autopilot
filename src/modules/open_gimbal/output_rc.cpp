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


#include "output_rc.h"

#include <uORB/topics/gimbal_controls.h>
#include <px4_platform_common/defines.h>
#include <matrix/matrix/math.hpp>

namespace open_gimbal
{

OutputRC::OutputRC(const Parameters &parameters)
	: OutputBase(parameters)
{
}

int OutputRC::update(const ControlData &control_data, bool new_setpoints)
{
	// Update if we have new setpoints
	if (new_setpoints) {

		// Only support angle setpoints
		if (control_data.type == ControlData::Type::Angle) {
			_q_setpoint = matrix::Quatf(control_data.type_data.angle.q);

		} else {
			PX4_ERR("ControlData type '%d' not supported", (int)control_data.type);
			return PX4_ERROR;
		}
	}

	hrt_abstime t = hrt_absolute_time();

	// Calculate the angle outputs
	if (_calculate_angle_output(t, control_data.q_zero_setpoint) == PX4_ERROR) {
		return PX4_ERROR;
	}

	// Publish the angle outputs
	_stream_device_attitude_status();

	// Publish the gimbal control outputs
	gimbal_controls_s gimbal_controls{};

	gimbal_controls.control[OutputBase::_INDEX_ROLL] = _gimbal_outputs[OutputBase::_INDEX_ROLL];
	gimbal_controls.control[OutputBase::_INDEX_PITCH] = _gimbal_outputs[OutputBase::_INDEX_PITCH];
	gimbal_controls.control[OutputBase::_INDEX_YAW] = _gimbal_outputs[OutputBase::_INDEX_YAW];

	gimbal_controls.timestamp = hrt_absolute_time();
	_gimbal_controls_pub.publish(gimbal_controls);

	_last_update = t;

	return PX4_OK;
}

void OutputRC::print_status() const
{
	PX4_INFO("Output: AUX");

	// Print the target angles
	PX4_INFO("Target Angles (deg):");
	PX4_INFO_RAW("  Roll:  % 4.1f\n", (double)(OutputBase::_angle_outputs_deg[OutputBase::_INDEX_ROLL]));
	PX4_INFO_RAW("  Pitch: % 4.1f\n", (double)(OutputBase::_angle_outputs_deg[OutputBase::_INDEX_PITCH]));
	PX4_INFO_RAW("  Yaw:   % 4.1f\n", (double)(OutputBase::_angle_outputs_deg[OutputBase::_INDEX_YAW]));

	// Print the angles after conversion
	PX4_INFO("Converted Angles (-1,1):");
	PX4_INFO_RAW("  Roll:  % 4.1f\n", (double)_gimbal_outputs[OutputBase::_INDEX_ROLL]);
	PX4_INFO_RAW("  Pitch: % 4.1f\n", (double)_gimbal_outputs[OutputBase::_INDEX_PITCH]);
	PX4_INFO_RAW("  Yaw:   % 4.1f\n", (double)_gimbal_outputs[OutputBase::_INDEX_YAW]);
	PX4_INFO_RAW("%c", '\0');
}

void OutputRC::_stream_device_attitude_status()
{
	// Publish the attitude status
	gimbal_device_attitude_status_s attitude_status{};

	attitude_status.timestamp = hrt_absolute_time();
	attitude_status.target_system = 0;
	attitude_status.target_component = 0;
	attitude_status.device_flags = gimbal_device_attitude_status_s::DEVICE_FLAGS_NEUTRAL    |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_ROLL_LOCK  |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_PITCH_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK;

	matrix::Eulerf euler(
		_gimbal_outputs[OutputBase::_INDEX_ROLL] * 180.0f,
		_gimbal_outputs[OutputBase::_INDEX_PITCH] * 180.0f,
		_gimbal_outputs[OutputBase::_INDEX_YAW] * 180.0f
	);

	(matrix::Quatf(euler)).copyTo(attitude_status.q);

	attitude_status.failure_flags = 0;
	_attitude_status_pub.publish(attitude_status);
}

} /* namespace open_gimbal */
