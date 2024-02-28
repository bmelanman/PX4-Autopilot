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


#include "output.h"

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/mount_orientation.h>
#include <px4_platform_common/defines.h>
#include <lib/geo/geo.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace open_gimbal
{

OutputBase::OutputBase(const Parameters &parameters)
	: _parameters(parameters)
{
	_last_update = hrt_absolute_time();
}

void OutputBase::publish()
{
	mount_orientation_s mount_orientation{};

	for (unsigned i = 0; i < 3; ++i) {
		mount_orientation.attitude_euler_angle[i] = _angle_outputs[i];
	}

	mount_orientation.timestamp = hrt_absolute_time();

	_mount_orientation_pub.publish(mount_orientation);
}

void OutputBase::_set_angle_setpoints(const ControlData &control_data)
{
	if (control_data.type != ControlData::Type::Angle) {
		PX4_ERR("_set_angle_setpoints called with invalid control_data type: %d", (int)control_data.type);
		return;
	}

	for (int i = 0; i < 3; ++i) {
		switch (control_data.type_data.angle.frames[i]) {
		case ControlData::TypeData::TypeAngle::Frame::AngleBodyFrame:
			_absolute_angle[i] = false;
			break;

		case ControlData::TypeData::TypeAngle::Frame::AngleAbsoluteFrame:
			_absolute_angle[i] = true;
			break;

		case ControlData::TypeData::TypeAngle::Frame::AngularRate:
			PX4_ERR("_set_angle_setpoints: Angular rate not supported!");
			break;

		}
	}

	_q_setpoint = matrix::Quatf(control_data.type_data.angle.q);
}

void OutputBase::_calculate_angle_output(const hrt_abstime &t)
{
	// Check if the vehicle is currently landed
	//if (_vehicle_land_detected_sub.updated()) {
	//	vehicle_land_detected_s vehicle_land_detected;
	//
	//	if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
	//		_landed = vehicle_land_detected.landed || vehicle_land_detected.maybe_landed;
	//	}
	//}

	int i;

	// Get the vehicle attitude
	vehicle_attitude_s vehicle_attitude;

	// Make sure the vehicle attitude is valid
	if (!_vehicle_attitude_sub.copy(&vehicle_attitude)) {
		PX4_ERR("Failed to get vehicle attitude! :(");

		for (i = 0; i < 3; ++i) {
			_angle_outputs[i] = 0.0f;
			_gimbal_outputs[i] = 0.0f;
		}

		return;
	}

	// Make sure the setpoints are valid
	if (!((_q_setpoint).isAllFinite())) {
		PX4_ERR("Invalid setpoint quaternions! :(");

		for (i = 0; i < 3; ++i) {
			_angle_outputs[i] = 0.0f;
			_gimbal_outputs[i] = 0.0f;
		}

		return;
	}

	// Convert the vehicle attitude and gimbal setpoint quaternions to euler angles
	matrix::Eulerf euler_vehicle((matrix::Quatf)vehicle_attitude.q);
	matrix::Eulerf euler_gimbal(_q_setpoint);

	// Get the current angle offsets and ranges
	const float offsets_rad[3] = {
		math::radians(_parameters.mnt_off_roll),
		math::radians(_parameters.mnt_off_pitch),
		math::radians(_parameters.mnt_off_yaw)
	};
	const float ranges_rad[3] = {
		math::radians(_parameters.mnt_range_roll),
		math::radians(_parameters.mnt_range_pitch),
		math::radians(_parameters.mnt_range_yaw)
	};

	float curr_angle = 0.0;//, curr_offset = 0.0, curr_range = 0.0;

	// TODO: This will need some amount of modification to allow for unlimited yaw rotation
	for (i = 0; i < 3; ++i) {

		// Stabilized output is the difference between the gimbal and vehicle angles
		curr_angle = (euler_gimbal(i) - euler_vehicle(i)) * 2.0f;

		// Add the angle offset and keep the angle in the range [-pi, pi]
		curr_angle = matrix::wrap_pi(curr_angle + offsets_rad[i]);

		// Apply the angle limit if it is within the range (0, pi)
		if (ranges_rad[i] > 0 && ranges_rad[i] < M_PI_F) {
			//_gimbal_outputs[i] = math::constrain(_gimbal_outputs[i], -ranges_rad, ranges_rad);

			if (curr_angle > ranges_rad[i]) {
				curr_angle = ranges_rad[i];

			} else if (curr_angle < -ranges_rad[i]) {
				curr_angle = -ranges_rad[i];
			}
		}

		// Add the angle to the output arrays
		_angle_outputs[i] = curr_angle;
		// Gimbal output is normalized to the range [-1, 1]
		_gimbal_outputs[i] = curr_angle / M_PI_F;
	}

	// constrain pitch to [MNT_LND_P_MIN, MNT_LND_P_MAX] if landed
	//if (_landed) {
	//	if (PX4_ISFINITE(_angle_outputs[1])) {
	//		_angle_outputs[1] = _translate_angle2gimbal(
	//					    _angle_outputs[1],
	//					    _parameters.mnt_off_pitch,
	//					    _parameters.mnt_range_pitch);
	//	}
	//}
}

void OutputBase::set_stabilize(bool roll_stabilize, bool pitch_stabilize, bool yaw_stabilize)
{
	_stabilize[0] = roll_stabilize;
	_stabilize[1] = pitch_stabilize;
	_stabilize[2] = yaw_stabilize;
}

} /* namespace open_gimbal */

