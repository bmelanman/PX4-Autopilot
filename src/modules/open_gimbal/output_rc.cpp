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

//#include <uORB/topics/vehicle_attitude.h>

namespace open_gimbal
{

OutputRC::OutputRC(const Parameters &parameters)
	: OutputBase(parameters)
{
}

int OutputRC::update(const ControlData &control_data, bool new_setpoints)
{
	// Update if we have new setpoints
	if (new_setpoints && _set_angle_setpoints(control_data) != PX4_OK) {
		return PX4_ERROR;
	}

	hrt_abstime t = hrt_absolute_time();

	// Calculate the angle outputs
	if (_calculate_angle_output(t) == PX4_ERROR) {
		return PX4_ERROR;
	}

	// Publish the angle outputs
	_stream_device_attitude_status();

	// Publish the gimbal control outputs
	gimbal_controls_s gimbal_controls{};

	gimbal_controls.control[OutputBase::_INDEX_ROLL] = _gimbal_output_norm[OutputBase::_INDEX_ROLL];
	gimbal_controls.control[OutputBase::_INDEX_PITCH] = _gimbal_output_norm[OutputBase::_INDEX_PITCH];
	gimbal_controls.control[OutputBase::_INDEX_YAW] = _gimbal_output_norm[OutputBase::_INDEX_YAW];

	gimbal_controls.timestamp = hrt_absolute_time();
	_gimbal_controls_pub.publish(gimbal_controls);

	_last_update_usec = t;

	return PX4_OK;
}

inline void _print_euler(const matrix::Eulerf &euler, bool rad_to_deg = false)
{
	if (rad_to_deg) {
		PX4_INFO_RAW("  Roll:  %8.4f \n", (double)(euler(0) * M_RAD_TO_DEG_F));
		PX4_INFO_RAW("  Pitch: %8.4f \n", (double)(euler(1) * M_RAD_TO_DEG_F));
		PX4_INFO_RAW("  Yaw:   %8.4f \n", (double)(euler(2) * M_RAD_TO_DEG_F));

	} else {
		PX4_INFO_RAW("  Roll:  %8.4f \n", (double)euler(0));
		PX4_INFO_RAW("  Pitch: %8.4f \n", (double)euler(1));
		PX4_INFO_RAW("  Yaw:   %8.4f \n", (double)euler(2));
	}
}

inline void _print_euler(const float euler[3], bool r2d = false)
{
	_print_euler(matrix::Eulerf{euler[0], euler[1], euler[2]}, r2d);
}

inline void _print_quat(const float q[4], bool r2d = false)
{
	// Convert the quaternion to Euler angles
	_print_euler(matrix::Quatf{q}, r2d);
}

void OutputRC::print_status() const
{
	PX4_INFO("Output: AUX\n");

	static uORB::Subscription _status_vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	static vehicle_attitude_s vehicle_attitude{0};
	static matrix::Eulerf veh_att{0, 0, 0};

	PX4_INFO_RAW("\n/* Vehicle Attitude ***************************/\n");

	// Get the current vehicle attitude
	if (_status_vehicle_attitude_sub.update(&vehicle_attitude)) {
		veh_att = matrix::Eulerf(matrix::Quatf(vehicle_attitude.q));
		_print_euler(veh_att, true);

	} else {
		PX4_INFO("Vehicle attitude not available\n");
	}

	matrix::Eulerf motor_offsets{
		_parameters.mnt_motor_roll,
		_parameters.mnt_motor_pitch,
		_parameters.mnt_motor_yaw
	};

	PX4_INFO_RAW("\n/* Vehicle Attitude with Motor Offset *********/\n");
	_print_euler(veh_att + motor_offsets, true);

	PX4_INFO_RAW("\n/* Target Setpoint ****************************/\n");
	_print_euler(_euler_setpoint, true);

	PX4_INFO_RAW("\n/* Output Angles ******************************/\n");
	_print_euler(_gimbal_output_rad, true);

	PX4_INFO_RAW("\n/* Output to Motors ***************************/\n");
	_print_euler(_gimbal_output_norm);

	PX4_INFO_RAW("\n/**********************************************/\n");
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
		_gimbal_output_norm[OutputBase::_INDEX_ROLL] * 180.0f,
		_gimbal_output_norm[OutputBase::_INDEX_PITCH] * 180.0f,
		_gimbal_output_norm[OutputBase::_INDEX_YAW] * 180.0f
	);

	//(matrix::Quatf(euler)).copyTo(attitude_status.q);

	attitude_status.failure_flags = 0;
	_attitude_status_pub.publish(attitude_status);
}

} /* namespace open_gimbal */
