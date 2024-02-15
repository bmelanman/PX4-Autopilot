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

using math::constrain;

namespace open_gimbal
{

OutputRC::OutputRC(const Parameters &parameters)
	: OutputBase(parameters)
{
}

// Convert the range [-180, 179.999] to the range [-32768, 32767]
#define NORM_ANGLE_TO_INT16 ( 182.0416666667f )
// ( (float)( ( 1 << 16 ) - 1 ) / 360.0f ) = 182.0416666667f

/**
 * @brief Converts a target angle in degrees to a gimbal control value in the range [-1, 1]
 * @param target_angle - The target angle in degrees.
 * @param angle_offset - The angle offset in degrees.
 * @param angle_limit - The maximum/minimum angle in degrees. A value of 0 with apply no limit.
 * @return - A gimbal control value in the range [-1, 1]
 */
inline float calculate_gimbal_control(float target_angle, float angle_offset, float angle_limit)
{
	// Normalize the inputs to the range [-32768, 32767]
	// (i.e. -32768 = -180, 0 = 0, 32767 = 179.999, etc.)

	// Need to round to make sure 180 degrees overflows to -180 degrees
	int16_t target_angle_norm = (int16_t)round(target_angle * NORM_ANGLE_TO_INT16),
		angle_offset_norm = (int16_t)(angle_offset * NORM_ANGLE_TO_INT16),
		final_angle = (target_angle_norm + angle_offset_norm);

	// Apply the angle limit if it is greater than 0
	if (angle_limit > 0) {
		int16_t angle_limit_norm = (int16_t)(angle_limit * NORM_ANGLE_TO_INT16);

		final_angle = constrain(final_angle,
					(int16_t)(~(angle_limit_norm) + 1), // 2's complement
					angle_limit_norm);
	}

	// Add the offset to the target angle and normalize the result to fit within [-1, 1]
	return (float)final_angle / 32767.0f;
}

void OutputRC::update(const ControlData &control_data, bool new_setpoints, uint8_t &gimbal_device_id)
{
	// Update if we have new setpoints
	if (new_setpoints) {
		_set_angle_setpoints(control_data);
	}

	// Update if we have new control data
	_handle_position_update(control_data);

	hrt_abstime t = hrt_absolute_time();

	// Calculate the angle outputs
	_calculate_angle_output(t);

	// Publish the angle outputs
	_stream_device_attitude_status();

	// _angle_outputs are in radians, gimbal_controls are in [-1, 1]
	gimbal_controls_s gimbal_controls{};

	gimbal_controls.control[gimbal_controls_s::INDEX_ROLL] = calculate_gimbal_control(
			math::degrees(_angle_outputs[0]),
			_parameters.mnt_off_roll,
			_parameters.mnt_range_roll
		);

	gimbal_controls.control[gimbal_controls_s::INDEX_PITCH] = calculate_gimbal_control(
			math::degrees(_angle_outputs[1]),
			_parameters.mnt_off_pitch,
			_parameters.mnt_range_pitch
		);

	gimbal_controls.control[gimbal_controls_s::INDEX_YAW] = calculate_gimbal_control(
			math::degrees(_angle_outputs[2]),
			_parameters.mnt_off_yaw,
			_parameters.mnt_range_yaw
		);

	gimbal_controls.timestamp = hrt_absolute_time();
	_gimbal_controls_pub.publish(gimbal_controls);

	_last_update = t;

	//static uint16_t counter = 0;
	//if (counter++ % 500 == 0) {
	//	// Save cursor position
	//	PX4_INFO_RAW("\0337");

	//	// Print the target angles
	//	PX4_INFO_RAW("\033[1;60H"); // Move to (1, 60)
	//	PX4_INFO_RAW("Target Angles (deg):");
	//	PX4_INFO_RAW("\033[2;60H"); // Move to (2, 60)
	//	PX4_INFO_RAW("Roll:  %8.4f\n", (double)math::degrees(_angle_outputs[0]));
	//	PX4_INFO_RAW("\033[3;60H"); // Move to (3, 60)
	//	PX4_INFO_RAW("Pitch: %8.4f\n", (double)math::degrees(_angle_outputs[1]));
	//	PX4_INFO_RAW("\033[4;60H"); // Move to (4, 60)
	//	PX4_INFO_RAW("Yaw:   %8.4f\n", (double)math::degrees(_angle_outputs[2]));

	//	// Print the angles after conversion
	//	PX4_INFO_RAW("\033[6;60H"); // Move to (6, 60)
	//	PX4_INFO_RAW("Output angles (-180,180):");
	//	PX4_INFO_RAW("\033[7;60H"); // Move to (7, 60)
	//	PX4_INFO_RAW("Roll:  %8.4f\n", (double)gimbal_controls.control[gimbal_controls_s::INDEX_ROLL]);
	//	PX4_INFO_RAW("\033[8;60H"); // Move to (8, 60)
	//	PX4_INFO_RAW("Pitch: %8.4f\n", (double)gimbal_controls.control[gimbal_controls_s::INDEX_PITCH]);
	//	PX4_INFO_RAW("\033[9;60H"); // Move to (9, 60)
	//	PX4_INFO_RAW("Yaw:   %8.4f\n", (double)gimbal_controls.control[gimbal_controls_s::INDEX_YAW]);

	//	// Restore cursor position
	//	PX4_INFO_RAW("\0338");
	//}
}

void OutputRC::print_status() const
{
	PX4_INFO("Output: AUX");
}

void OutputRC::_stream_device_attitude_status()
{
	gimbal_device_attitude_status_s attitude_status{};
	attitude_status.timestamp = hrt_absolute_time();
	attitude_status.target_system = 0;
	attitude_status.target_component = 0;
	attitude_status.device_flags = gimbal_device_attitude_status_s::DEVICE_FLAGS_NEUTRAL |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_ROLL_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_PITCH_LOCK |
				       gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK;

	matrix::Eulerf euler(_angle_outputs[0], _angle_outputs[1], _angle_outputs[2]);
	matrix::Quatf q(euler);
	q.copyTo(attitude_status.q);

	attitude_status.failure_flags = 0;
	_attitude_status_pub.publish(attitude_status);
}

} /* namespace open_gimbal */
