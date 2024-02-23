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

float OutputBase::_calculate_pitch(double lon, double lat, float altitude,
				   const vehicle_global_position_s &global_position)
{
	if (!_projection_reference.isInitialized()) {
		_projection_reference.initReference(global_position.lat, global_position.lon);
	}

	float x1, y1, x2, y2;
	_projection_reference.project(lat, lon, x1, y1);
	_projection_reference.project(global_position.lat, global_position.lon, x2, y2);
	float dx = x1 - x2, dy = y1 - y2;
	float target_distance = sqrtf(dx * dx + dy * dy);
	float z = altitude - global_position.alt;

	return atan2f(z, target_distance);
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

void OutputBase::_handle_position_update(const ControlData &control_data, bool force_update)
{
	if (control_data.type != ControlData::Type::LonLat) {
		return;
	}

	vehicle_global_position_s vehicle_global_position{};

	if (force_update) {
		_vehicle_global_position_sub.copy(&vehicle_global_position);

	} else {
		if (!_vehicle_global_position_sub.update(&vehicle_global_position)) {
			return;
		}
	}

	const double &vlat = vehicle_global_position.lat;
	const double &vlon = vehicle_global_position.lon;

	const double &lat = control_data.type_data.lonlat.lat;
	const double &lon = control_data.type_data.lonlat.lon;
	const float &alt = control_data.type_data.lonlat.altitude;

	float roll = PX4_ISFINITE(control_data.type_data.lonlat.roll_offset)
		     ? control_data.type_data.lonlat.roll_offset
		     : 0.0f;

	// interface: use fixed pitch value > -pi otherwise consider ROI altitude
	float pitch = (control_data.type_data.lonlat.pitch_fixed_angle >= -M_PI_F) ?
		      control_data.type_data.lonlat.pitch_fixed_angle :
		      _calculate_pitch(lon, lat, alt, vehicle_global_position);

	float yaw = get_bearing_to_next_waypoint(vlat, vlon, lat, lon);
	// We set the yaw angle in the absolute frame in this case.
	_absolute_angle[2] = true;

	// add offsets from VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET
	if (PX4_ISFINITE(control_data.type_data.lonlat.pitch_offset)) {
		pitch += control_data.type_data.lonlat.pitch_offset;
	}

	if (PX4_ISFINITE(control_data.type_data.lonlat.yaw_offset)) {
		yaw += control_data.type_data.lonlat.yaw_offset;
	}

	//matrix::Quatf(matrix::Eulerf(roll, pitch, yaw)).copyTo(_q_setpoint);
	_q_setpoint = matrix::Quatf(matrix::Eulerf(roll, pitch, yaw));

	_angle_velocity[0] = NAN;
	_angle_velocity[1] = NAN;
	_angle_velocity[2] = NAN;
}

///**
// * @brief Converts a target angle in radians to a gimbal control value in the range [-1, 1]
// * @param target_angle - The target angle in radians.
// * @param angle_offset - The angle offset in radians.
// * @param angle_limit - The maximum/minimum angle in radians. A value of 0 with apply no limit.
// * @return - A gimbal control value in the range [-1, 1]
// */
//float OutputBase::_translate_angle2gimbal(float target_angle, float angle_offset, float angle_limit)
//{
//	// Apply the angle offset
//	target_angle += angle_offset;

//	// Apply the angle limit if it is within the range (0, pi)
//	if (angle_limit > 0 && angle_limit < M_PI_F) {
//		//target_angle = math::constrain(target_angle, -angle_limit, angle_limit);

//		if (target_angle > angle_limit) {
//			target_angle = angle_limit;

//		} else if (target_angle < -angle_limit) {
//			target_angle = -angle_limit;
//		}
//	}

//	// Normalize the target angle to the range [-1, 1]
//	return (target_angle + angle_offset) / M_PI_F;
//}

void OutputBase::_calculate_angle_output(const hrt_abstime &t)
{
	//if (_vehicle_land_detected_sub.updated()) {
	//	vehicle_land_detected_s vehicle_land_detected;
	//
	//	if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
	//		_landed = vehicle_land_detected.landed || vehicle_land_detected.maybe_landed;
	//	}
	//}
	//
	//// Calculate the time delta (must be between 1ms and 1s)
	//float dt = math::constrain((t - _last_update) * 1.e-6f, 0.001f, 1.f);

	// Get the vehicle attitude
	vehicle_attitude_s vehicle_attitude;

	// Make sure the vehicle attitude is valid
	if (!_vehicle_attitude_sub.copy(&vehicle_attitude)) {
		PX4_ERR("Failed to get vehicle attitude! :(");
		return;
	}

	// Make sure the setpoints are valid
	if (!((_q_setpoint).isAllFinite())) {
		PX4_ERR("Invalid setpoint quaternions! :(");
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

	// TODO: This will need some amount of modification to allow for unlimited yaw rotation
	for (int i = 0; i < 3; ++i) {

		// Stabilized output is the difference between the gimbal and vehicle angles
		_angle_outputs[i] = (euler_gimbal(i) - euler_vehicle(i)) * 2.0f;

		// Calculate the gimbal output in the range [-1, 1]
		//_gimbal_outputs[i] = _translate_angle2gimbal(_angle_outputs[i], offsets_rad[i], ranges_rad[i]);

		// Add the angle offset and wrap the gimbal output to the range [-pi, pi]
		_gimbal_outputs[i] = matrix::wrap_pi(_angle_outputs[i] + offsets_rad[i]);

		// Apply the angle limit if it is within the range (0, pi)
		if (ranges_rad[i] > 0 && ranges_rad[i] < M_PI_F) {
			//_gimbal_outputs[i] = math::constrain(_gimbal_outputs[i], -ranges_rad, ranges_rad);

			if (_gimbal_outputs[i] > ranges_rad[i]) {
				_gimbal_outputs[i] = ranges_rad[i];

			} else if (_gimbal_outputs[i] < -ranges_rad[i]) {
				_gimbal_outputs[i] = -ranges_rad[i];
			}
		}

		// Normalize the gimbal output to the range [-1, 1]
		_gimbal_outputs[i] /= M_PI_F;

		//if (q_setpoint_valid && IS_NOT_NAN(euler_gimbal(i))) {
		//	_angle_outputs[i] = euler_gimbal(i);
		//}
		//
		//if (IS_NOT_NAN(_angle_velocity[i])) {
		//	_angle_outputs[i] += dt * _angle_velocity[i];
		//}
		//
		//if (_absolute_angle[i] && IS_NOT_NAN(euler_vehicle(i))) {
		//	_angle_outputs[i] -= euler_vehicle(i);
		//}
		//
		//if (IS_NOT_NAN(_angle_outputs[i])) {
		//	// bring angles into proper range [-pi, pi]
		//	_angle_outputs[i] = matrix::wrap_pi(_angle_outputs[i]);
		//}
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

