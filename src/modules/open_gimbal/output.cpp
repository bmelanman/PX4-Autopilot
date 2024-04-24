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

#include <px4_platform_common/defines.h>

namespace open_gimbal
{

OutputBase::OutputBase(const Parameters &parameters)
	: _parameters(parameters)
{
	_last_update = hrt_absolute_time();
	_rate_controller = new RateControl();
}

//OutputBase::~OutputBase()
//{
//	_vehicle_attitude_sub.unsubscribe();
//	_vehicle_global_position_sub.unsubscribe();
//	_vehicle_land_detected_sub.unsubscribe();
//}

matrix::Quatf OutputBase::_get_q_setpoint()
{
	// Debug: For printing the quaternion setpoint
	return _q_setpoint;
}

void OutputBase::publish()
{
	//mount_orientation_s mount_orientation{};

	//mount_orientation.attitude_euler_angle[OutputBase::_INDEX_ROLL] = _angle_outputs_deg[OutputBase::_INDEX_ROLL];
	//mount_orientation.attitude_euler_angle[OutputBase::_INDEX_PITCH] = _angle_outputs_deg[OutputBase::_INDEX_PITCH];
	//mount_orientation.attitude_euler_angle[OutputBase::_INDEX_YAW] = _angle_outputs_deg[OutputBase::_INDEX_YAW];

	//mount_orientation.timestamp = hrt_absolute_time();

	//_mount_orientation_pub.publish(mount_orientation);
}

void OutputBase::_set_angle_setpoints(const ControlData &control_data)
{
	// Only support angle setpoints
	if (control_data.type != ControlData::Type::Angle) {
		PX4_ERR("_set_angle_setpoints called with invalid control_data type: %d", (int)control_data.type);
		return;
	}

	// Set the absolute angle flags based on the control data
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

	// Update the setpoint
	_q_setpoint = matrix::Quatf(control_data.type_data.angle.q);
}

#define ANG_ACC_NUL (matrix::Vector3f(0.0f, 0.0f, 0.0f))

int OutputBase::_calculate_angle_output(const hrt_abstime &t, matrix::Quatf q_zero_setpoint)
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
	if (!_vehicle_attitude_sub.copy(&vehicle_attitude) || !(matrix::Quatf(vehicle_attitude.q)).isAllFinite()) {
		PX4_ERR("Failed to get vehicle attitude! :(");

		for (i = 0; i < 3; ++i) {
			_angle_outputs_deg[i] = 0.0f;
			_gimbal_outputs[i] = 0.0f;
		}

		return PX4_ERROR;
	}

	// Make sure the setpoints are valid
	if (!_q_setpoint.isAllFinite() || !q_zero_setpoint.isAllFinite()) {
		PX4_ERR("Invalid setpoint quaternions! :(");

		for (i = 0; i < 3; ++i) {
			_angle_outputs_deg[i] = 0.0f;
			_gimbal_outputs[i] = 0.0f;
		}

		return PX4_ERROR;
	}

	// Make sure the rate controller is initialized
	if (_rate_controller == nullptr) {

		_rate_controller = new RateControl();
		t_prev = t;

		PX4_WARN("Rate controller initialized HERE! %s:%d", __FILE__, __LINE__);
	}

	// Set the PID gains
	_rate_controller->setPidGains( // Roll, Pitch, Yaw
		matrix::Vector3f(_parameters.mnt_roll_p, _parameters.mnt_pitch_p, _parameters.mnt_yaw_p), // P Gain
		matrix::Vector3f(_parameters.mnt_roll_i, _parameters.mnt_pitch_i, _parameters.mnt_yaw_i), // I Gain
		matrix::Vector3f(_parameters.mnt_roll_d, _parameters.mnt_pitch_d, _parameters.mnt_yaw_d)  // D Gain
	);

	// Convert the vehicle attitude and gimbal setpoint quaternions to euler angles
	matrix::Eulerf att_current(matrix::Quatf(vehicle_attitude.q));
	matrix::Eulerf att_setpoint(q_zero_setpoint * _q_setpoint.inversed()); // Debug: Inverted?

	// Get the current timestamp and calculate the time difference
	hrt_abstime t_now = t;

	// TODO: Add _landed check to update
	bool _is_landed = false;

	// Pass the euler angles to the rate controller
	matrix::Vector3f gimbal_rate = _rate_controller->update(
					       att_current, att_setpoint, ANG_ACC_NUL, (t_now - t_prev), _is_landed);

	// Update the previous timestamp
	t_prev = t_now;

	// Get the current angle offsets and ranges
	//const float ranges_rad[3] = {
	//	math::radians(_parameters.mnt_range_roll),
	//	math::radians(_parameters.mnt_range_pitch),
	//	math::radians(_parameters.mnt_range_yaw)
	//};

	// Wrap the axis setpoint around [-pi, pi] and normalize it to the range [-1, 1]
	for (i = 0; i < 3; ++i) {
		gimbal_rate(i) = matrix::wrap_pi(gimbal_rate(i)) / M_PI_F;

		// Debug: Add the angle in degrees to the outputs array
		_angle_outputs_deg[i] = math::degrees(gimbal_rate(i));
	}

	// Roll and pitch can only move +/- 90 degrees from the zero setpoint
	_gimbal_outputs[_INDEX_ROLL] = math::constrain(gimbal_rate(_INDEX_ROLL), -0.5f, 0.5f);
	_gimbal_outputs[_INDEX_PITCH] = math::constrain(gimbal_rate(_INDEX_PITCH), -0.5f, 0.5f);

	// Yaw can move infinitely (+/- 180 degrees)
	_gimbal_outputs[_INDEX_YAW] = gimbal_rate(_INDEX_YAW);

	// constrain pitch to [MNT_LND_P_MIN, MNT_LND_P_MAX] if landed
	//if (_landed) {
	//	if (PX4_ISFINITE(_angle_outputs_deg[1])) {
	//		_angle_outputs_deg[1] = _translate_angle2gimbal(
	//					    _angle_outputs_deg[1],
	//					    _parameters.mnt_off_pitch,
	//					    _parameters.mnt_range_pitch);
	//	}
	//}

	return PX4_OK;
}

void OutputBase::set_stabilize(bool roll_stabilize, bool pitch_stabilize, bool yaw_stabilize)
{
	_stabilize[0] = roll_stabilize;
	_stabilize[1] = pitch_stabilize;
	_stabilize[2] = yaw_stabilize;
}

} /* namespace open_gimbal */
