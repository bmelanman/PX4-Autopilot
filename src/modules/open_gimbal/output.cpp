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

// Minimum and maximum time step in seconds
#define DELTA_T_MIN 0.001f
#define DELTA_T_MAX 1.0f

// PID Controller limits
#define PID_INT_LIM 100.0f // TODO: What should this value be?
#define PID_OUT_LIM M_1_PI_F // Limited to [-pi, pi]

#define _pid_calculate(pid, sp, val, dt) pid_calculate(pid, sp, val, 0.0f, dt)

namespace open_gimbal
{

OutputBase::OutputBase(const Parameters &parameters)
	: _parameters(parameters)
{
	_last_update_usec = hrt_absolute_time();

	_rate_controller = new RateControl();

	/* PID Controller */
	_pid_controller = new PID_t();

	// 0.01f -> Minimum time step of 10ns
	// PID_MODE_DERIVATIV_CALC -> Ignores the `val_dot` param
	pid_init(_pid_controller, PID_MODE_DERIVATIV_CALC, DELTA_T_MIN);

	// Set the PID gains
	pid_set_parameters(
		_pid_controller,
		_parameters.mnt_roll_p, _parameters.mnt_roll_i, _parameters.mnt_roll_d,
		PID_INT_LIM, PID_OUT_LIM
	);
}

//OutputBase::~OutputBase()
//{
//	_vehicle_attitude_sub.unsubscribe();
//	_vehicle_global_position_sub.unsubscribe();
//	_vehicle_land_detected_sub.unsubscribe();
//}

void OutputBase::publish()
{
	//mount_orientation_s mount_orientation{};

	//mount_orientation.attitude_euler_angle[OutputBase::_INDEX_ROLL] = _gimbal_output_rad[OutputBase::_INDEX_ROLL];
	//mount_orientation.attitude_euler_angle[OutputBase::_INDEX_PITCH] = _gimbal_output_rad[OutputBase::_INDEX_PITCH];
	//mount_orientation.attitude_euler_angle[OutputBase::_INDEX_YAW] = _gimbal_output_rad[OutputBase::_INDEX_YAW];

	//mount_orientation.timestamp = hrt_absolute_time();

	//_mount_orientation_pub.publish(mount_orientation);
}

int OutputBase::_set_angle_setpoints(const ControlData &control_data)
{
	_euler_setpoint = matrix::Eulerf(control_data.euler_angle);

	return PX4_OK;
}

#define ANG_ACC_0 (matrix::Vector3f(0.0f, 0.0f, 0.0f))
#define ANG_ACC_1 (matrix::Vector3f(1.0f, 1.0f, 1.0f))

void _print_euler(const matrix::Eulerf &euler, const char *prefix = "")
{
	PX4_INFO_RAW(
		"  %18s: %8.4f, %8.4f, %8.4f\n", prefix, (double)euler(0), (double)euler(1), (double)euler(2)
	);
}

void _print_euler(const float euler[3], const char *prefix = "")
{
	_print_euler(matrix::Eulerf{euler[0], euler[1], euler[2]}, prefix);
}

int OutputBase::_calculate_angle_output(const hrt_abstime &t_usec)
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

	// Get the vehicle attitude and angular velocity
	vehicle_attitude_s vehicle_attitude;
	vehicle_angular_velocity_s vehicle_angular_velocity;

	// Validate the vehicle attitude
	if (!_vehicle_attitude_sub.copy(&vehicle_attitude)) {
		PX4_ERR("Failed to get vehicle attitude! :(");

		for (i = 0; i < 3; ++i) {
			_gimbal_output_rad[i] = 0.0f;
			_gimbal_output_norm[i] = 0.0f;
		}

		return PX4_ERROR;
	}

	// Validate the vehicle angular velocity
	if (!_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
		PX4_ERR("Failed to get vehicle angular velocity! :(");

		for (i = 0; i < 3; ++i) {
			_gimbal_output_rad[i] = 0.0f;
			_gimbal_output_norm[i] = 0.0f;
		}

		return PX4_ERROR;
	}

	// Validate the gimbal setpoints
	if (!_euler_setpoint.isAllFinite()) {
		PX4_ERR("Invalid setpoint quaternions! :(");

		for (i = 0; i < 3; ++i) {
			_gimbal_output_rad[i] = 0.0f;
			_gimbal_output_norm[i] = 0.0f;
		}

		return PX4_ERROR;
	}

	// TODO: Add _landed check
	bool _is_landed = false;

	// Get the time delta and constrain it
	float dt_usec = math::constrain((float)(t_usec - _last_update_usec) / USEC_PER_SEC, DELTA_T_MIN, DELTA_T_MAX);

	// Convert the vehicle attitude to euler angles
	matrix::Eulerf euler_vehicle{matrix::Quatf(vehicle_attitude.q)};

	// Convert the vehicle angular velocity to euler angles
	matrix::Eulerf euler_angular_velocity{
		vehicle_angular_velocity.xyz[0],
		vehicle_angular_velocity.xyz[1],
		vehicle_angular_velocity.xyz[2]
	};

	// Get the zero offsets for the motors
	const matrix::Eulerf zero_offsets{
		_parameters.mnt_motor_roll,
		_parameters.mnt_motor_pitch,
		_parameters.mnt_motor_yaw
	};

	// Get the current angle offsets and ranges
	//const matrix::Eulerf ranges_rad = {
	//	math::radians(_parameters.mnt_range_roll),
	//	math::radians(_parameters.mnt_range_pitch),
	//	math::radians(_parameters.mnt_range_yaw)
	//};

	/* RateController */
	// Set the PID gains
	_rate_controller->setPidGains( // Roll, Pitch, Yaw
		matrix::Vector3f(_parameters.mnt_roll_p, _parameters.mnt_pitch_p, _parameters.mnt_yaw_p), // P Gain
		matrix::Vector3f(_parameters.mnt_roll_i, _parameters.mnt_pitch_i, _parameters.mnt_yaw_i), // I Gain
		matrix::Vector3f(_parameters.mnt_roll_d, _parameters.mnt_pitch_d, _parameters.mnt_yaw_d)  // D Gain
	);

	// Pass the euler angles to the rate controller (returns the gimbal rates in radians)
	matrix::Eulerf updated_rate = _rate_controller->update(
					      euler_vehicle, _euler_setpoint, euler_angular_velocity,
					      dt_usec, _is_landed) * M_1_PI_F;

	// Calculate the PID output
	//float pid[3] = {0.0f, 0.0f, 0.0f};

	// Wrap the output and normalize it
	for (i = 0; i < 3; ++i) {

		// Start with the setpoint
		_gimbal_output_rad[i] = _euler_setpoint(i);
		//_gimbal_output_rad[i] = euler_vehicle(i);

		// Add the rate output
		_gimbal_output_rad[i] += (float)((double)dt_usec / USEC_PER_SEC) * euler_angular_velocity(i);
		//pid[i] = _pid_calculate(_pid_controller, _euler_setpoint(i), euler_vehicle(i), dt_usec);
		//_gimbal_output_rad[i] += pid[i];
		//_gimbal_output_rad[i] += updated_rate(i);

		// Account for the vehicle attitude
		_gimbal_output_rad[i] -= euler_vehicle(i);

		// Account for the motor zero poiont offset
		//_gimbal_output_rad[i] -= zero_offsets(i);

		// Make sure the output is within the range [-pi, pi]
		_gimbal_output_rad[i] = matrix::wrap_pi(_gimbal_output_rad[i]);

		// Constrain the output to the (optionally) given range
		//_gimbal_output_rad[i] = math::constrain(_gimbal_output_rad[i], -ranges_rad(i), ranges_rad(i));

		// Normalize the output to [-1, 1]
		_gimbal_output_norm[i] = _gimbal_output_rad[i] / M_PI_F;
	}

	static uint32_t counter = 0;

	if (_parameters.og_debug > 0 && counter++ % _parameters.og_debug == 0) {

		PX4_INFO("Gimbal outputs:");
		_print_euler(euler_vehicle, "euler_vehicle");
		_print_euler(euler_angular_velocity, "euler_angular_velocity");
		_print_euler(_euler_setpoint, "euler_setpoint");
		_print_euler(updated_rate, "updated_rate");
		//_print_euler(pid, "pid");
		_print_euler(_gimbal_output_rad, "gimbal_output_rad");
		_print_euler(_gimbal_output_norm, "gimbal_output_norm");
		PX4_INFO_RAW("  %18s: %8.4f (usec)\n\n", "dt_usec", (double)dt_usec);
	}

	// Roll and pitch can only move +/- 90 degrees from the zero setpoint
	//_gimbal_output_norm[_INDEX_ROLL] = math::constrain(updated_rate(_INDEX_ROLL), -0.5f, 0.5f);
	//_gimbal_output_norm[_INDEX_PITCH] = math::constrain(updated_rate(_INDEX_PITCH), -0.5f, 0.5f);

	//_gimbal_output_norm[_INDEX_ROLL] = updated_rate(_INDEX_ROLL);
	//_gimbal_output_norm[_INDEX_PITCH] = updated_rate(_INDEX_PITCH);

	//// Yaw can move infinitely (+/- 180 degrees)
	//_gimbal_output_norm[_INDEX_YAW] = updated_rate(_INDEX_YAW);

	// constrain pitch to [MNT_LND_P_MIN, MNT_LND_P_MAX] if landed
	//if (_landed) {
	//	if (PX4_ISFINITE(_gimbal_output_rad[1])) {
	//		_gimbal_output_rad[1] = _translate_angle2gimbal(
	//					    _gimbal_output_rad[1],
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
