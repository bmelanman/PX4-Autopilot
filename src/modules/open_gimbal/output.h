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

#pragma once

#include "common.h"
#include "gimbal_params.h"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/rate_control/rate_control.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/gimbal_controls.h>

namespace open_gimbal
{

class OutputBase
{
public:

	OutputBase() = delete;
	explicit OutputBase(const Parameters &parameters);
	virtual ~OutputBase() = default;

	virtual int update(const ControlData &control_data, bool new_setpoints) = 0;

	virtual void print_status() const = 0;

	void publish();

	void set_stabilize(bool roll_stabilize, bool pitch_stabilize, bool yaw_stabilize);
	matrix::Quatf _get_q_setpoint();

	static constexpr uint8_t _INDEX_ROLL = gimbal_controls_s::INDEX_ROLL;
	static constexpr uint8_t _INDEX_PITCH = gimbal_controls_s::INDEX_PITCH;
	static constexpr uint8_t _INDEX_YAW = gimbal_controls_s::INDEX_YAW;

	float _angle_outputs_deg[3] = { 0.f, 0.f, 0.f }; ///< calculated output angles (roll, pitch, yaw) [deg]
protected:
	MapProjection _projection_reference{}; ///< class to convert (lon, lat) to local [m]

	const Parameters &_parameters;

	/** set angle setpoints, speeds & stabilize flags */
	void _set_angle_setpoints(const ControlData &control_data);

	matrix::Quatf _q_setpoint = matrix::Quatf(NAN, NAN, NAN, NAN);   ///< can be NAN if not specifically set

	RateControl *rate_controller = nullptr; ///< rate controller for the gimbal
	hrt_abstime t_prev = 0;

	bool _stabilize[3] = { false, false, false };

	// Pitch and role are by default aligned with the horizon.
	// Yaw follows the vehicle (not lock/absolute mode).
	bool _absolute_angle[3] = {true, true, false };

	/** calculate the _angle_outputs_deg (with speed) and stabilize if needed */
	int _calculate_angle_output(const hrt_abstime &t);

	float _gimbal_outputs[3] = { 0.f, 0.f, 0.f }; ///< calculated output angles (roll, pitch, yaw) [-1, 1]

	hrt_abstime _last_update;

private:
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::Publication<mount_orientation_s> _mount_orientation_pub{ORB_ID(mount_orientation)};

	//float _translate_angle2gimbal(float target_angle, float angle_offset, float angle_limit);

	bool _landed{true};
};


} /* namespace gimbal */
