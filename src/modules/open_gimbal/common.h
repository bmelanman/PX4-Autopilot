/****************************************************************************
*
*   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

#include <stdint.h>

#include <matrix/math.hpp>

namespace open_gimbal
{

/**
 * @struct ControlData
 * This defines the common API between an input and an output of the gimbal driver.
 * Each output must support the (full) set of the commands, and an input can create all
 * or a subset of the types.
 */
struct ControlData {

	//enum class Type {
	//	Neutral = 0,
	//	Angle,
	//	LonLat
	//};

	matrix::Eulerf euler_angle;
	//union TypeData {
	//	struct TypeAngle {
	//		float q[4];
	//		float angular_velocity[3];

	//		enum class Frame : uint8_t {
	//			AngleBodyFrame = 0, // Also called follow mode, angle relative to vehicle forward (usually default for yaw axis).
	//			AngularRate = 1, // Angular rate set only.
	//			AngleAbsoluteFrame = 2 // Also called lock mode, angle relative to horizon/world, lock mode. (usually default for roll and pitch).
	//		} frames[3];
	//	} angle;

	//	// TODO: Remove?
	//	struct TypeLonLat {
	//		double lon; // longitude in deg
	//		double lat; // latitude in deg
	//		float altitude; // altitude in m
	//		float roll_offset; // roll offset in rad
	//		float pitch_offset; // pitch offset in rad
	//		float yaw_offset;  // yaw offset in rad
	//		float pitch_fixed_angle; // ignored if < -pi, otherwise use a fixed pitch angle instead of the altitude
	//	} lonlat;
	//} type_data;

	//Type type = Type::Neutral;

	//matrix::Eulerf euler_zero_setpoint{ 0.0f, 0.0f, 0.0f };

	// TODO: Remove?
	//uint8_t device_compid = 0;
	//uint8_t device_sysid = 0;
};


} /* namespace open_gimbal */
