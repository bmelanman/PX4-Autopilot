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

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/gimbal_controls.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <lib/rate_control/rate_control.hpp>
#include <matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include "common.h"
#include "open_gimbal_params.h"

namespace open_gimbal {

class OutputBase {
   public:
    OutputBase() = delete;
    explicit OutputBase( const Parameters &parameters );
    virtual ~OutputBase() = default;

    virtual int update( const ControlData &control_data, bool new_setpoints, bool new_params ) = 0;

    virtual void print_status() const = 0;

    void publish();

    void set_stabilize( bool roll_stabilize, bool pitch_stabilize, bool yaw_stabilize );

    static constexpr uint8_t _INDEX_ROLL = gimbal_controls_s::INDEX_ROLL;
    static constexpr uint8_t _INDEX_PITCH = gimbal_controls_s::INDEX_PITCH;
    static constexpr uint8_t _INDEX_YAW = gimbal_controls_s::INDEX_YAW;

    float _gimbal_output_rad[3] = { 0.f, 0.f, 0.f };  ///< output for debugging [rad]

   protected:
    const Parameters &_parameters;

    /** set angle setpoints, speeds & stabilize flags */
    int _set_angle_setpoints( const ControlData &control_data );

    /** calculate the angle output */
    int _calculate_angle_output( const hrt_abstime &t, bool new_params );

    hrt_abstime _last_update_usec = 0;  ///< last update time

    matrix::Eulerf _euler_setpoint{ 0, 0, 0 };         ///< setpoint angles [rad]
    float _gimbal_output_norm[3] = { 0.f, 0.f, 0.f };  ///< calculated output angles [-1, 1]

    RateControl *_rate_controller = nullptr;  ///< rate controller for the gimbal

   private:
    uORB::Subscription _vehicle_attitude_sub{ ORB_ID( vehicle_attitude ) };
    uORB::Subscription _vehicle_angular_velocity_sub{ ORB_ID( vehicle_angular_velocity ) };

    uORB::Subscription _vehicle_global_position_sub{ ORB_ID( vehicle_global_position ) };
    uORB::Subscription _vehicle_land_detected_sub{ ORB_ID( vehicle_land_detected ) };

    // Update rate controller gains
    void _update_rate_controller( void );

    /* PID parameters */
    param_t _param_pid_roll_p, _param_pid_roll_i, _param_pid_roll_d;     // Roll PID
    param_t _param_pid_pitch_p, _param_pid_pitch_i, _param_pid_pitch_d;  // Pitch PID
    param_t _param_pid_yaw_p, _param_pid_yaw_i, _param_pid_yaw_d;        // Yaw PID

    param_t _param_pid_roll_imax, _param_pid_pitch_imax, _param_pid_yaw_imax;  // PID integrator max
    param_t _param_pid_roll_ff, _param_pid_pitch_ff, _param_pid_yaw_ff;        // PID feed forward

    // uORB::Publication<mount_orientation_s> _mount_orientation_pub{ORB_ID(mount_orientation)};

    bool _landed{ true };
};

}  // namespace open_gimbal
