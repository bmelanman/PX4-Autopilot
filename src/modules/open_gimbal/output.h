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
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
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
    explicit OutputBase( Parameters &parameters );
    virtual ~OutputBase() = default;

    virtual int update( const ControlData &control_data, bool new_setpoints ) = 0;

    virtual void print_status() const = 0;

    void publish();

    void set_stabilize( bool roll_stabilize, bool pitch_stabilize, bool yaw_stabilize );

    void update_params( const Parameters &parameters );

    static constexpr uint8_t _INDEX_ROLL = gimbal_controls_s::INDEX_ROLL;
    static constexpr uint8_t _INDEX_PITCH = gimbal_controls_s::INDEX_PITCH;
    static constexpr uint8_t _INDEX_YAW = gimbal_controls_s::INDEX_YAW;

   protected:
    Parameters &_parameters;

    /** set angle setpoints, speeds & stabilize flags */
    void _set_angle_setpoints( const ControlData &control_data );

    /** calculate the angle output */
    int _calculate_angle_output( const hrt_abstime &t );

    hrt_abstime _last_update_usec = 0;  ///< last update time

    void _update_debug(
        float dt, matrix::Vector3f err, matrix::Vector3f rates, matrix::Vector3f rates_sp,
        matrix::Vector3f angular_accel, matrix::Vector3f updated_rate
    );

    RateControl *_rate_controller = nullptr;         ///< rate controller for the gimbal
    matrix::Vector3f _angle_setpoint{ 0, 0, 0 };     ///< setpoint angles [rad]
    matrix::Vector3f _gimbal_output_rad{ 0, 0, 0 };  ///< calculated output angles [rad]

    float _debug_dt;  //>> units are seconds
    matrix::Vector3f _debug_sp_error{};
    matrix::Vector3f _debug_rates{};           //>> units are rad/s
    matrix::Vector3f _debug_rates_setpoint{};  //>> units are rad/s
    matrix::Vector3f _debug_angular_accel{};   //>> units are rad/s^2
    matrix::Vector3f _debug_updated_rate{};    //>> units are rad/s

   private:
    // Update rate controller gains
    void _update_rate_controller( void );
    // Publish the rate controller status
    void _publish_rate_ctrl_status( void );

    uORB::Subscription _vehicle_attitude_sub{ ORB_ID( vehicle_attitude ) };
    uORB::Subscription _vehicle_angular_velocity_sub{ ORB_ID( vehicle_angular_velocity ) };
    uORB::Subscription _vehicle_land_detected_sub{ ORB_ID( vehicle_land_detected ) };

    uORB::Publication<mount_orientation_s> _mount_orientation_pub{ ORB_ID( mount_orientation ) };
    uORB::Publication<rate_ctrl_status_s> _rate_ctrl_status_pub{ ORB_ID( rate_ctrl_status ) };

    bool _landed{ true };
};

}  // namespace open_gimbal
