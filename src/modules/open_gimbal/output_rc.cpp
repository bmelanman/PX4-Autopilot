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

#include <px4_platform_common/defines.h>
#include <uORB/topics/gimbal_controls.h>
#include <uORB/topics/vehicle_angular_velocity.h>

#include <matrix/matrix/math.hpp>

namespace open_gimbal {

OutputRC::OutputRC( Parameters &parameters ) : OutputBase( parameters )
{
    // Set the euler setpoint to zero
    // _euler_setpoint.zero();
    _angle_setpoint.zero();
    _gimbal_output_rad.zero();
    // memset( _gimbal_output_rad, 0, sizeof( _gimbal_output_rad ) );

    // Update the output to move the motors to a known position (0, 0, 0)
    _publish_gimbal_output_rc( hrt_absolute_time() );
}

int OutputRC::_publish_gimbal_output_rc( const hrt_abstime &t_usec )
{
    // Publish the gimbal control outputs
    gimbal_controls_s _gimbal_controls{};

    _gimbal_controls.timestamp = t_usec;

    // Normalize the output to the range [-1, 1] and send it to the motors
    _gimbal_controls.control[OutputBase::_INDEX_ROLL] = _gimbal_output_rad( OutputBase::_INDEX_ROLL ) / M_PI_F;
    _gimbal_controls.control[OutputBase::_INDEX_PITCH] = _gimbal_output_rad( OutputBase::_INDEX_PITCH ) / M_PI_F;
    _gimbal_controls.control[OutputBase::_INDEX_YAW] = _gimbal_output_rad( OutputBase::_INDEX_YAW ) / M_PI_F;

    // Publish the gimbal control outputs
    if ( !_gimbal_controls_pub.publish( _gimbal_controls ) )
    {
        return PX4_ERROR;
    }

    _last_update_usec = t_usec;

    return PX4_OK;
}

int OutputRC::update( const ControlData &control_data, bool new_setpoints )
{
    // Update if we have new setpoints
    if ( new_setpoints )
    {
        _set_angle_setpoints( control_data );
    }

    hrt_abstime t = hrt_absolute_time();

    // Calculate the angle outputs
    if ( _calculate_angle_output( t ) == PX4_ERROR )
    {
        return PX4_ERROR;
    }

    // Publish the angle outputs
    _stream_device_attitude_status();

    return _publish_gimbal_output_rc( t );
}

void OutputRC::print_status() const
{
    static uORB::Subscription _status_vehicle_attitude_sub{ ORB_ID( vehicle_attitude ) };
    static vehicle_attitude_s _stat_vehicle_att{ 0 };

    static uORB::Subscription _status_vehicle_angular_velocity_sub{ ORB_ID( vehicle_angular_velocity ) };
    static vehicle_angular_velocity_s _stat_vehicle_angular_vel{ 0 };

    // Get the current vehicle attitude
    if ( !_status_vehicle_attitude_sub.copy( &_stat_vehicle_att ) )
    {
        memset( &_stat_vehicle_att, 0, sizeof( _stat_vehicle_att ) );
    }

    if ( !_status_vehicle_angular_velocity_sub.copy( &_stat_vehicle_angular_vel ) )
    {
        memset( &_stat_vehicle_angular_vel, 0, sizeof( _stat_vehicle_angular_vel ) );
    }

    /* Vehicle Attitude */
    static matrix::Vector3f veh_att{ 0, 0, 0 };
    static matrix::Vector3f att_with_off{ 0, 0, 0 };

    veh_att = matrix::Eulerf( matrix::Quatf{ _stat_vehicle_att.q } );
    att_with_off = matrix::Vector3f(
        matrix::wrap_pi( veh_att( 0 ) + math::radians( _parameters.og_off_roll ) ),
        matrix::wrap_pi( veh_att( 1 ) + math::radians( _parameters.og_off_pitch ) ),
        matrix::wrap_pi( veh_att( 2 ) + math::radians( _parameters.og_off_yaw ) )
    );

    /* Vehicle Angular Velocity */
    static matrix::Vector3f veh_angular_vel{ 0, 0, 0 };
    static matrix::Vector3f veh_angular_accel{ 0, 0, 0 };

    veh_angular_vel = matrix::Vector3f( _stat_vehicle_angular_vel.xyz );
    veh_angular_accel = matrix::Vector3f( _stat_vehicle_angular_vel.xyz_derivative );

    PX4_INFO_RAW( "Output: AUX\n" );

    PX4_INFO_RAW( "/* Vehicle Attitude ***************************/\n" );
    PX4_INFO_RAW( "\tRoll:  %8.4f\n", (double)( veh_att( 0 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tPitch: %8.4f\n", (double)( veh_att( 1 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tYaw:   %8.4f\n", (double)( veh_att( 2 ) * M_RAD_TO_DEG_F ) );

    PX4_INFO_RAW( "/* Vehicle Attitude with Motor Offset *********/\n" );
    PX4_INFO_RAW( "\tRoll:  %8.4f\n", (double)( att_with_off( 0 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tPitch: %8.4f\n", (double)( att_with_off( 1 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tYaw:   %8.4f\n", (double)( att_with_off( 2 ) * M_RAD_TO_DEG_F ) );

    PX4_INFO_RAW( "/* Vehicle Angular Velocity *******************/\n" );
    PX4_INFO_RAW( "\tRoll:  %8.4f\n", (double)( veh_angular_vel( 0 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tPitch: %8.4f\n", (double)( veh_angular_vel( 1 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tYaw:   %8.4f\n", (double)( veh_angular_vel( 2 ) * M_RAD_TO_DEG_F ) );

    PX4_INFO_RAW( "/* Vehicle Angular Acceleration ***************/\n" );
    PX4_INFO_RAW( "\tRoll:  %8.4f\n", (double)( veh_angular_accel( 0 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tPitch: %8.4f\n", (double)( veh_angular_accel( 1 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tYaw:   %8.4f\n", (double)( veh_angular_accel( 2 ) * M_RAD_TO_DEG_F ) );

    PX4_INFO_RAW( "/* Target Setpoint ****************************/\n" );
    PX4_INFO_RAW( "\tRoll:  %8.4f\n", (double)( _angle_setpoint( 0 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tPitch: %8.4f\n", (double)( _angle_setpoint( 1 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tYaw:   %8.4f\n", (double)( _angle_setpoint( 2 ) * M_RAD_TO_DEG_F ) );

    // clang-format off
    PX4_INFO_RAW( "/* Rate Controller Output Rates ***************/\n" );
    PX4_INFO_RAW( "\tdt: %8.4f [ms]\n", (double)_debug_dt * MSEC_PER_SEC );
    PX4_INFO_RAW( "\tsp_error:       { %8.4f, %8.4f, %8.4f } [rad]\n", (double)_debug_sp_error(1),(double)_debug_sp_error(1),(double)_debug_sp_error(3));
    PX4_INFO_RAW( "\trates:          { %8.4f, %8.4f, %8.4f } [rad/s]\n", (double)_debug_rates(1),(double)_debug_rates(1),(double)_debug_rates(3));
    PX4_INFO_RAW( "\trates_setpoint: { %8.4f, %8.4f, %8.4f } [rad/s]\n", (double)_debug_rates_setpoint(1),(double)_debug_rates_setpoint(1),(double)_debug_rates_setpoint(3));
    PX4_INFO_RAW( "\tangular_accel:  { %8.4f, %8.4f, %8.4f } [rad/s]\n", (double)_debug_angular_accel(1),(double)_debug_angular_accel(1),(double)_debug_angular_accel(3));
    PX4_INFO_RAW( "\tupdated_rate:   { %8.4f, %8.4f, %8.4f } [rad/s]\n", (double)_debug_updated_rate(1),(double)_debug_updated_rate(1),(double)_debug_updated_rate(3));
    // clang-format on

    PX4_INFO_RAW( "/* Output Angles ******************************/\n" );
    PX4_INFO_RAW( "\tRoll:  %8.4f\n", (double)( _gimbal_output_rad( 0 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tPitch: %8.4f\n", (double)( _gimbal_output_rad( 1 ) * M_RAD_TO_DEG_F ) );
    PX4_INFO_RAW( "\tYaw:   %8.4f\n", (double)( _gimbal_output_rad( 2 ) * M_RAD_TO_DEG_F ) );

    PX4_INFO_RAW( "/**********************************************/\n" );
}

void OutputRC::_stream_device_attitude_status()
{
    // Publish the attitude status
    gimbal_device_attitude_status_s _gimbal_attitude_status{};

    _gimbal_attitude_status.timestamp = hrt_absolute_time();
    _gimbal_attitude_status.target_system = 0;
    _gimbal_attitude_status.target_component = 0;
    _gimbal_attitude_status.device_flags = gimbal_device_attitude_status_s::DEVICE_FLAGS_NEUTRAL |
                                           gimbal_device_attitude_status_s::DEVICE_FLAGS_ROLL_LOCK |
                                           gimbal_device_attitude_status_s::DEVICE_FLAGS_PITCH_LOCK |
                                           gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK;

    matrix::Quatf q( matrix::Eulerf(
        _gimbal_output_rad( OutputBase::_INDEX_ROLL ),   //
        _gimbal_output_rad( OutputBase::_INDEX_PITCH ),  //
        _gimbal_output_rad( OutputBase::_INDEX_YAW )     //
    ) );

    _gimbal_attitude_status.q[0] = q( 0 );
    _gimbal_attitude_status.q[1] = q( 1 );
    _gimbal_attitude_status.q[2] = q( 2 );
    _gimbal_attitude_status.q[3] = q( 3 );

    _gimbal_attitude_status.failure_flags = 0;
    _attitude_status_pub.publish( _gimbal_attitude_status );
}

} /* namespace open_gimbal */
