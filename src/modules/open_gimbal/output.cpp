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

// Scale factor for the rate controller output
#define RATE_SCALE_FACTOR ( 2U )

namespace open_gimbal {

OutputBase::OutputBase( const Parameters &parameters ) : _parameters( parameters )
{
    _last_update_usec = hrt_absolute_time();

    // Initialize the rate controller
    _rate_controller = new RateControl();

    // Initialize the PID params
    _param_pid_roll_p = param_find( "OG_ROLL_P" );
    _param_pid_roll_i = param_find( "OG_ROLL_I" );
    _param_pid_roll_d = param_find( "OG_ROLL_D" );
    _param_pid_roll_imax = param_find( "OG_ROLL_IMAX" );
    _param_pid_roll_ff = param_find( "OG_ROLL_FF" );

    _param_pid_pitch_p = param_find( "OG_PITCH_P" );
    _param_pid_pitch_i = param_find( "OG_PITCH_I" );
    _param_pid_pitch_d = param_find( "OG_PITCH_D" );
    _param_pid_pitch_imax = param_find( "OG_PITCH_IMAX" );
    _param_pid_pitch_ff = param_find( "OG_PITCH_FF" );

    _param_pid_yaw_p = param_find( "OG_YAW_P" );
    _param_pid_yaw_i = param_find( "OG_YAW_I" );
    _param_pid_yaw_d = param_find( "OG_YAW_D" );
    _param_pid_yaw_imax = param_find( "OG_YAW_IMAX" );
    _param_pid_yaw_ff = param_find( "OG_YAW_FF" );
}

void OutputBase::publish()
{
    // TODO: What is this for?
    // mount_orientation_s mount_orientation{};

    // mount_orientation.attitude_euler_angle[OutputBase::_INDEX_ROLL] = _gimbal_output_rad[OutputBase::_INDEX_ROLL];
    // mount_orientation.attitude_euler_angle[OutputBase::_INDEX_PITCH] = _gimbal_output_rad[OutputBase::_INDEX_PITCH];
    // mount_orientation.attitude_euler_angle[OutputBase::_INDEX_YAW] = _gimbal_output_rad[OutputBase::_INDEX_YAW];

    // mount_orientation.timestamp = hrt_absolute_time();

    //_mount_orientation_pub.publish(mount_orientation);
}

int OutputBase::_set_angle_setpoints( const ControlData &control_data )
{
    _euler_setpoint = matrix::Eulerf( control_data.euler_angle );

    return PX4_OK;
}

void _print_euler( const matrix::Eulerf &euler, const char *prefix = "" )
{
    PX4_INFO_RAW(
        "  %18.18s: %8.4f, %8.4f, %8.4f\n", prefix, (double)euler( 0 ), (double)euler( 1 ), (double)euler( 2 )
    );
}

void _print_euler( const float euler[3], const char *prefix = "" )
{
    _print_euler( matrix::Eulerf{ euler[0], euler[1], euler[2] }, prefix );
}

void OutputBase::_update_rate_controller( void )
{
    static matrix::Vector3f gains_p{ 0 }, gains_i{ 0 }, gains_d{ 0 }, int_lim{ 0 }, gains_ff{ 0 };

    /* PID Gains */
    param_get( _param_pid_roll_p, &gains_p( 0 ) );
    param_get( _param_pid_roll_i, &gains_i( 0 ) );
    param_get( _param_pid_roll_d, &gains_d( 0 ) );

    param_get( _param_pid_pitch_p, &gains_p( 1 ) );
    param_get( _param_pid_pitch_i, &gains_i( 1 ) );
    param_get( _param_pid_pitch_d, &gains_d( 1 ) );

    param_get( _param_pid_yaw_p, &gains_p( 2 ) );
    param_get( _param_pid_yaw_i, &gains_i( 2 ) );
    param_get( _param_pid_yaw_d, &gains_d( 2 ) );

    _rate_controller->setPidGains( gains_p, gains_i, gains_d );

    /* Integral Limits */
    param_get( _param_pid_roll_imax, &int_lim( 0 ) );
    param_get( _param_pid_pitch_imax, &int_lim( 1 ) );
    param_get( _param_pid_yaw_imax, &int_lim( 2 ) );

    _rate_controller->setIntegratorLimit( int_lim );

    /* Feed Forward Gains */
    param_get( _param_pid_roll_ff, &gains_ff( 0 ) );
    param_get( _param_pid_pitch_ff, &gains_ff( 1 ) );
    param_get( _param_pid_yaw_ff, &gains_ff( 2 ) );

    _rate_controller->setFeedForwardGain( gains_ff );
}

int OutputBase::_calculate_angle_output( const hrt_abstime &t_usec, bool new_params )
{
    // Check if the vehicle is currently landed
    // if (_vehicle_land_detected_sub.updated()) {
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
    if ( !_vehicle_attitude_sub.copy( &vehicle_attitude ) )
    {
        PX4_ERR( "Failed to get vehicle attitude! :(" );

        for ( i = 0; i < 3; ++i )
        {
            _gimbal_output_rad[i] = 0.0f;
            _gimbal_output_norm[i] = 0.0f;
        }

        return PX4_ERROR;
    }

    // Validate the vehicle angular velocity
    if ( !_vehicle_angular_velocity_sub.copy( &vehicle_angular_velocity ) )
    {
        PX4_ERR( "Failed to get vehicle angular velocity! :(" );

        for ( i = 0; i < 3; ++i )
        {
            _gimbal_output_rad[i] = 0.0f;
            _gimbal_output_norm[i] = 0.0f;
        }

        return PX4_ERROR;
    }

    // Validate the gimbal setpoints
    if ( !_euler_setpoint.isAllFinite() )
    {
        PX4_ERR( "Invalid setpoint quaternions! :(" );

        for ( i = 0; i < 3; ++i )
        {
            _gimbal_output_rad[i] = 0.0f;
            _gimbal_output_norm[i] = 0.0f;
        }

        return PX4_ERROR;
    }

    // TODO: Add _landed check
    bool _is_landed = false;

    // Get the time delta and constrain it
    float dt_usec = math::constrain( (float)( t_usec - _last_update_usec ) / USEC_PER_SEC, DELTA_T_MIN, DELTA_T_MAX );

    // Convert the vehicle attitude to euler angles
    matrix::Eulerf euler_vehicle{ matrix::Quatf( vehicle_attitude.q ) };

    // Convert the vehicle angular velocity to euler angles
    matrix::Eulerf euler_angular_accel{
        vehicle_angular_velocity.xyz_derivative[0], vehicle_angular_velocity.xyz_derivative[1],
        vehicle_angular_velocity.xyz_derivative[2]
    };

    // Get the zero offsets for the motors
    const matrix::Eulerf zero_offsets{
        _parameters.og_motor_roll, _parameters.og_motor_pitch, _parameters.og_motor_yaw
    };

    // TODO: Constrain the output to the given range params
    // Get the current angle offsets and ranges
    // const matrix::Eulerf ranges_rad = {
    //	math::radians(_parameters.og_range_roll),
    //	math::radians(_parameters.og_range_pitch),
    //	math::radians(_parameters.og_range_yaw)
    //};

    // Update the rate controller when necessary
    if ( new_params )
    {
        _update_rate_controller();
        PX4_INFO( "Rate controller updated" );
    }

    // Run the rate controller
    matrix::Eulerf updated_rate =
        _rate_controller->update(
            euler_vehicle - zero_offsets, _euler_setpoint, euler_angular_accel, dt_usec, _is_landed
        ) *
        M_2_PI_F * RATE_SCALE_FACTOR;

    for ( i = 0; i < 3; ++i )
    {
        // Make sure the output is within the range [-pi, pi]
        _gimbal_output_rad[i] = matrix::wrap_pi( updated_rate( i ) );

        // Normalize the output to [-1, 1]
        _gimbal_output_norm[i] = _gimbal_output_rad[i] / M_PI_F;
    }

    // static uint32_t counter = 0;
    // if (_parameters.og_debug1 > 0 && (counter++ % (uint32_t)(_parameters.og_debug1 * 100)) == 0) {
    //	PX4_INFO("Gimbal outputs:");
    //	_print_euler(euler_vehicle, "euler_vehicle");
    //	_print_euler(euler_angular_accel, "euler_angular_accel");
    //	_print_euler(updated_rate, "updated_rate");
    //	_print_euler(_gimbal_output_rad, "gimbal_output_rad");
    //	PX4_INFO_RAW("  %18s: %8.4f (usec)\n\n", "dt_usec", (double)dt_usec);
    // }

    // constrain pitch to [OG_LND_P_MIN, OG_LND_P_MAX] if landed
    // if (_landed) {
    //	if (PX4_ISFINITE(_gimbal_output_rad[1])) {
    //		_gimbal_output_rad[1] = _translate_angle2gimbal(
    //					    _gimbal_output_rad[1],
    //					    _parameters.og_off_pitch,
    //					    _parameters.og_range_pitch);
    //	}
    //}

    return PX4_OK;
}

void OutputBase::set_stabilize( bool roll_stabilize, bool pitch_stabilize, bool yaw_stabilize )
{
    (void)roll_stabilize;
    (void)pitch_stabilize;
    (void)yaw_stabilize;
}

} /* namespace open_gimbal */
