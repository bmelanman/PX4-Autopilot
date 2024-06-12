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

// 180 degrees as a float
#define M_180_F 180.0f

// Minimum and maximum time step in seconds
#define DELTA_T_MIN 0.000125f
#define DELTA_T_MAX 0.02f

// Maximum output angular velocity in rad/s
#define ANG_VEL_LIMIT 20.0f

// Scale factor for the rate controller output
#define RATE_SCALE_FACTOR ( 2U )

namespace open_gimbal {

OutputBase::OutputBase( Parameters &parameters ) : _parameters( parameters )
{
    _last_update_usec = hrt_absolute_time();

    // Initialize the rate controller
    _rate_controller = new RateControl();
}

void OutputBase::publish()
{
    mount_orientation_s mount_orientation{};

    mount_orientation.attitude_euler_angle[OutputBase::_INDEX_ROLL] = _gimbal_output_rad( OutputBase::_INDEX_ROLL );
    mount_orientation.attitude_euler_angle[OutputBase::_INDEX_PITCH] = _gimbal_output_rad( OutputBase::_INDEX_PITCH );
    mount_orientation.attitude_euler_angle[OutputBase::_INDEX_YAW] = _gimbal_output_rad( OutputBase::_INDEX_YAW );

    mount_orientation.timestamp = hrt_absolute_time();

    _mount_orientation_pub.publish( mount_orientation );
}

void OutputBase::_set_angle_setpoints( const ControlData &control_data )
{
    // _euler_setpoint = matrix::Eulerf( control_data.euler_angle );
    _angle_setpoint( 0 ) = control_data.input_angle_rad( 0 );
    _angle_setpoint( 1 ) = control_data.input_angle_rad( 1 );
    _angle_setpoint( 2 ) = control_data.input_angle_rad( 2 );
}

void OutputBase::_update_rate_controller( void )
{
    static matrix::Vector3f gains_p{ 0 }, gains_i{ 0 }, gains_d{ 0 }, int_lim{ 0 }, gains_ff{ 0 };

    /* PID Gains */
    gains_p( 0 ) = _parameters.og_roll_p;
    gains_i( 0 ) = _parameters.og_roll_i;
    gains_d( 0 ) = _parameters.og_roll_d;

    gains_p( 1 ) = _parameters.og_pitch_p;
    gains_i( 1 ) = _parameters.og_pitch_i;
    gains_d( 1 ) = _parameters.og_pitch_d;

    gains_p( 2 ) = _parameters.og_yaw_p;
    gains_i( 2 ) = _parameters.og_yaw_i;
    gains_d( 2 ) = _parameters.og_yaw_d;

    _rate_controller->setPidGains( gains_p, gains_i, gains_d );

    /* Integral Limits */
    int_lim( 0 ) = _parameters.og_roll_imax;
    int_lim( 1 ) = _parameters.og_pitch_imax;
    int_lim( 2 ) = _parameters.og_yaw_imax;

    _rate_controller->setIntegratorLimit( int_lim );

    /* Feed Forward Gains */
    gains_ff( 0 ) = _parameters.og_roll_ff;
    gains_ff( 1 ) = _parameters.og_pitch_ff;
    gains_ff( 2 ) = _parameters.og_yaw_ff;

    _rate_controller->setFeedForwardGain( gains_ff );

    PX4_INFO( "Rate controller gains updated" );
}

void OutputBase::_publish_rate_ctrl_status( void )
{
    static rate_ctrl_status_s rate_ctrl_status;

    _rate_controller->getRateControlStatus( rate_ctrl_status );
    rate_ctrl_status.timestamp = hrt_absolute_time();

    _rate_ctrl_status_pub.publish( rate_ctrl_status );
}

void OutputBase::_update_debug(
    float dt, matrix::Vector3f err, matrix::Vector3f rates, matrix::Vector3f rates_sp, matrix::Vector3f angular_accel,
    matrix::Vector3f updated_rate
)
{
    _debug_dt = dt;
    _debug_sp_error = err;
    _debug_rates = rates;
    _debug_rates_setpoint = rates_sp;
    _debug_angular_accel = angular_accel;
    _debug_updated_rate = updated_rate;
}

int OutputBase::_calculate_angle_output( const hrt_abstime &t_usec )
{
    // TODO: Add _landed check
    bool _is_landed = false;

    vehicle_attitude_s _vehicle_att_data{ 0 };
    vehicle_angular_velocity_s _vehicle_angular_vel_data{ 0 };

    // Check if the vehicle is currently landed
    // if (_vehicle_land_detected_sub.updated()) {
    //	vehicle_land_detected_s vehicle_land_detected;
    //
    //	if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
    //		_landed = vehicle_land_detected.landed || vehicle_land_detected.maybe_landed;
    //	}
    //}

    // Get the vehicle attitude
    if ( !_vehicle_attitude_sub.copy( &_vehicle_att_data ) )
    {
        memset( &_vehicle_att_data, 0, sizeof( _vehicle_att_data ) );
    }

    // Get the vehicle angular velocity
    if ( !_vehicle_angular_velocity_sub.copy( &_vehicle_angular_vel_data ) )
    {
        memset( &_vehicle_angular_vel_data, 0, sizeof( _vehicle_angular_vel_data ) );
    }

    static unsigned int i;

    // Get the time delta and constrain it
    const float dt = math::constrain( (float)( t_usec - _last_update_usec ) / USEC_PER_SEC, DELTA_T_MIN, DELTA_T_MAX );

    matrix::Quatf curr_veh_att_q{ _vehicle_att_data.q };
    matrix::Vector3f rates_rad_s{ _vehicle_angular_vel_data.xyz };                  //>> units are rad/s
    matrix::Vector3f ang_accel_rad_s2{ _vehicle_angular_vel_data.xyz_derivative };  //>> units are rad/s^2
    matrix::Vector3f rates_sp_rad_s{};                                              //>> units are rad/s
    matrix::Vector3f sp_error{};

    matrix::Vector3f ranges_rad{
        math::radians( _parameters.og_range_roll ),   //
        math::radians( _parameters.og_range_pitch ),  //
        math::radians( _parameters.og_range_yaw )     //
    };

    matrix::Vector3f zero_offsets_rad{
        math::radians( _parameters.og_off_roll ),   //
        math::radians( _parameters.og_off_pitch ),  //
        math::radians( _parameters.og_off_yaw )     //
    };

    // Calculate the necessary angular velocity to reach the setpoint within `dt` seconds
    for ( i = 0; i < 3; ++i )
    {
        if ( PX4_ISFINITE( _angle_setpoint( i ) ) )
        {
            // Find the difference between where we are and were we want to be
            sp_error( i ) = ( ( _angle_setpoint( i ) - zero_offsets_rad( i ) ) - _gimbal_output_rad( i ) );

            // Convert the setpoint error to rad/s and apply a speed limit
            rates_sp_rad_s( i ) = math::constrain( sp_error( i ) / dt, -ANG_VEL_LIMIT, ANG_VEL_LIMIT );
        }
        else
        {
            // DEBUG: Print a message if the setpoint is not finite
            if ( _parameters.og_debug2 > 0.0f )
            {
                PX4_ERR(
                    "Setpoint %d is not finite: { %8.4f, %8.4f, %8.4f }", i, (double)_angle_setpoint( 0 ),
                    (double)_angle_setpoint( 1 ), (double)_angle_setpoint( 2 )
                );
            }

            // If the setpoint is not finite, set the setpoint to the current angle
            rates_sp_rad_s( i ) = rates_rad_s( i );
        }
    }

    // Run the rate controller and convert the output from rad/s to rad
    matrix::Vector3f updated_rate_rad =  //>> units are ( rad/s * s ) = rad
        _rate_controller->update( rates_rad_s, rates_sp_rad_s, ang_accel_rad_s2, dt, _is_landed ) * dt;

    // Publish rate controller status
    _publish_rate_ctrl_status();

    // Debug: Update the debug variables
    _update_debug( dt, sp_error, rates_rad_s, rates_sp_rad_s, ang_accel_rad_s2, updated_rate_rad );

    for ( i = 0; i < 3; ++i )
    {
        // Keep the output within [-pi, pi]
        _gimbal_output_rad( i ) = matrix::wrap_pi( _gimbal_output_rad( i ) + updated_rate_rad( i ) );

        // Limit the range of each axis (only if the range is positive and non-zero)
        // if ( ranges_rad( i ) > 0.0f && ranges_rad( i ) < M_PI_F )
        //{
        //    _gimbal_output_rad(i) = math::constrain( _gimbal_output_rad(i), -ranges_rad( i ), ranges_rad( i ) );
        //}

        // Account for the zero offsets
        //_gimbal_output_rad( i ) -= zero_offsets_rad( i );
    }

    // constrain pitch to [OG_LND_P_MIN, OG_LND_P_MAX] if landed
    // if (_landed) {
    //	if (PX4_ISFINITE(_gimbal_output_rad(OutputBase::_INDEX_PITCH))) {
    //		_gimbal_output_rad(OutputBase::_INDEX_PITCH) = _translate_angle2gimbal(
    //					    _gimbal_output_rad(OutputBase::_INDEX_PITCH),
    //					    _parameters.og_off_pitch,
    //					    _parameters.og_range_pitch);
    //	}
    //}

    return PX4_OK;
}

void OutputBase::update_params( const Parameters &parameters )
{
    memcpy( &_parameters, &parameters, sizeof( Parameters ) );

    // Update the rate controller gains
    _update_rate_controller();
}

void OutputBase::set_stabilize( bool roll_stabilize, bool pitch_stabilize, bool yaw_stabilize )
{
    (void)roll_stabilize;
    (void)pitch_stabilize;
    (void)yaw_stabilize;
}

} /* namespace open_gimbal */
