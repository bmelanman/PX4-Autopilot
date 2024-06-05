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

#include "input_can.h"

#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <matrix/matrix/math.hpp>

namespace open_gimbal {

InputCAN::InputCAN( Parameters &parameters ) : InputBase( parameters ) {}

InputCAN::~InputCAN()
{
    _vehicle_command_sub.unsubscribe();
    _gimbal_manager_set_attitude_sub.unsubscribe();
    _gimbal_device_information_pub.unadvertise();
}

int InputCAN::initialize()
{
    _vehicle_command_sub.subscribe();
    _gimbal_manager_set_attitude_sub.subscribe();
    _gimbal_device_information_pub.advertise();

    // Publish to GimbalDeviceInformation
    _provide_gimbal_device_information();

    return PX4_OK;
}

void InputCAN::_provide_gimbal_device_information()
{
    // Setup the response
    gimbal_device_information.timestamp = hrt_absolute_time();

    // TODO: Fill in more fields, maybe via params?
    memcpy( gimbal_device_information.vendor_name, InputBase::_gimbal_vendor_name, GIMBAL_INFO_MAX_NAME_LEN );
    memcpy( gimbal_device_information.model_name, InputBase::_gimbal_model_name, GIMBAL_INFO_MAX_NAME_LEN );
    memcpy( gimbal_device_information.custom_name, InputBase::_gimbal_custom_name, GIMBAL_INFO_MAX_NAME_LEN );
    gimbal_device_information.firmware_version = 0;
    gimbal_device_information.hardware_version = 0;
    gimbal_device_information.uid = 0;

    // Gimbal has roll and pitch axes, as well as an infinite yaw axis
    gimbal_device_information.cap_flags = gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS |
                                          gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS |
                                          gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS |
                                          gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW;

    // Ignore custom capabilities (for now?)
    gimbal_device_information.custom_cap_flags = 0;

    // Struct's angle limits are in radians, but params are in degrees!
    gimbal_device_information.roll_min = -( _parameters.og_range_roll ) * M_DEG_TO_RAD_F;
    gimbal_device_information.roll_max = ( _parameters.og_range_roll ) * M_DEG_TO_RAD_F;

    gimbal_device_information.pitch_min = -( _parameters.og_range_pitch ) * M_DEG_TO_RAD_F;
    gimbal_device_information.pitch_max = ( _parameters.og_range_pitch ) * M_DEG_TO_RAD_F;

    gimbal_device_information.yaw_min = -( _parameters.og_range_yaw ) * M_DEG_TO_RAD_F;
    gimbal_device_information.yaw_max = ( _parameters.og_range_yaw ) * M_DEG_TO_RAD_F;

    // Set the gimbal device ID
    gimbal_device_information.gimbal_device_compid = _parameters.og_comp_id;

    // Publish the information
    _gimbal_device_information_pub.publish( gimbal_device_information );

    // Done!
    PX4_INFO( "Gimbal device information provided!" );
}

void InputCAN::_check_for_gimbal_device_information_request()
{
    // Check for a gimbal device information request, OR a vehicle command requesting gimbal device information
    if ( _vehicle_command_sub.update( &vehicle_command ) &&
         vehicle_command.command == vehicle_command_s::VEHICLE_CMD_REQUEST_MESSAGE &&
         ( (uint16_t)vehicle_command.param1 ) == vehicle_command_s::VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION )
    {
        // Provide the gimbal device information
        _provide_gimbal_device_information();

        _connected = true;
    }
}

InputCAN::UpdateResult InputCAN::update( ControlData &control_data )
{
    static unsigned int i;

    // Check for a gimbal device information request
    if ( !_connected )
    {
        _check_for_gimbal_device_information_request();
    }

    // Read the new data from the gimbal_manager_set_attitude topic only if there is new data
    if ( _gimbal_manager_set_attitude_sub.update( &_gimbal_manager_set_attitude ) )
    {
        // Validate the setpoint
        for ( i = 0; i < 4; ++i )
        {
            if ( !PX4_ISFINITE( _gimbal_manager_set_attitude.q[i] ) )
            {
                // Invalid data
                PX4_WARN( "Invalid gimbal attitude data received from CAN bus!" );

                // Disconnect
                _connected = false;

                return UpdateResult::NoUpdate;
            }
        }

        // Save the last input
        _last_input_euler_angle = matrix::Eulerf( matrix::Quatf( _gimbal_manager_set_attitude.q ) );

        // Update the setpoint
        control_data.input_angle_rad = _last_input_euler_angle;

        // Debug: Print a status message
        // PX4_INFO("Received new gimbal attitude from InputCAN!");

        return UpdateResult::UpdatedActive;
    }

    // No new data
    return UpdateResult::NoUpdate;
}

void InputCAN::print_status( bool is_active ) const
{
    PX4_INFO_RAW( "Input: CAN (%s)\n", is_active ? "active" : "inactive" );
    PX4_INFO_RAW( "  Roll : % 4.1f deg\n", math::degrees( (double)_last_input_euler_angle( 0 ) ) );
    PX4_INFO_RAW( "  Pitch: % 4.1f deg\n", math::degrees( (double)_last_input_euler_angle( 1 ) ) );
    PX4_INFO_RAW( "  Yaw  : % 4.1f deg\n", math::degrees( (double)_last_input_euler_angle( 2 ) ) );
}

} /* namespace open_gimbal */
