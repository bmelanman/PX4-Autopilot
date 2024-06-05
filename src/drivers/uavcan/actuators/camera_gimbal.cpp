/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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

/**
 * @file camera_gimbal.cpp
 *
 * @author Matthew Province, Gimbal Guys Team <mtprovin@calpoly.edu>
 *
 * Converts uORB gimbal_device_set_attitude messages to UAVCAN camera_gimbal::AngularCommand messages.
 */

#include "camera_gimbal.hpp"

#include <systemlib/err.h>

#include <uavcan/equipment/camera_gimbal/Mode.hpp>

UavcanCameraGimbalController::UavcanCameraGimbalController( uavcan::INode &node )
    : ScheduledWorkItem( MODULE_NAME, px4::wq_configurations::uavcan ), _node( node ), _uavcan_pub_raw_cmd( node )
{
    _uavcan_pub_raw_cmd.setPriority( uavcan::TransferPriority::OneLowerThanHighest );
}

UavcanCameraGimbalController::~UavcanCameraGimbalController() {}

int UavcanCameraGimbalController::init()
{
    // Register the uORB subscription
    if ( !_gimbal_device_set_attitude_sub.registerCallback() )
    {
        PX4_ERR( "UavcanCameraGimbalController callback registration failed! :(" );
        return false;
    }

    // One or more updates from the ORB topic should trigger the callback
    _gimbal_device_set_attitude_sub.set_required_updates( 1 );

    PX4_INFO( "UavcanCameraGimbalController initialized!" );

    return true;
}

void UavcanCameraGimbalController::Run()
{
    static gimbal_device_set_attitude_s gimbal_device_set_attitude;

    // TODO: Is this check necessary?
    if ( !_gimbal_device_set_attitude_sub.update( &gimbal_device_set_attitude ) )
    {
        PX4_ERR( "Update not available in UavcanCameraGimbalController::Run()???" );
        return;
    }

    memcpy(
        &_cmd.angular_velocity_xyz, &gimbal_device_set_attitude.angular_velocity, sizeof( _cmd.angular_velocity_xyz )
    );

    // Setup the rest of the uavcan message
    _cmd.gimbal_id = gimbal_device_set_attitude.target_component;
    _cmd.mode = uavcan::equipment::camera_gimbal::Mode::COMMAND_MODE_ANGULAR_VELOCITY;

    // Publish the angular velocity to the gimbal
    if ( _uavcan_pub_raw_cmd.broadcast( _cmd ) < 0 )
    {
        PX4_ERR( "Error publishing camera_gimbal::AngularCommand! :(" );
    }
}
