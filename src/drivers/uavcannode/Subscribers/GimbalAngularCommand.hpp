/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/*
 * Converts UAVCAN camera_gimbal::AngularCommand messages to uORB gimbal_manager_set_attitude messages.
 *
 * CAN -> CAN_Messsage{camera_gimbal::AngularCommand} -> uORB_Message{gimbal_manager_set_attitude} -> uORB
 */

#include <uORB/topics/gimbal_manager_set_attitude.h>

#include <uORB/Publication.hpp>
#include <uavcan/equipment/camera_gimbal/AngularCommand.hpp>

#include "UavcanSubscriberBase.hpp"

namespace uavcannode {

class GimbalAngularCommand;

typedef uavcan::MethodBinder<
    GimbalAngularCommand *, void ( GimbalAngularCommand::* )( const uavcan::ReceivedDataStructure<
                                                              uavcan::equipment::camera_gimbal::AngularCommand> & )>
    GimbalAngularCommandBinder;

class GimbalAngularCommand
    : public UavcanSubscriberBase,
      private uavcan::Subscriber<uavcan::equipment::camera_gimbal::AngularCommand, GimbalAngularCommandBinder> {
   public:
    GimbalAngularCommand( uavcan::INode &node )
        : UavcanSubscriberBase( uavcan::equipment::camera_gimbal::AngularCommand::DefaultDataTypeID ),
          uavcan::Subscriber<uavcan::equipment::camera_gimbal::AngularCommand, GimbalAngularCommandBinder>( node )
    {
    }

    bool init()
    {
        if ( start( GimbalAngularCommandBinder( this, &GimbalAngularCommand::callback ) ) < 0 )
        {
            PX4_ERR( "uavcan::equipment::camera_gimbal::AngularCommand subscription failed" );
            return false;
        }

        if ( _gimbal_manager_set_attitude_pub.advertise() != true )
        {
            PX4_ERR( "gimbal_manager_set_attitude publication failed" );
            return false;
        }

        PX4_INFO( "GimbalAngularCommand initialized successfully" );

        return true;
    }

    void PrintInfo() const override
    {
        printf(
            "\t%s:%d -> %s\n", uavcan::equipment::camera_gimbal::AngularCommand::getDataTypeFullName(),
            uavcan::equipment::camera_gimbal::AngularCommand::DefaultDataTypeID,
            _gimbal_manager_set_attitude_pub.get_topic()->o_name
        );
    }

   private:
    uORB::Publication<gimbal_manager_set_attitude_s> _gimbal_manager_set_attitude_pub{
        ORB_ID( gimbal_manager_set_attitude )
    };

    void callback( const uavcan::ReceivedDataStructure<uavcan::equipment::camera_gimbal::AngularCommand> &msg )
    {
        static gimbal_manager_set_attitude_s gimbal_manager_set_attitude{};

        memcpy( &gimbal_manager_set_attitude.q, &msg.quaternion_xyzw, sizeof( gimbal_manager_set_attitude.q ) );

        if ( !_gimbal_manager_set_attitude_pub.publish( gimbal_manager_set_attitude ) )
        {
            PX4_ERR( "gimbal_manager_set_attitude publication failed" );
        }
    }
};

}  // namespace uavcannode
