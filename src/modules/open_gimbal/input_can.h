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

#include <uORB/topics/gimbal_device_information.h>
#include <uORB/topics/gimbal_manager_set_attitude.h>
#include <uORB/topics/vehicle_command.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include "input.h"

namespace open_gimbal {

class InputCAN : public InputBase {
   public:
    InputCAN() = delete;
    explicit InputCAN( Parameters &parameters );

    virtual ~InputCAN();

    virtual void print_status( bool is_active ) const;
    virtual UpdateResult update( ControlData &control_data );
    virtual int initialize();

   private:
    bool _connected = false;
    matrix::Eulerf _last_input_euler_angle{ 0.f, 0.f, 0.f };

    // Check for gimbal device information request
    void _check_for_gimbal_device_information_request();
    // Provide the gimbal device information response
    void _provide_gimbal_device_information();

    // Vehicle Command subscription
    uORB::Subscription _vehicle_command_sub{ ORB_ID( vehicle_command ) };
    // Gimbal controller subscription
    uORB::Subscription _gimbal_manager_set_attitude_sub{ ORB_ID( gimbal_manager_set_attitude ) };
    // Gimbal device information publication
    uORB::Publication<gimbal_device_information_s> _gimbal_device_information_pub{ ORB_ID( gimbal_device_information )
    };

    vehicle_command_s vehicle_command{};
    gimbal_manager_set_attitude_s _gimbal_manager_set_attitude{};
    gimbal_device_information_s gimbal_device_information{};
};

} /* namespace open_gimbal */
