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
 * @file camera_gimbal.hpp
 *
 * @author Gimbal Guys Team <mtprovin@calpoly.edu>
 */

#pragma once

#include <perf/perf_counter.h>
#include <uORB/topics/gimbal_device_set_attitude.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uavcan/equipment/camera_gimbal/AngularCommand.hpp>
#include <uavcan/equipment/camera_gimbal/Status.hpp>
#include <uavcan/uavcan.hpp>

/**
 * @brief The UavcanCameraGimbalController class
 */

class UavcanCameraGimbalController : public px4::ScheduledWorkItem {
   public:
    UavcanCameraGimbalController( uavcan::INode &node );
    ~UavcanCameraGimbalController();

    /*
     * setup callback
     */
    int init();

    void Run() override;

   private:
    // UAVCAN node object
    uavcan::INode &_node;

    // UAVCAN message
    uavcan::equipment::camera_gimbal::AngularCommand _cmd;

    // UAVCAN Publisher
    uavcan::Publisher<uavcan::equipment::camera_gimbal::AngularCommand> _uavcan_pub_raw_cmd;

    // uORB Subscription
    uORB::SubscriptionCallbackWorkItem _gimbal_device_set_attitude_sub{ this, ORB_ID( gimbal_device_set_attitude ) };
};
