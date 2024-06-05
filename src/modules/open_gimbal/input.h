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

#include "common.h"
#include "math.h"
#include "open_gimbal_params.h"

#define GIMBAL_INFO_MAX_NAME_LEN ( 32U )

namespace open_gimbal {

struct Parameters;

class InputBase {
   public:
    enum class UpdateResult { NoUpdate, UpdatedActive };

    InputBase() = delete;
    explicit InputBase( Parameters &parameters );
    virtual ~InputBase() = default;

    virtual int initialize() = 0;
    virtual UpdateResult update( ControlData &control_data ) = 0;
    virtual void print_status( bool is_active ) const = 0;

    void update_params( const Parameters &parameters );

   protected:
    Parameters &_parameters;

    // Gimbal Metadata
    static constexpr char _gimbal_vendor_name[GIMBAL_INFO_MAX_NAME_LEN] = "ARK Electronics";
    static constexpr char _gimbal_model_name[GIMBAL_INFO_MAX_NAME_LEN] = "OpenGimbal";
    static constexpr char _gimbal_custom_name[GIMBAL_INFO_MAX_NAME_LEN] = "OG";
};

} /* namespace open_gimbal */
