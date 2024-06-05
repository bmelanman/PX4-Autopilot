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

#include "input_test.h"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <lib/matrix/matrix/math.hpp>

namespace open_gimbal {

InputTest::InputTest( Parameters &parameters ) : InputBase( parameters ) {}

InputTest::UpdateResult InputTest::update( ControlData &control_data )
{
    if ( !_new_input_available.load() )
    {
        return UpdateResult::NoUpdate;
    }

    control_data.input_angle_rad( 0 ) = math::radians( _roll_deg );
    control_data.input_angle_rad( 1 ) = math::radians( _pitch_deg );
    control_data.input_angle_rad( 2 ) = math::radians( _yaw_deg );

    _new_input_available.store( false );

    return UpdateResult::UpdatedActive;
}

int InputTest::initialize()
{
    // Set initial setpoints to 0
    set_test_input( 0, 0, 0 );

    return 0;
}

void InputTest::print_status( bool is_active ) const
{
    PX4_INFO_RAW( "Input: Test (%s)\n", is_active ? "active" : "inactive" );
    PX4_INFO_RAW( "  Roll : % 4.1f deg\n", (double)_roll_deg );
    PX4_INFO_RAW( "  Pitch: % 4.1f deg\n", (double)_pitch_deg );
    PX4_INFO_RAW( "  Yaw  : % 4.1f deg\n", (double)_yaw_deg );
}

void InputTest::set_test_input( float roll_deg, float pitch_deg, float yaw_deg )
{
    _roll_deg = roll_deg;
    _pitch_deg = pitch_deg;
    _yaw_deg = yaw_deg;

    _new_input_available.store( true );
}

} /* namespace open_gimbal */
