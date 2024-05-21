/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include <lib/parameters/param.h>
#include <stdint.h>

namespace open_gimbal {

struct Parameters
{
    int32_t og_mode_in;
    int32_t og_mode_out;

    int32_t og_man_pitch;
    int32_t og_man_roll;
    int32_t og_man_yaw;

    int32_t og_do_stab;

    float og_range_pitch;
    float og_range_roll;
    float og_range_yaw;

    float og_off_pitch;
    float og_off_roll;
    float og_off_yaw;

    float og_rate_pitch;
    float og_rate_yaw;

    int32_t og_rc_in_mode;

    int32_t og_comp_id;

    float og_lnd_p_min;
    float og_lnd_p_max;

    float og_roll_p;
    float og_roll_i;
    float og_roll_d;

    float og_pitch_p;
    float og_pitch_i;
    float og_pitch_d;

    float og_yaw_p;
    float og_yaw_i;
    float og_yaw_d;

    float og_motor_roll;
    float og_motor_pitch;
    float og_motor_yaw;

    float og_debug1;
    float og_debug2;
    float og_debug3;
};

struct ParameterHandles
{
    param_t og_mode_in;
    param_t og_mode_out;

    param_t og_man_pitch;
    param_t og_man_roll;
    param_t og_man_yaw;

    param_t og_do_stab;

    param_t og_range_pitch;
    param_t og_range_roll;
    param_t og_range_yaw;

    param_t og_off_pitch;
    param_t og_off_roll;
    param_t og_off_yaw;

    param_t og_rate_pitch;
    param_t og_rate_yaw;

    param_t og_rc_in_mode;

    param_t og_comp_id;

    param_t og_lnd_p_min;
    param_t og_lnd_p_max;

    param_t og_roll_p;
    param_t og_roll_i;
    param_t og_roll_d;

    param_t og_pitch_p;
    param_t og_pitch_i;
    param_t og_pitch_d;

    param_t og_yaw_p;
    param_t og_yaw_i;
    param_t og_yaw_d;

    param_t og_motor_roll;
    param_t og_motor_pitch;
    param_t og_motor_yaw;

    param_t og_debug1;
    param_t og_debug2;
    param_t og_debug3;
};

}  // namespace open_gimbal
