/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

// OpenGimbal Version
#define OG_VERSION "2.5.0"

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/tasks.h>
#include <stdlib.h>
#include <string.h>

#include <cstdint>
#include <matrix/matrix/math.hpp>

// Parameters
#include "open_gimbal_params.h"

// Inputs
#include "input_can.h"
#include "input_test.h"

// Outputs
#include "output_rc.h"

// uORB Subscriptions and Publications
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>

// Polling refresh rate in microseconds
#define REFRESH_RATE_US ( (useconds_t)( 1000 ) )  // 1000us = 1ms

using namespace time_literals;
using namespace open_gimbal;

static px4::atomic<bool> thread_should_exit{ true };
static px4::atomic<bool> thread_running{ false };

#define MAX_NUM_INPUT_OBJS ( 2U )

struct ThreadData
{
    unsigned int input_objs_len = 0;
    unsigned int last_input_active = -1;
    InputTest *test_input = nullptr;
    InputBase *input_objs[MAX_NUM_INPUT_OBJS] = { 0 };

    OutputBase *output_obj = nullptr;

    ControlData control_data{};
    Parameters thread_params{};
};

static ThreadData *g_thread_data = nullptr;

// Subscribe to VehicleAttitude
uORB::Subscription _vehicle_attitude_sub{ ORB_ID( vehicle_attitude ) };

// Subscribe to parameter updates
uORB::SubscriptionInterval _parameter_update_sub{ ORB_ID( parameter_update ), 1_s };

static void usage();
static void update_params( ParameterHandles &param_handles, Parameters &params );
static bool initialize_params( ParameterHandles &param_handles, Parameters &params );

static int open_gimbal_thread_main( int argc, char *argv[] );
extern "C" __EXPORT int open_gimbal_main( int argc, char *argv[] );

static int _open_gimbal_deinit( ThreadData &thread_data )
{
    PX4_INFO( "Deinitializing..." );
    unsigned int i;

    // Clean up input objects
    for ( i = 0; i < MAX_NUM_INPUT_OBJS; ++i )
    {
        if ( thread_data.input_objs[i] )
        {
            delete ( thread_data.input_objs[i] );
            thread_data.input_objs[i] = nullptr;
        }
    }

    thread_data.input_objs_len = 0;

    // Clean up the output object
    if ( thread_data.output_obj )
    {
        delete ( thread_data.output_obj );
        thread_data.output_obj = nullptr;
    }

    // Set the thread status
    thread_running.store( false );

    PX4_INFO( "Deinitialization complete, exiting..." );

    return PX4_OK;
}

static int _open_gimbal_init( ParameterHandles &param_handles, ThreadData &thread_data )
{
    // Initialize parameters
    // TODO: This function now always returns true
    if ( !initialize_params( param_handles, thread_data.thread_params ) )
    {
        PX4_ERR( "could not get mount parameters!" );

        // Exit the thread
        thread_should_exit.store( true );
        return PX4_ERROR;
    }

    // Set the thread status
    thread_running.store( true );

    // Initialize test input object to set the initial orientation
    thread_data.test_input = new InputTest( thread_data.thread_params );

    // Create input objects
    thread_data.input_objs[thread_data.input_objs_len++] = thread_data.test_input;
    thread_data.input_objs[thread_data.input_objs_len++] = new InputCAN( thread_data.thread_params );

    unsigned int i;

    // Verify the input objects were created
    for ( i = 0; i < thread_data.input_objs_len; ++i )
    {
        if ( !thread_data.input_objs[i] )
        {
            PX4_ERR( "Input %d failed to initialize! Exiting... :(", i );
            thread_should_exit.store( true );

            // Deinitialize the thread and exit
            goto init_error;
        }
    }

    // Initialize input objects
    for ( i = 0; i < thread_data.input_objs_len; ++i )
    {
        if ( thread_data.input_objs[i]->initialize() != PX4_OK )
        {
            PX4_ERR( "Input %d failed", i );
            thread_should_exit.store( true );

            // Deinitialize the thread and exit
            goto init_error;
        }
    }

    // Create the output object
    thread_data.output_obj = new OutputRC( thread_data.thread_params );

    // Verify the output object was created
    if ( !thread_data.output_obj )
    {
        PX4_ERR( "output memory allocation failed" );
        thread_should_exit.store( true );

        // Deinitialize the thread and exit
        goto init_error;
    };

    // Wait up to 10 seconds for the vehicle attitude to initialize
    while ( !thread_should_exit.load() && i++ < 200 )
    {
        // Check vehicle attitude
        if ( _vehicle_attitude_sub.updated() )
        {
            // Initialization complete!
            PX4_INFO( "Initialization complete!" );
            return PX4_OK;
        }

        PX4_WARN( "Waiting for vehicle attitude to initialize..." );

        px4_usleep( 100000 );
    }

    PX4_ERR( "Could not initialize vehicle attitude!" );

init_error:

    _open_gimbal_deinit( thread_data );
    return PX4_ERROR;
}

static int open_gimbal_thread_main( int argc, char *argv[] )
{
    unsigned int i;

    ParameterHandles param_handles;
    ThreadData thread_data;

    // Debug: For printing quaternions
    g_thread_data = &thread_data;

    // Initialize the gimbal
    if ( _open_gimbal_init( param_handles, thread_data ) != PX4_OK )
    {
        return PX4_ERROR;
    }

    // TODO: Add a param to update_params() to pass the param_update struct
    static parameter_update_s param_update;
    static InputBase::UpdateResult update_result;
    static bool already_active;

    while ( !thread_should_exit.load() )
    {
        // Check for parameter updates
        // if ( _parameter_update_sub.updated() )
        if ( _parameter_update_sub.update( &param_update ) )
        {
            update_params( param_handles, thread_data.thread_params );

            // Update the input and output objects' parameters
            for ( i = 0; i < thread_data.input_objs_len; ++i )
            {
                thread_data.input_objs[i]->update_params( thread_data.thread_params );
            }

            thread_data.output_obj->update_params( thread_data.thread_params );
        }

        // Update input and output objects
        if ( thread_data.input_objs_len > 0 )
        {
            // Check each input object for new setpoints
            for ( i = 0; i < thread_data.input_objs_len; ++i )
            {
                already_active = ( thread_data.last_input_active == i );

                // Update input
                update_result = thread_data.input_objs[i]->update( thread_data.control_data );

                // Check if the input has been updated since we last checked
                if ( update_result == InputBase::UpdateResult::UpdatedActive )
                {
                    thread_data.last_input_active = i;

                    break;
                }

                // Otherwise, update the last active input to `none`
                else if ( already_active )
                {
                    // No longer active.
                    thread_data.last_input_active = -1;
                }
            }

            // Update output
            if ( thread_data.output_obj->update( thread_data.control_data, true ) == PX4_ERROR )
            {
                PX4_ERR( "Output update failed, exiting gimbal thread..." );
                thread_should_exit.store( true );
            }

            // Publish the mount orientation
            thread_data.output_obj->publish();
        }

        // Wait for a bit before the next loop
        px4_usleep( REFRESH_RATE_US );
    }

    return _open_gimbal_deinit( thread_data );
}

int start( void )
{
    if ( thread_running.load() )
    {
        PX4_WARN( "mount driver already running" );
        return PX4_ERROR;
    }

    PX4_INFO( "Starting..." );

    thread_should_exit.store( false );

    // start the thread
    int open_gimbal_thread = px4_task_spawn_cmd(
        "open_gimbal", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 2100, (px4_main_t)&open_gimbal_thread_main,
        (char *const *)nullptr
    );

    if ( open_gimbal_thread < 0 )
    {
        PX4_ERR( "failed to start" );
        return PX4_ERROR;
    }

    int counter = 0;

    while ( thread_running.load() != true )
    {
        px4_usleep( 5000 );

        if ( ++counter >= 100 )
        {
            PX4_ERR( "Timed out waiting for thread to start, terminating..." );
            thread_should_exit.store( true );
            return PX4_ERROR;
        }
    }

    if ( thread_should_exit.load() )
    {
        PX4_ERR( "thread exited during startup, terminating..." );
        return PX4_ERROR;
    }

    PX4_DEBUG( "thread started successfully!" );

    return PX4_OK;
}

int stop( void )
{
    if ( !thread_running.load() )
    {
        PX4_WARN( "mount driver not running" );
        return PX4_OK;
    }

    thread_should_exit.store( true );

    int counter = 0;

    while ( thread_running.load() )
    {
        px4_usleep( 100000 );

        if ( ++counter >= 5 )
        {
            PX4_ERR( "timed out waiting for task to stop" );
            return PX4_ERROR;
        }
    }

    return PX4_OK;
}

int status( void )
{
    unsigned int i;

    if ( thread_running.load() && g_thread_data )
    {
        for ( i = 0; i < g_thread_data->input_objs_len; ++i )
        {
            g_thread_data->input_objs[i]->print_status( i == g_thread_data->last_input_active );
        }

        if ( g_thread_data->output_obj )
        {
            g_thread_data->output_obj->print_status();
        }
        else
        {
            PX4_INFO_RAW( "Output: None" );
        }
    }
    else
    {
        PX4_INFO( "not running" );
    }

    return PX4_OK;
}

int test( int argc, char *argv[] )
{
    if ( !thread_running.load() )
    {
        PX4_WARN( "not running" );
        usage();
        return PX4_ERROR;
    }

    if ( g_thread_data && g_thread_data->test_input )
    {
        if ( argc >= 4 )
        {
            bool found_axis = false;
            const char *axis_names[3] = { "roll", "pitch", "yaw" };
            float angles[3] = { 0, 0, 0 };

            for ( int arg_i = 2; arg_i < ( argc - 1 ); ++arg_i )
            {
                for ( int axis_i = 0; axis_i < 3; ++axis_i )
                {
                    if ( !strcmp( argv[arg_i], axis_names[axis_i] ) )
                    {
                        float angle_deg = (float)strtof( argv[arg_i + 1], nullptr );
                        angles[axis_i] = angle_deg;
                        found_axis = true;
                    }
                }
            }

            if ( !found_axis )
            {
                usage();
                return PX4_ERROR;
            }

            g_thread_data->test_input->set_test_input( angles[0], angles[1], angles[2] );
        }
        else
        {
            PX4_ERR( "not enough arguments" );
            usage();
            return PX4_ERROR;
        }
    }
    else
    {
        PX4_ERR( "test input not available???" );
        return PX4_ERROR;
    }

    return PX4_OK;
}

int monitor( void )
{
    if ( !thread_running.load() )
    {
        PX4_WARN( "not running" );
        usage();
        return PX4_ERROR;
    }

    struct pollfd fds = {
        .fd = STDIN_FILENO,
        .events = POLLIN  // Poll for input events from stdin
    };

    // User input polling loop
    while ( true )
    {
        // Clear the screen
        PX4_INFO_RAW( "\033[2J" );
        // Move the cursor to the home position
        PX4_INFO_RAW( "\033[H" );

        // User prompt
        PX4_INFO_RAW( "Press any key to exit\n" );

        // Print the current gimbal status
        status();

        // Check for user input
        if ( ::poll( &fds, 1, 0 ) > 0 )
        {
            // Exit if input is received
            return PX4_OK;
        }

        // Sleep
        px4_usleep( 1.5 * 1e6 );
    }
}

int open_gimbal_main( int argc, char *argv[] )
{
    static char *cmd;
    cmd = argv[1];

    // Verify the input
    if ( argc < 2 || cmd == nullptr )
    {
        PX4_ERR( "missing command" );
        usage();
        return PX4_ERROR;
    }

    if ( !strcmp( cmd, "start" ) )
    {
        return start();
    }

    if ( !strcmp( cmd, "test" ) )
    {
        return test( argc, argv );
    }

    if ( !strcmp( cmd, "stop" ) )
    {
        return stop();
    }

    if ( !strcmp( cmd, "monitor" ) )
    {
        return monitor();
    }

    if ( !strcmp( cmd, "status" ) )
    {
        return status();
    }

    PX4_ERR( "Unrecognized command '%s'", cmd );
    usage();
    return PX4_ERROR;
}

#define INIT_PARAM( handle, name )                                                      \
    do                                                                                  \
    {                                                                                   \
        handle = param_find( name );                                                    \
        if ( ( handle ) == PARAM_INVALID ) PX4_ERR( "failed to find parameter " name ); \
    } while ( 0 )

#define UPDATE_PARAM( handles, params, name )                                                          \
    do                                                                                                 \
    {                                                                                                  \
        if ( param_get( handles.name, &params.name ) != PX4_OK )                                       \
            PX4_ERR( "failed to update parameter `" #name "` (set to %d)", (int)( params.name = 0 ) ); \
    } while ( 0 )

void update_params( ParameterHandles &param_handles, Parameters &params )
{
    UPDATE_PARAM( param_handles, params, og_mode_in );
    UPDATE_PARAM( param_handles, params, og_mode_out );

    UPDATE_PARAM( param_handles, params, og_man_pitch );
    UPDATE_PARAM( param_handles, params, og_man_roll );
    UPDATE_PARAM( param_handles, params, og_man_yaw );

    UPDATE_PARAM( param_handles, params, og_do_stab );

    UPDATE_PARAM( param_handles, params, og_range_pitch );
    UPDATE_PARAM( param_handles, params, og_range_roll );
    UPDATE_PARAM( param_handles, params, og_range_yaw );

    UPDATE_PARAM( param_handles, params, og_off_pitch );
    UPDATE_PARAM( param_handles, params, og_off_roll );
    UPDATE_PARAM( param_handles, params, og_off_yaw );

    UPDATE_PARAM( param_handles, params, og_rate_pitch );
    UPDATE_PARAM( param_handles, params, og_rate_yaw );

    UPDATE_PARAM( param_handles, params, og_rc_in_mode );

    UPDATE_PARAM( param_handles, params, og_comp_id );

    UPDATE_PARAM( param_handles, params, og_lnd_p_min );
    UPDATE_PARAM( param_handles, params, og_lnd_p_max );

    UPDATE_PARAM( param_handles, params, og_roll_p );
    UPDATE_PARAM( param_handles, params, og_roll_i );
    UPDATE_PARAM( param_handles, params, og_roll_d );
    UPDATE_PARAM( param_handles, params, og_roll_imax );
    UPDATE_PARAM( param_handles, params, og_roll_ff );

    UPDATE_PARAM( param_handles, params, og_pitch_p );
    UPDATE_PARAM( param_handles, params, og_pitch_i );
    UPDATE_PARAM( param_handles, params, og_pitch_d );
    UPDATE_PARAM( param_handles, params, og_pitch_imax );
    UPDATE_PARAM( param_handles, params, og_pitch_ff );

    UPDATE_PARAM( param_handles, params, og_yaw_p );
    UPDATE_PARAM( param_handles, params, og_yaw_i );
    UPDATE_PARAM( param_handles, params, og_yaw_d );
    UPDATE_PARAM( param_handles, params, og_yaw_imax );
    UPDATE_PARAM( param_handles, params, og_yaw_ff );

    UPDATE_PARAM( param_handles, params, og_debug1 );
    UPDATE_PARAM( param_handles, params, og_debug2 );
    UPDATE_PARAM( param_handles, params, og_debug3 );
}

bool initialize_params( ParameterHandles &param_handles, Parameters &params )
{
    INIT_PARAM( param_handles.og_mode_in, "OG_MODE_IN" );
    INIT_PARAM( param_handles.og_mode_out, "OG_MODE_OUT" );

    INIT_PARAM( param_handles.og_man_pitch, "OG_MAN_PITCH" );
    INIT_PARAM( param_handles.og_man_roll, "OG_MAN_ROLL" );
    INIT_PARAM( param_handles.og_man_yaw, "OG_MAN_YAW" );

    INIT_PARAM( param_handles.og_do_stab, "OG_DO_STAB" );

    INIT_PARAM( param_handles.og_range_pitch, "OG_RANGE_PITCH" );
    INIT_PARAM( param_handles.og_range_roll, "OG_RANGE_ROLL" );
    INIT_PARAM( param_handles.og_range_yaw, "OG_RANGE_YAW" );

    INIT_PARAM( param_handles.og_off_pitch, "OG_OFF_PITCH" );
    INIT_PARAM( param_handles.og_off_roll, "OG_OFF_ROLL" );
    INIT_PARAM( param_handles.og_off_yaw, "OG_OFF_YAW" );

    INIT_PARAM( param_handles.og_rate_pitch, "OG_RATE_PITCH" );
    INIT_PARAM( param_handles.og_rate_yaw, "OG_RATE_YAW" );

    INIT_PARAM( param_handles.og_rc_in_mode, "OG_RC_IN_MODE" );

    INIT_PARAM( param_handles.og_comp_id, "OG_COMP_ID" );

    INIT_PARAM( param_handles.og_lnd_p_min, "OG_LND_P_MIN" );
    INIT_PARAM( param_handles.og_lnd_p_max, "OG_LND_P_MAX" );

    INIT_PARAM( param_handles.og_roll_p, "OG_ROLL_P" );
    INIT_PARAM( param_handles.og_roll_i, "OG_ROLL_I" );
    INIT_PARAM( param_handles.og_roll_d, "OG_ROLL_D" );
    INIT_PARAM( param_handles.og_roll_imax, "OG_ROLL_IMAX" );
    INIT_PARAM( param_handles.og_roll_ff, "OG_ROLL_FF" );

    INIT_PARAM( param_handles.og_pitch_p, "OG_PITCH_P" );
    INIT_PARAM( param_handles.og_pitch_i, "OG_PITCH_I" );
    INIT_PARAM( param_handles.og_pitch_d, "OG_PITCH_D" );
    INIT_PARAM( param_handles.og_roll_imax, "OG_PITCH_IMAX" );
    INIT_PARAM( param_handles.og_roll_ff, "OG_PITCH_FF" );

    INIT_PARAM( param_handles.og_yaw_p, "OG_YAW_P" );
    INIT_PARAM( param_handles.og_yaw_i, "OG_YAW_I" );
    INIT_PARAM( param_handles.og_yaw_d, "OG_YAW_D" );
    INIT_PARAM( param_handles.og_roll_imax, "OG_YAW_IMAX" );
    INIT_PARAM( param_handles.og_roll_ff, "OG_YAW_FF" );

    INIT_PARAM( param_handles.og_debug1, "OG_DEBUG1" );
    INIT_PARAM( param_handles.og_debug2, "OG_DEBUG2" );
    INIT_PARAM( param_handles.og_debug3, "OG_DEBUG3" );

    update_params( param_handles, params );

    return true;
}

static void usage()
{
    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Mount/gimbal Gimbal control driver. It maps several different input methods (eg. RC or MAVLink) to a configured
output (eg. AUX channels or MAVLink).

Documentation how to use it is on the [gimbal_control](https://docs.px4.io/main/en/advanced/gimbal_control.html) page.

### Examples
Test the output by setting a angles (all omitted axes are set to 0):
$ open_gimbal test pitch -45 yaw 30
)DESCR_STR"
    );

    PRINT_MODULE_USAGE_NAME( "open_gimbal", "driver" );
    PRINT_MODULE_USAGE_COMMAND( "start" );
    // PRINT_MODULE_USAGE_COMMAND_DESCR("init", "Set the gimbal's zero setpoint");
    PRINT_MODULE_USAGE_COMMAND_DESCR(
        "test", "Test the output: set a fixed angle for one or multiple axes (gimbal must be running)"
    );
    PRINT_MODULE_USAGE_ARG( "<roll|pitch|yaw <angle>>", "Specify an axis and an angle in degrees", false );
    PRINT_MODULE_USAGE_COMMAND_DESCR( "monitor", "Monitor the output of the gimbal" );
    PRINT_MODULE_USAGE_COMMAND_DESCR( "publish_attitude", "Publish a vehicle attitude message" );
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    PRINT_MODULE_USAGE_COMMAND_DESCR( "version", "Current Version: " OG_VERSION );
}
