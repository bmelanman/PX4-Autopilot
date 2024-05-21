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
#define OG_VERSION "2.3.1"

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
#include <uORB/topics/gimbal_controls.h>
#include <uORB/topics/gimbal_device_information.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>

#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>

// Polling refresh rate in microseconds
#define REFRESH_RATE_US ( (useconds_t)( 1e4 ) )

#define M_180_DEG_F 180.0f

using namespace time_literals;
using namespace open_gimbal;

static px4::atomic<bool> thread_should_exit{ false };
static px4::atomic<bool> thread_running{ false };

static constexpr int input_objs_len_max = 2;

struct ThreadData
{
    bool initialized = false;

    InputBase *input_objs[input_objs_len_max] = { nullptr, nullptr };
    int input_objs_len = 0;
    int last_input_active = -1;

    InputTest *test_input = nullptr;
    OutputBase *output_obj = nullptr;

    ControlData control_data{};
    Parameters thread_params{};
};

static ThreadData *g_thread_data = nullptr;

// Convert a float quat[4] to a matrix::Eulerf
#define QUAT_TO_MATRIX_EULERF( quat ) ( matrix::Eulerf( matrix::Quatf( quat ) ) )

// Subscribe to VehicleAttitude
uORB::Subscription _vehicle_attitude_sub{ ORB_ID( vehicle_attitude ) };
static vehicle_attitude_s g_vehicle_attitude{};
static matrix::Eulerf g_euler_vehicle_attitude;

// Subscribe to VehicleCommand
uORB::Subscription _vehicle_command_sub{ ORB_ID( vehicle_command ) };
vehicle_command_s vehicle_command{};

// Publish to GimbalDeviceInformation
uORB::Publication<gimbal_device_information_s> _gimbal_device_information_pub{ ORB_ID( gimbal_device_information ) };
gimbal_device_information_s gimbal_device_information{};

// Publish to GimbalControls
static uORB::Publication<gimbal_controls_s> _gimbal_controls_pub{ ORB_ID( gimbal_controls ) };

// Gimbal Metadata
#define GIMBAL_INFO_MAX_NAME_LEN ( 32U )
static constexpr char _gimbal_vendor_name[GIMBAL_INFO_MAX_NAME_LEN] = "ARK Electronics";
static constexpr char _gimbal_model_name[GIMBAL_INFO_MAX_NAME_LEN] = "Open-Gimbal";
static constexpr char _gimbal_custom_name[GIMBAL_INFO_MAX_NAME_LEN] = "";

static void usage();
static void update_params( ParameterHandles &param_handles, Parameters &params );
static bool initialize_params( ParameterHandles &param_handles, Parameters &params );

static int open_gimbal_thread_main( int argc, char *argv[] );
extern "C" __EXPORT int open_gimbal_main( int argc, char *argv[] );

static bool _update_vehicle_attitude()
{
    if ( _vehicle_attitude_sub.update( &g_vehicle_attitude ) )
    {
        g_euler_vehicle_attitude = QUAT_TO_MATRIX_EULERF( g_vehicle_attitude.q );
        return true;
    }

    PX4_ERR( "Error getting vehicle attitude! :(" );

    return false;
}

static void _provide_gimbal_device_information( bool force = false )
{
    // Check if the command is a request for gimbal device information
    if ( ( vehicle_command.command == vehicle_command_s::VEHICLE_CMD_REQUEST_MESSAGE &&
           (uint16_t)( vehicle_command.param1 ) == vehicle_command_s::VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION ) ||
         force == true )
    {
        // Setup the response
        gimbal_device_information.timestamp = hrt_absolute_time();

        // TODO: Fill in more fields, maybe via params?
        memcpy( gimbal_device_information.vendor_name, _gimbal_vendor_name, GIMBAL_INFO_MAX_NAME_LEN );
        memcpy( gimbal_device_information.model_name, _gimbal_model_name, GIMBAL_INFO_MAX_NAME_LEN );
        memcpy( gimbal_device_information.custom_name, _gimbal_custom_name, GIMBAL_INFO_MAX_NAME_LEN );
        gimbal_device_information.firmware_version = 0;
        gimbal_device_information.hardware_version = 0;
        gimbal_device_information.uid = 0;

        // Gimbal has roll and pitch axes, as well as an infinite yaw axis
        gimbal_device_information.cap_flags =
            gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS |
            gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS |
            gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS |
            gimbal_device_information_s::GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW;

        // Ignore custom capabilities (for now?)
        gimbal_device_information.custom_cap_flags = 0;

        // Struct's angle limits are in radians, but params are in degrees!
        gimbal_device_information.roll_min = -( g_thread_data->thread_params.og_range_roll ) * M_DEG_TO_RAD_F;
        gimbal_device_information.roll_max = ( g_thread_data->thread_params.og_range_roll ) * M_DEG_TO_RAD_F;

        gimbal_device_information.pitch_min = -( g_thread_data->thread_params.og_range_pitch ) * M_DEG_TO_RAD_F;
        gimbal_device_information.pitch_max = ( g_thread_data->thread_params.og_range_pitch ) * M_DEG_TO_RAD_F;

        gimbal_device_information.yaw_min = -( g_thread_data->thread_params.og_range_yaw ) * M_DEG_TO_RAD_F;
        gimbal_device_information.yaw_max = ( g_thread_data->thread_params.og_range_yaw ) * M_DEG_TO_RAD_F;

        // Set the gimbal device ID
        gimbal_device_information.gimbal_device_compid = g_thread_data->thread_params.og_comp_id;

        // Publish the information
        _gimbal_device_information_pub.publish( gimbal_device_information );

        // Done!
        PX4_INFO( "Gimbal device information provided!" );
    }
}

static int open_gimbal_thread_main( int argc, char *argv[] )
{
    int i = 0;

    // Debug: For moving the pitch axis up and down
    hrt_abstime last_time = hrt_absolute_time();
    int toggle = 1;

    ParameterHandles param_handles;
    ThreadData thread_data;

    // Debug: For printing quaternions
    g_thread_data = &thread_data;

    // Initialize parameters
    if ( !initialize_params( param_handles, thread_data.thread_params ) )
    {
        PX4_ERR( "could not get mount parameters!" );

        // Clean up if we fail to initialize
        for ( i = 0; i < thread_data.input_objs_len; ++i )
        {
            if ( thread_data.input_objs[i] )
            {
                delete ( thread_data.input_objs[i] );
                thread_data.input_objs[i] = nullptr;
            }
        }

        return PX4_ERROR;
    }

    // Set the thread status
    thread_running.store( true );

    // Subscribe to parameter updates
    uORB::SubscriptionInterval parameter_update_sub{ ORB_ID( parameter_update ), 1_s };

    // Initialize test input object to set the initial orientation
    thread_data.test_input = new InputTest( thread_data.thread_params );

    // Create input objects
    thread_data.input_objs[thread_data.input_objs_len++] = thread_data.test_input;
    thread_data.input_objs[thread_data.input_objs_len++] = new InputCAN( thread_data.thread_params );

    // Verify the input objects were created
    for ( i = 0; i < thread_data.input_objs_len; ++i )
    {
        if ( !thread_data.input_objs[i] )
        {
            PX4_ERR( "Input %d failed to initialize! Exiting... :(", i );
            thread_should_exit.store( true );

            break;
        }
    }

    // Initialize input objects
    if ( !thread_should_exit.load() )
    {
        for ( i = 0; i < thread_data.input_objs_len; ++i )
        {
            if ( thread_data.input_objs[i]->initialize() != 0 )
            {
                PX4_ERR( "Input %d failed", i );
                thread_should_exit.store( true );

                break;
            }
        }
    }

    // Create the output object
    thread_data.output_obj = new OutputRC( thread_data.thread_params );

    // Verify the output object was created
    if ( !thread_data.output_obj )
    {
        PX4_ERR( "output memory allocation failed" );
        thread_should_exit.store( true );
    };

    // Publish the gimbal device info while we wait for vehicle attitude to initialize
    _provide_gimbal_device_information( true );

    // Wait up to 10 seconds for the vehicle attitude to initialize
    while ( !thread_should_exit.load() && i++ < 100 )
    {
        // Check vehicle attitude
        if ( _update_vehicle_attitude() )
        {
            break;
        }

        px4_usleep( 100000 );
    }

    static bool new_setpoints, new_params = true;
    static InputBase::UpdateResult update_result;

    while ( !thread_should_exit.load() )
    {
        // Check for parameter updates
        if ( parameter_update_sub.updated() )
        {
            parameter_update_s pupdate;
            parameter_update_sub.copy( &pupdate );

            update_params( param_handles, thread_data.thread_params );
            new_params = true;
        }

        // Check for a new vehicle command
        if ( _vehicle_command_sub.update( &vehicle_command ) )
        {
            _provide_gimbal_device_information();
        }

        new_setpoints = false;
        update_result = InputBase::UpdateResult::NoUpdate;

        // Update input and output objects
        if ( thread_data.input_objs_len > 0 )
        {
            // Check each input object for new setpoints
            for ( i = 0; i < thread_data.input_objs_len; ++i )
            {
                const bool already_active = ( thread_data.last_input_active == i );
                // poll only on active input to reduce latency, or on all if none is active
                const unsigned int poll_timeout = ( already_active || thread_data.last_input_active == -1 ) ? 20 : 0;

                // Update input
                update_result =
                    thread_data.input_objs[i]->update( poll_timeout, thread_data.control_data, already_active );

                // Check if we need to switch to a different input
                if ( update_result == InputBase::UpdateResult::NoUpdate && already_active )
                {
                    // No longer active.
                    thread_data.last_input_active = -1;
                }
                else if ( update_result == InputBase::UpdateResult::UpdatedActive )
                {
                    thread_data.last_input_active = i;
                    new_setpoints = true;
                    break;

                }  // Else ignore, input not active
            }

            // Update output
            if ( thread_data.output_obj->update( thread_data.control_data, new_setpoints, new_params ) == PX4_ERROR )
            {
                PX4_ERR( "Output update failed, exiting gimbal thread..." );
                thread_should_exit.store( true );
            }

            // Publish the mount orientation
            thread_data.output_obj->publish();
        }

        // Reset the flags
        new_params = false;

        // Debug: Move the pitch axis up and down by 10 degrees every 5 seconds
        if ( hrt_elapsed_time( &last_time ) > 5_s )
        {
            thread_data.test_input->set_test_input( 0, toggle * 10, 0 );
            last_time = hrt_absolute_time();
            toggle ^= 1;
        }

        // Wait for a bit before the next loop
        px4_usleep( REFRESH_RATE_US );
    }

    PX4_INFO( "Deinitializing..." );

    // Clean up input objects
    for ( i = 0; i < input_objs_len_max; ++i )
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

//////////////////////////////////////////////////////////
// TODO: Move zero position value storage to parameters //
//////////////////////////////////////////////////////////
/* int init( ThreadData *thread_data )
{
    PX4_INFO( "Initializing..." );

    char c = 'n';

    matrix::Eulerf zero_offsets{ 0, 0, 0 };
    param_t motor_params[3] = {
        param_find( "OG_OFF_ROLL" ), param_find( "OG_OFF_PITCH" ), param_find( "OG_OFF_YAW" )
    };

    param_get( motor_params[0], &zero_offsets( 0 ) );
    param_get( motor_params[1], &zero_offsets( 1 ) );
    param_get( motor_params[2], &zero_offsets( 2 ) );

    // Check if the offset params are already non-zero (to avoid unnecessary initialization)
    if ( !( matrix::isEqualF( ( zero_offsets.abs() ).max(), 0.0f ) ) )
    {
        PX4_WARN(
            "One or more of the motor offset parameters appear to be non-zero, "
            "would you like to continue with initialization anyway? (y/N)      "
        );

        c = getchar();
    }

    if ( c == 'y' || c == 'Y' )
    {
        // Initialize the axis offsets
        PX4_INFO( "The vehicle will now move to find each motor's zero position, press any key to continue..." );

        // Wait for user input
        (void)getchar();

        // Publish (0, 0, 0) to the gimbal controls
        gimbal_controls_s gimbal_controls = { 0 };
        gimbal_controls.timestamp = hrt_absolute_time();
        _gimbal_controls_pub.publish( gimbal_controls );

        // Wait for the gimbal to stabilize
        // TODO: Implement a loop that waits until the changes in attitude are below a certain threshold
        px4_usleep( 1e7 );

        // Get a copy of the current vehicle attitude
        if ( _update_vehicle_attitude() )
        {
            // Set the zero offsets
            zero_offsets( 0 ) = g_euler_vehicle_attitude( 0 );
            zero_offsets( 1 ) = g_euler_vehicle_attitude( 1 );
            zero_offsets( 2 ) = g_euler_vehicle_attitude( 2 );

            g_thread_data->initialized = true;

            PX4_INFO( "Zero position has been recorded!" );

            // Update the parameters
            param_set( motor_params[0], &zero_offsets( 0 ) );
            param_set( motor_params[1], &zero_offsets( 1 ) );
            param_set( motor_params[2], &zero_offsets( 2 ) );

            PX4_INFO( "Parameters updated!" );
        }
        else
        {
            PX4_ERR( "Initialization failed!" );
            return PX4_ERROR;
        }
    }

    PX4_INFO( "Initialization complete!" );

    PX4_INFO(
        "Zero position set to:	\n"
        "  Roll:  %8.4f		\n"
        "  Pitch: %8.4f		\n"
        "  Yaw:   %8.4f		\n",
        (double)( zero_offsets( 0 ) * M_RAD_TO_DEG_F ), (double)( zero_offsets( 1 ) * M_RAD_TO_DEG_F ),
        (double)( zero_offsets( 2 ) * M_RAD_TO_DEG_F )
    );

    return PX4_OK;
} */

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
    int i = 0;

    if ( thread_running.load() && g_thread_data && g_thread_data->test_input )
    {
        if ( g_thread_data->input_objs_len == 0 )
        {
            PX4_INFO( "Input: None" );
        }
        else
        {
            PX4_INFO( "Input Selected" );

            for ( i = 0; i < g_thread_data->input_objs_len; ++i )
            {
                if ( i == g_thread_data->last_input_active )
                {
                    g_thread_data->input_objs[i]->print_status();
                }
            }

            PX4_INFO( "Input not selected" );

            for ( i = 0; i < g_thread_data->input_objs_len; ++i )
            {
                if ( i != g_thread_data->last_input_active )
                {
                    g_thread_data->input_objs[i]->print_status();
                }
            }
        }

        if ( g_thread_data->output_obj )
        {
            g_thread_data->output_obj->print_status();
        }
        else
        {
            PX4_INFO( "Output: None" );
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
            int angles[3] = { 0, 0, 0 };

            for ( int arg_i = 2; arg_i < ( argc - 1 ); ++arg_i )
            {
                for ( int axis_i = 0; axis_i < 3; ++axis_i )
                {
                    if ( !strcmp( argv[arg_i], axis_names[axis_i] ) )
                    {
                        int angle_deg = (int)strtol( argv[arg_i + 1], nullptr, 0 );
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

int open_gimbal_main( int argc, char *argv[] )
{
    if ( argc < 2 )
    {
        PX4_ERR( "missing command" );
        usage();
        status();
        return PX4_ERROR;
    }
    else if ( !strcmp( argv[1], "start" ) )
    {
        return start();
    }
    else if ( !strcmp( argv[1], "test" ) )
    {
        return test( argc, argv );
    }
    else if ( !strcmp( argv[1], "stop" ) )
    {
        return stop();
    }
    else if ( !strcmp( argv[1], "status" ) )
    {
        return status();
    }

    PX4_ERR( "Unrecognized command '%s'", argv[1] );
    usage();

    return PX4_ERROR;
}

void update_params( ParameterHandles &param_handles, Parameters &params )
{
    param_get( param_handles.og_mode_in, &params.og_mode_in );
    param_get( param_handles.og_mode_out, &params.og_mode_out );

    param_get( param_handles.og_man_pitch, &params.og_man_pitch );
    param_get( param_handles.og_man_roll, &params.og_man_roll );
    param_get( param_handles.og_man_yaw, &params.og_man_yaw );

    param_get( param_handles.og_do_stab, &params.og_do_stab );

    param_get( param_handles.og_range_pitch, &params.og_range_pitch );
    param_get( param_handles.og_range_roll, &params.og_range_roll );
    param_get( param_handles.og_range_yaw, &params.og_range_yaw );

    param_get( param_handles.og_off_pitch, &params.og_off_pitch );
    param_get( param_handles.og_off_roll, &params.og_off_roll );
    param_get( param_handles.og_off_yaw, &params.og_off_yaw );

    param_get( param_handles.og_rate_pitch, &params.og_rate_pitch );
    param_get( param_handles.og_rate_yaw, &params.og_rate_yaw );

    param_get( param_handles.og_rc_in_mode, &params.og_rc_in_mode );

    param_get( param_handles.og_comp_id, &params.og_comp_id );

    param_get( param_handles.og_lnd_p_min, &params.og_lnd_p_min );
    param_get( param_handles.og_lnd_p_max, &params.og_lnd_p_max );

    param_get( param_handles.og_roll_p, &params.og_roll_p );
    param_get( param_handles.og_roll_i, &params.og_roll_i );
    param_get( param_handles.og_roll_d, &params.og_roll_d );

    param_get( param_handles.og_pitch_p, &params.og_pitch_p );
    param_get( param_handles.og_pitch_i, &params.og_pitch_i );
    param_get( param_handles.og_pitch_d, &params.og_pitch_d );

    param_get( param_handles.og_yaw_p, &params.og_yaw_p );
    param_get( param_handles.og_yaw_i, &params.og_yaw_i );
    param_get( param_handles.og_yaw_d, &params.og_yaw_d );

    param_get( param_handles.og_debug1, &params.og_debug1 );
    param_get( param_handles.og_debug2, &params.og_debug2 );
    param_get( param_handles.og_debug3, &params.og_debug3 );
}

#define INIT_PARAM( handle, name, err_flag )                    \
    do                                                          \
    {                                                           \
        if ( ( handle = param_find( name ) ) == PARAM_INVALID ) \
        {                                                       \
            PX4_ERR( "failed to find parameter " name );        \
            err_flag = true;                                    \
        }                                                       \
    } while ( 0 )

bool initialize_params( ParameterHandles &param_handles, Parameters &params )
{
    bool err_flag = false;

    INIT_PARAM( param_handles.og_mode_in, "OG_MODE_IN", err_flag );
    INIT_PARAM( param_handles.og_mode_out, "OG_MODE_OUT", err_flag );

    INIT_PARAM( param_handles.og_man_pitch, "OG_MAN_PITCH", err_flag );
    INIT_PARAM( param_handles.og_man_roll, "OG_MAN_ROLL", err_flag );
    INIT_PARAM( param_handles.og_man_yaw, "OG_MAN_YAW", err_flag );

    INIT_PARAM( param_handles.og_do_stab, "OG_DO_STAB", err_flag );

    INIT_PARAM( param_handles.og_range_pitch, "OG_RANGE_PITCH", err_flag );
    INIT_PARAM( param_handles.og_range_roll, "OG_RANGE_ROLL", err_flag );
    INIT_PARAM( param_handles.og_range_yaw, "OG_RANGE_YAW", err_flag );

    INIT_PARAM( param_handles.og_off_pitch, "OG_OFF_PITCH", err_flag );
    INIT_PARAM( param_handles.og_off_roll, "OG_OFF_ROLL", err_flag );
    INIT_PARAM( param_handles.og_off_yaw, "OG_OFF_YAW", err_flag );

    INIT_PARAM( param_handles.og_rate_pitch, "OG_RATE_PITCH", err_flag );
    INIT_PARAM( param_handles.og_rate_yaw, "OG_RATE_YAW", err_flag );

    INIT_PARAM( param_handles.og_rc_in_mode, "OG_RC_IN_MODE", err_flag );

    INIT_PARAM( param_handles.og_comp_id, "OG_COMP_ID", err_flag );

    INIT_PARAM( param_handles.og_lnd_p_min, "OG_LND_P_MIN", err_flag );
    INIT_PARAM( param_handles.og_lnd_p_max, "OG_LND_P_MAX", err_flag );

    INIT_PARAM( param_handles.og_roll_p, "OG_ROLL_P", err_flag );
    INIT_PARAM( param_handles.og_roll_i, "OG_ROLL_I", err_flag );
    INIT_PARAM( param_handles.og_roll_d, "OG_ROLL_D", err_flag );

    INIT_PARAM( param_handles.og_pitch_p, "OG_PITCH_P", err_flag );
    INIT_PARAM( param_handles.og_pitch_i, "OG_PITCH_I", err_flag );
    INIT_PARAM( param_handles.og_pitch_d, "OG_PITCH_D", err_flag );

    INIT_PARAM( param_handles.og_yaw_p, "OG_YAW_P", err_flag );
    INIT_PARAM( param_handles.og_yaw_i, "OG_YAW_I", err_flag );
    INIT_PARAM( param_handles.og_yaw_d, "OG_YAW_D", err_flag );

    INIT_PARAM( param_handles.og_debug1, "OG_DEBUG1", err_flag );
    INIT_PARAM( param_handles.og_debug2, "OG_DEBUG2", err_flag );
    INIT_PARAM( param_handles.og_debug3, "OG_DEBUG3", err_flag );

    if ( !err_flag )
    {
        update_params( param_handles, params );
    }

    return !err_flag;
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
    PRINT_MODULE_USAGE_COMMAND_DESCR( "publish_attitude", "Publish a vehicle attitude message" );
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    PRINT_MODULE_USAGE_COMMAND_DESCR( "version", "Current Version: " OG_VERSION );
}
