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

// TODO:
// :- Enable DEBUG_APPLICATION_INPLACE
// :- Follow up on motors?

#include <cstdint>
#include <stdlib.h>
#include <string.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/tasks.h>

// Parameters
#include "gimbal_params.h"

// Inputs
#include "input_rc.h"
#include "input_test.h"

// Outputs
#include "output_rc.h"

// Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mount_orientation.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/atomic.h>

// Polling refresh rate in microseconds
#define REFRESH_RATE_US ( 100U )

using namespace time_literals;
using namespace open_gimbal;

static px4::atomic<bool> thread_should_exit {false};
static px4::atomic<bool> thread_running {false};

static constexpr int input_objs_len_max = 3;

struct ThreadData {
	InputBase *input_objs[input_objs_len_max] = {nullptr, nullptr, nullptr};
	int input_objs_len = 0;
	int last_input_active = -1;
	OutputBase *output_obj = nullptr;
	InputTest *test_input = nullptr;
	ControlData control_data = {0};
};

static ThreadData *g_thread_data = nullptr;

static void usage();
static void update_params(ParameterHandles &param_handles, Parameters &params);
static bool initialize_params(ParameterHandles &param_handles, Parameters &params);

static int open_gimbal_thread_main(int argc, char *argv[]);
extern "C" __EXPORT int open_gimbal_main(int argc, char *argv[]);

static int open_gimbal_thread_main(int argc, char *argv[])
{
	ParameterHandles param_handles;
	Parameters params {};
	ThreadData thread_data;

	if (!initialize_params(param_handles, params)) {
		PX4_ERR("could not get mount parameters!");

		if (thread_data.test_input) {
			delete (thread_data.test_input);
			thread_data.test_input = nullptr;
		}

		return PX4_ERROR;
	}

	thread_running.store(true);

	uORB::SubscriptionInterval parameter_update_sub{ORB_ID(parameter_update), 1_s};
	g_thread_data = &thread_data;

	thread_data.test_input = new InputTest(params);

	// Initialize input object(s)
	thread_data.input_objs[thread_data.input_objs_len++] = thread_data.test_input;
	//thread_data.input_objs[thread_data.input_objs_len++] = new InputRC(params);

	for (int i = 0; i < thread_data.input_objs_len; ++i) {
		if (!thread_data.input_objs[i]) {
			PX4_ERR("input objs memory allocation failed");
			thread_should_exit.store(true);

			break;
		}
	}

	if (!thread_should_exit.load()) {
		for (int i = 0; i < thread_data.input_objs_len; ++i) {
			if (thread_data.input_objs[i]->initialize() != 0) {
				PX4_ERR("Input %d failed", i);
				thread_should_exit.store(true);

				break;
			}
		}
	}

	// Initialize output object(s)
	thread_data.output_obj = new OutputRC(params);

	if (!thread_data.output_obj) {
		PX4_ERR("output memory allocation failed");
		thread_should_exit.store(true);
	};

	PX4_INFO("Initialization complete!");

	while (!thread_should_exit.load()) {

		const bool updated = parameter_update_sub.updated();

		if (updated) {
			parameter_update_s pupdate;
			parameter_update_sub.copy(&pupdate);
			update_params(param_handles, params);
		}

		if (thread_data.last_input_active == -1) {
			// Reset control as no one is active anymore, or yet.
			thread_data.control_data.device_compid = 0;
		}

		InputBase::UpdateResult update_result = InputBase::UpdateResult::NoUpdate;

		if (thread_data.input_objs_len > 0) {

			// get input: we cannot make the timeout too large, because the output needs to update
			// periodically for stabilization and angle updates.

			for (int i = 0; i < thread_data.input_objs_len; ++i) {

				const bool already_active = (thread_data.last_input_active == i);
				// poll only on active input to reduce latency, or on all if none is active
				const unsigned int poll_timeout =
					(already_active || thread_data.last_input_active == -1) ? 20 : 0;

				// Update input
				update_result = thread_data.input_objs[i]->update(poll_timeout, thread_data.control_data, already_active);

				// Check if we need to switch to a different input
				if (update_result == InputBase::UpdateResult::NoUpdate) {
					if (already_active) {
						// No longer active.
						thread_data.last_input_active = -1;
					}

				} else if (update_result == InputBase::UpdateResult::UpdatedActive) {
					thread_data.last_input_active = i;
					break;

				} else if (update_result == InputBase::UpdateResult::UpdatedActiveOnce) {
					thread_data.last_input_active = -1;
					break;

				} // Else ignore, input not active

			}

			// Set stabilization mode
			switch (params.mnt_do_stab) {
			case 1:
				// Always stabilize
				thread_data.output_obj->set_stabilize(true, true, true);
				break;

			case 2:
				// Only stabilize the yaw axis
				thread_data.output_obj->set_stabilize(false, false, true);
				break;

			default:
				// Never stabilize
				thread_data.output_obj->set_stabilize(false, false, false);
				break;
			}

			// Update output
			thread_data.output_obj->update(
				thread_data.control_data,
				update_result != InputBase::UpdateResult::NoUpdate, thread_data.control_data.device_compid);

			// Publish the mount orientation
			thread_data.output_obj->publish();

		}

		// Wait for a bit before the next loop
		px4_usleep(REFRESH_RATE_US);
	}

	PX4_INFO("Deinitializing...");

	g_thread_data = nullptr;

	for (int i = 0; i < input_objs_len_max; ++i) {
		if (thread_data.input_objs[i]) {
			delete (thread_data.input_objs[i]);
			thread_data.input_objs[i] = nullptr;
		}
	}

	thread_data.input_objs_len = 0;

	if (thread_data.output_obj) {
		delete (thread_data.output_obj);
		thread_data.output_obj = nullptr;
	}

	thread_running.store(false);

	PX4_INFO("Deinitialization complete, exiting...");

	return PX4_OK;
}

int start(void)
{
	if (thread_running.load()) {
		PX4_WARN("mount driver already running");
		return PX4_ERROR;
	}

	PX4_INFO("Starting...");

	thread_should_exit.store(false);

	// start the task
	int open_gimbal_task = px4_task_spawn_cmd(
				       "open_gimbal",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_DEFAULT,
				       2100,
				       (px4_main_t)&open_gimbal_thread_main,
				       (char *const *)nullptr);

	if (open_gimbal_task < 0) {
		PX4_ERR("failed to start");
		return PX4_ERROR;
	}

	int counter = 0;

	while (thread_running.load() != true) {
		px4_usleep(5000);

		if (++counter >= 100) {
			PX4_ERR("timed out waiting for task to start");
			thread_should_exit.store(true);
			return PX4_ERROR;
		}
	}

	if (thread_should_exit.load()) {
		PX4_ERR("task exited during startup");
		return PX4_ERROR;
	}

	PX4_INFO("Task started successfully!");

	return PX4_OK;
}

int stop(void)
{
	if (!thread_running.load()) {
		PX4_WARN("mount driver not running");
		return PX4_OK;
	}

	thread_should_exit.store(true);

	int counter = 0;

	while (thread_running.load()) {
		px4_usleep(100000);

		if (++counter >= 5) {
			PX4_ERR("timed out waiting for task to stop");
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

int status(void)
{
	if (thread_running.load() && g_thread_data && g_thread_data->test_input) {

		if (g_thread_data->input_objs_len == 0) {
			PX4_INFO("Input: None");

		} else {
			PX4_INFO("Input Selected");

			for (int i = 0; i < g_thread_data->input_objs_len; ++i) {
				if (i == g_thread_data->last_input_active) {
					g_thread_data->input_objs[i]->print_status();
				}
			}

			PX4_INFO("Input not selected");

			for (int i = 0; i < g_thread_data->input_objs_len; ++i) {
				if (i != g_thread_data->last_input_active) {
					g_thread_data->input_objs[i]->print_status();
				}
			}

			PX4_INFO("Primary control:   %d/%d", 0, 0);
			// g_thread_data->control_data.sysid_primary_control, g_thread_data->control_data.compid_primary_control);

		}

		if (g_thread_data->output_obj) {
			g_thread_data->output_obj->print_status();

		} else {
			PX4_INFO("Output: None");
		}

	} else {
		PX4_INFO("not running");
	}

	return PX4_OK;
}

int test(int argc, char *argv[])
{
	if (!thread_running.load()) {
		PX4_WARN("not running");
		usage();
		return PX4_ERROR;
	}

	if (g_thread_data && g_thread_data->test_input) {

		if (argc >= 4) {

			bool found_axis = false;
			const char *axis_names[3] = {"roll", "pitch", "yaw"};
			int angles[3] = { 0, 0, 0 };

			for (int arg_i = 2 ; arg_i < (argc - 1); ++arg_i) {

				for (int axis_i = 0; axis_i < 3; ++axis_i) {

					if (!strcmp(argv[arg_i], axis_names[axis_i])) {

						int angle_deg = (int)strtol(argv[arg_i + 1], nullptr, 0);
						angles[axis_i] = angle_deg;
						found_axis = true;
					}
				}
			}

			if (!found_axis) {
				usage();
				return PX4_ERROR;
			}

			g_thread_data->test_input->set_test_input(angles[0], angles[1], angles[2]);

		} else {
			PX4_ERR("not enough arguments");
			usage();
			return PX4_ERROR;
		}

	} else {
		PX4_ERR("test input not available???");
		return PX4_ERROR;
	}

	return PX4_OK;
}

#define OPEN_GIMBAL_GYRO_TEST_SLEEP_TIME ( 1000000U )

int gyro_test(int argc, char *argv[])
{
	int i = 1000;

	// Orientation subscription
	uORB::Subscription mount_sub{ORB_ID(mount_orientation)};
	mount_orientation_s mount;

	PX4_INFO("\n\n");

	// Loop until the user presses CTRL+C or the driver
	// is stopped for some other reason.
	while (i--) {

		// Get the latest mount data
		mount_sub.copy(&mount);

		// Move the cursor to the top of the screen and clear the line
		PX4_INFO("\033[2J\033[1;1H");

		// Print the current gyro output
		PX4_INFO("Current Orientation: %8.4f %8.4f %8.4f",
			 (double)mount.attitude_euler_angle[0],
			 (double)mount.attitude_euler_angle[1],
			 (double)mount.attitude_euler_angle[2]);

		// Wait for a bit before the next loop
		px4_usleep(OPEN_GIMBAL_GYRO_TEST_SLEEP_TIME);
	}

	return PX4_OK;
}

int primary_control(int argc, char *argv[])
{
	if (thread_running.load() && g_thread_data && g_thread_data->test_input) {

		if (argc == 4) {
			//g_thread_data->control_data.sysid_primary_control = (uint8_t)strtol(argv[2], nullptr, 0);
			//g_thread_data->control_data.compid_primary_control = (uint8_t)strtol(argv[3], nullptr, 0);

			PX4_INFO("Control set to: %d/%d", 0, 0);
			// g_thread_data->control_data.sysid_primary_control,
			// g_thread_data->control_data.compid_primary_control);

			return PX4_OK;

		} else {
			PX4_ERR("not enough arguments");
			usage();
			return PX4_ERROR;
		}

	} else {
		PX4_WARN("not running");
		usage();
		return PX4_ERROR;
	}
}

int open_gimbal_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_ERR("missing command");
		usage();
		return PX4_ERROR;
	}

	if (!strcmp(argv[1], "start")) {
		return start();

	} else if (!strcmp(argv[1], "stop")) {
		return stop();

	} else if (!strcmp(argv[1], "status")) {
		return status();

	} else if (!strcmp(argv[1], "gyro_test")) {
		return gyro_test(argc, argv);

	} else if (!strcmp(argv[1], "test")) {
		return test(argc, argv);

		//} else if (!strcmp(argv[1], "primary-control")) {
		//	return primary_control(argc, argv);

	}

	PX4_ERR("unrecognized command");
	usage();
	return PX4_ERROR;
}

void update_params(ParameterHandles &param_handles, Parameters &params)
{
	param_get(param_handles.mnt_mode_in, &params.mnt_mode_in);
	param_get(param_handles.mnt_mode_out, &params.mnt_mode_out);

	param_get(param_handles.mnt_mav_sysid_v1, &params.mnt_mav_sysid_v1);
	param_get(param_handles.mnt_mav_compid_v1, &params.mnt_mav_compid_v1);

	param_get(param_handles.mnt_man_pitch, &params.mnt_man_pitch);
	param_get(param_handles.mnt_man_roll, &params.mnt_man_roll);
	param_get(param_handles.mnt_man_yaw, &params.mnt_man_yaw);

	param_get(param_handles.mnt_do_stab, &params.mnt_do_stab);

	param_get(param_handles.mnt_range_pitch, &params.mnt_range_pitch);
	param_get(param_handles.mnt_range_roll, &params.mnt_range_roll);
	param_get(param_handles.mnt_range_yaw, &params.mnt_range_yaw);

	param_get(param_handles.mnt_off_pitch, &params.mnt_off_pitch);
	param_get(param_handles.mnt_off_roll, &params.mnt_off_roll);
	param_get(param_handles.mnt_off_yaw, &params.mnt_off_yaw);

	//param_get(param_handles.mav_sysid, &params.mav_sysid);
	//param_get(param_handles.mav_compid, &params.mav_compid);
	params.mav_sysid = 0;
	params.mav_compid = 0;

	param_get(param_handles.mnt_rate_pitch, &params.mnt_rate_pitch);
	param_get(param_handles.mnt_rate_yaw, &params.mnt_rate_yaw);

	param_get(param_handles.mnt_rc_in_mode, &params.mnt_rc_in_mode);

	param_get(param_handles.mnt_lnd_p_min, &params.mnt_lnd_p_min);
	param_get(param_handles.mnt_lnd_p_max, &params.mnt_lnd_p_max);
}

#define INIT_PARAM(handle, name, err_flag) do { \
        if ((handle = param_find(name)) == PARAM_INVALID) { \
            PX4_ERR("failed to find parameter " name); \
            err_flag = true; \
        } \
    } while (0)

bool initialize_params(ParameterHandles &param_handles, Parameters &params)
{
	bool err_flag = false;

	INIT_PARAM(param_handles.mnt_mode_in, 		"MNT_MODE_IN", 	    	err_flag);
	INIT_PARAM(param_handles.mnt_mode_out, 		"MNT_MODE_OUT", 	err_flag);

	INIT_PARAM(param_handles.mnt_mav_sysid_v1, 	"MNT_MAV_SYSID", 	err_flag);
	INIT_PARAM(param_handles.mnt_mav_compid_v1, 	"MNT_MAV_COMPID", 	err_flag);

	INIT_PARAM(param_handles.mnt_man_pitch, 	"MNT_MAN_PITCH", 	err_flag);
	INIT_PARAM(param_handles.mnt_man_roll, 		"MNT_MAN_ROLL", 	err_flag);
	INIT_PARAM(param_handles.mnt_man_yaw, 		"MNT_MAN_YAW",		err_flag);

	INIT_PARAM(param_handles.mnt_do_stab, 		"MNT_DO_STAB", 	    	err_flag);

	INIT_PARAM(param_handles.mnt_range_pitch, 	"MNT_RANGE_PITCH", 	err_flag);
	INIT_PARAM(param_handles.mnt_range_roll, 	"MNT_RANGE_ROLL", 	err_flag);
	INIT_PARAM(param_handles.mnt_range_yaw, 	"MNT_RANGE_YAW", 	err_flag);

	INIT_PARAM(param_handles.mnt_off_pitch, 	"MNT_OFF_PITCH", 	err_flag);
	INIT_PARAM(param_handles.mnt_off_roll, 		"MNT_OFF_ROLL", 	err_flag);
	INIT_PARAM(param_handles.mnt_off_yaw, 		"MNT_OFF_YAW", 	    	err_flag);

	INIT_PARAM(param_handles.mnt_rate_pitch, 	"MNT_RATE_PITCH", 	err_flag);
	INIT_PARAM(param_handles.mnt_rate_yaw, 		"MNT_RATE_YAW", 	err_flag);

	INIT_PARAM(param_handles.mnt_rc_in_mode, 	"MNT_RC_IN_MODE", 	err_flag);

	INIT_PARAM(param_handles.mnt_lnd_p_min, 	"MNT_LND_P_MIN", 	err_flag);
	INIT_PARAM(param_handles.mnt_lnd_p_max, 	"MNT_LND_P_MAX", 	err_flag);

	if (!err_flag) {
		update_params(param_handles, params);
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
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("open_gimbal", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	//PRINT_MODULE_USAGE_COMMAND_DESCR("primary-control", "Set who is in control of gimbal");
	//PRINT_MODULE_USAGE_ARG("<sysid> <compid>", "MAVLink system ID and MAVLink component ID", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test the output: set a fixed angle for one or multiple axes (gimbal must be running)");
	PRINT_MODULE_USAGE_ARG("roll|pitch|yaw <angle>", "Specify an axis and an angle in degrees", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("gyro_test", "Print the current gyro output");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}
