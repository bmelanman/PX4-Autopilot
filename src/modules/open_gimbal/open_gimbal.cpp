/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "open_gimbal.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#define MOTOR_COUNT 3
#define MOTOR_PITCH (actuator_motors_s::ACTUATOR_FUNCTION_MOTOR1)
#define MOTOR_ROLL  (MOTOR_PITCH + 1)
#define MOTOR_YAW   (MOTOR_PITCH + 2)

#define SLEEP_TIME_US 200000

static uint64_t system_start_time = hrt_absolute_time();

static px4::atomic<bool> thread_should_exit {false};
static px4::atomic<bool> thread_running {false};

struct ThreadData {
	bool new_data;
	int pitch;
	int roll;
	int yaw;
};

static ThreadData *g_thread_data = nullptr;

// Subscriptions
uORB::SubscriptionInterval 	_parameter_update_sub		{ORB_ID(parameter_update), 1_s};
uORB::SubscriptionInterval	_vehicle_angular_velocity_sub	{ORB_ID(vehicle_angular_velocity)};

// Publications
uORB::Publication <vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub	{ORB_ID(vehicle_attitude_setpoint)};

extern "C" __EXPORT int open_gimbal_main(int argc, char *argv[]);

namespace open_gimbal
{

OpenGimbal::~OpenGimbal() {};

OpenGimbal::OpenGimbal(int pitch, int roll, int yaw) : ModuleParams(nullptr)
{
}

int OpenGimbal::update_params(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		ModuleParams::updateParams();
	}

	return 0;
}

int OpenGimbal::publish_attitude_setpoint(vehicle_attitude_setpoint_s &att_sp)
{
	att_sp.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(att_sp);

	return 0;
}

int OpenGimbal::open_gimbal_thread_main(int argc, char *argv[])
{
	vehicle_attitude_setpoint_s att_sp = {};

	thread_running.store(true);

	//update_params(true);

	g_thread_data->new_data = true;

	while (!thread_should_exit.load()) {

		if (g_thread_data->new_data) {

			att_sp.roll_body = g_thread_data->roll * M_DEG_TO_RAD_F;
			att_sp.pitch_body = g_thread_data->pitch * M_DEG_TO_RAD_F;
			att_sp.yaw_body = g_thread_data->yaw * M_DEG_TO_RAD_F;

			att_sp.reset_integral = true;

			g_thread_data->new_data = false;

			publish_attitude_setpoint(att_sp);
		}

		//update_params();

		usleep(SLEEP_TIME_US);
	}

	thread_running.store(false);

	return PX4_OK;
}

int OpenGimbal::start(void)
{
	if (thread_running.load()) {
		PX4_WARN("already running");
		return PX4_ERROR;
	}

	thread_should_exit.store(false);
	int ret = PX4_OK;

	g_thread_data = new ThreadData();
	g_thread_data->new_data = false;
	g_thread_data->pitch = 0;
	g_thread_data->roll = 0;
	g_thread_data->yaw = 0;

	// start the task
	int g_task = px4_task_spawn_cmd("open_gimbal",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT,
					2000,
					(px4_main_t)&open_gimbal_thread_main,
					(char *const *)nullptr);

	if (g_task < 0) {
		PX4_ERR("task start failed");
		return -errno;
	}

	thread_running.store(true);
	return ret;
}

int OpenGimbal::stop(void)
{
	if (!thread_running.load()) {
		PX4_WARN("not running");
		return PX4_ERROR;
	}

	thread_should_exit.store(true);
	int timeout_cnt = 1000;

	while (thread_running.load()) {
		usleep(200000);

		if (--timeout_cnt <= 0) {
			PX4_WARN("failed to stop");
			return PX4_ERROR;
		}
	}

	delete g_thread_data;
	g_thread_data = nullptr;

	return PX4_OK;
}

int OpenGimbal::status(void)
{
	// -- EXAMPLE OUTPUTS --
	// Active: true
	// Angles (deg):
	//   Pitch: 90
	//   Roll:  180
	//   Yaw:   270
	// -- OR ---------------
	// Active: false
	// ---------------------

	if (thread_running.load()) {
		PX4_INFO("Angles (deg):");
		PX4_INFO("  Pitch: %d", g_thread_data->pitch);
		PX4_INFO("  Roll:  %d", g_thread_data->roll);
		PX4_INFO("  Yaw:   %d", g_thread_data->yaw);

		return PX4_OK;
	}

	PX4_INFO("not running");

	return PX4_ERROR;
}

int OpenGimbal::load_thread_data(int pitch, int roll, int yaw)
{

	// Wait for the thread to be ready
	int timeout_cnt = 10;

	while (g_thread_data->new_data) {

		if (--timeout_cnt <= 0) {
			PX4_ERR("failed to load thread data!");
			return PX4_ERROR;
		}

		usleep(SLEEP_TIME_US);
	}

	bool new_data = false;

	if (pitch != -1 && pitch != g_thread_data->pitch) {
		g_thread_data->pitch = pitch;
		new_data = true;
	}

	if (roll != -1 && roll != g_thread_data->roll) {
		g_thread_data->roll = roll;
		new_data = true;
	}

	if (yaw != -1 && yaw != g_thread_data->yaw) {
		g_thread_data->yaw = yaw;
		new_data = true;
	}

	if (!new_data) {
		return PX4_OK;
	}

	g_thread_data->new_data = true;

	return PX4_OK;
}

int OpenGimbal::move(int argc, char *argv[])
{
	if (argc < 4) {
		PX4_ERR("missing arguments");
		usage();
		return PX4_ERROR;
	}

	if (!thread_running.load()) {
		return PX4_ERROR;
	}

	int pitch = -1, roll = -1, yaw = -1;
	bool err = false;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "p:r:y:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			pitch = strtol(myoptarg, nullptr, 10);

			if (pitch < 0 || pitch > 360) {
				PX4_ERR("invalid pitch angle!");
				err = true;
			}

			break;

		case 'r':
			roll = strtol(myoptarg, nullptr, 10);

			if (roll < 0 || roll > 360) {
				PX4_ERR("invalid roll angle!");
				err = true;
			}

			break;

		case 'y':
			yaw = strtol(myoptarg, nullptr, 10);

			if (yaw < 0 || yaw > 360) {
				PX4_ERR("invalid yaw angle!");
				err = true;
			}

			break;

		default:
			PX4_ERR("unrecognized argument");
			usage();
			return PX4_ERROR;
		}
	}

	if (pitch == -1 && roll == -1 && yaw == -1) {
		PX4_ERR("no angles given");
		err = true;
	}

	if (err != false) {
		usage();
		return PX4_ERROR;
	}

	return load_thread_data(pitch, roll, yaw);
}

int OpenGimbal::reset(void)
{
	if (!thread_running.load()) {
		return PX4_ERROR;
	}

	return load_thread_data(0, 0, 0);
}

int OpenGimbal::usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Move each motor of the gimbal to the given angle in degrees.

### Examples
CLI usage example:
$ open_gimbal start
$ open_gimbal move -p 90 -r 180 -y 270
$ open_gimbal status
Active: true
Angles (deg):
  Pitch: 90
  Roll:  180
  Yaw:   270
$ open_gimbal reset
$ open_gimbal status
Active: true
Angles (deg):
  Pitch: 0
  Roll:  0
  Yaw:   0
$ open_gimbal stop
$ open_gimbal status
Active: false

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "open_gimbal");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("move", "Move each motor of the gimbal to the given angle in degrees (-360 to 360)");
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, -360, 360, "Move the pitch motor", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, -360, 360, "Move the roll motor", true);
	PRINT_MODULE_USAGE_PARAM_INT('y', 0, -360, 360, "Move the yaw motor", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset the gimbal to the default position (0, 0, 0)");

	return PX4_OK;
}

int open_gimbal_main(int argc, char *argv[])
{
	if (argc < 2 ||
	    strcmp(argv[1], "-h")    == 0 ||
	    strcmp(argv[1], "help")  == 0 ||
	    strcmp(argv[1], "info")  == 0 ||
	    strcmp(argv[1], "usage") == 0) {
		OpenGimbal::usage();
		return PX4_OK;
	}

	if (!strcmp(argv[1], "start")) {
		return OpenGimbal::start();
	}

	if (!strcmp(argv[1], "stop")) {
		return OpenGimbal::stop();
	}

	if (!strcmp(argv[1], "status")) {
		return OpenGimbal::status();
	}

	if (!strcmp(argv[1], "move")) {
		return OpenGimbal::move(argc, argv);
	}

	if (!strcmp(argv[1], "reset")) {
		return OpenGimbal::reset();
	}

	PX4_ERR("unrecognized command");
	OpenGimbal::usage();
	return PX4_ERROR;
}

} /* namespace open_gimbal */
