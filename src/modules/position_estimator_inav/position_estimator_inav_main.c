/****************************************************************************
 *
 *   Copyright (C) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file position_estimator_inav_main.c
 * Model-identification based position estimator for multirotors
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <math.h>
#include <float.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_system_global_offset.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/optical_flow.h>
#include <mavlink/mavlink_log.h>
#include <poll.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_hrt.h>

#include "position_estimator_inav_params.h"
#include "inertial_filter.h"

#define MIN_VALID_W 0.00001f
#define PUB_INTERVAL 10000	// limit publish rate to 100 Hz
#define EST_BUF_SIZE 250000 / PUB_INTERVAL		// buffer size is 0.5s

static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int position_estimator_inav_task; /**< Handle of deamon task / thread */
static bool verbose_mode = false;

__EXPORT int position_estimator_inav_main(int argc, char *argv[]);

int position_estimator_inav_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: position_estimator_inav {start|stop|status} [-v]\n\n");
	exit(1);
}

/**
 * The position_estimator_inav_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_inav_main(int argc, char *argv[])
{
	if (argc < 1) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			exit(0);
		}

		verbose_mode = false;

		if (argc > 1)
			if (!strcmp(argv[2], "-v")) {
				verbose_mode = true;
			}

		thread_should_exit = false;
		position_estimator_inav_task = task_spawn_cmd("position_estimator_inav",
					       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 5000,
					       position_estimator_inav_thread_main,
					       (argv) ? (const char **) &argv[2] : (const char **) NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("app not started");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("app is running");

		} else {
			warnx("app not started");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/****************************************************************************
 * main
 ****************************************************************************/
int position_estimator_inav_thread_main(int argc, char *argv[])
{
	warnx("started");
	int mavlink_fd;
	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
	mavlink_log_info(mavlink_fd, "[inav] started");

	float z_est[2] = { 0.0f, 0.0f };	// pos, vel
	float z_est_prev[2];
	memset(z_est_prev, 0, sizeof(z_est_prev));

	float surface_offset = 0.0f;	// ground level offset from reference altitude
	float surface_offset_rate = 0.0f;	// surface offset change rate

	uint16_t attitude_updates = 0;

	hrt_abstime pub_last = hrt_absolute_time();

	/* declare and safely initialize all structs */
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_local_position_s local_pos;
	memset(&local_pos, 0, sizeof(local_pos));

	/* the "vehicle_local_position_system_global_offset" topic
	   is used to communicate local position from webcam
	   (timestamp,x,y,z,yaw). */
	struct vehicle_local_position_system_global_offset_s local_pos_coord;
	memset(&local_pos_coord, 0, sizeof(local_pos_coord));
    int vehicle_local_position_system_global_offset_sub = orb_subscribe(ORB_ID(vehicle_local_position_system_global_offset));

	/* subscribe */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* advertise */
//	orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_pos);

	thread_running = true;

	/* main loop */
	struct pollfd fds[1] = {
		{ .fd = vehicle_attitude_sub, .events = POLLIN },
	};

	// test frequency
	hrt_abstime t1 = 0;
	hrt_abstime t2 = 0;
	float freq;
	bool updated;

	while (!thread_should_exit) {
		int ret = poll(fds, 1, 20); // wait maximal 20 ms = 50 Hz minimum rate
		hrt_abstime t = hrt_absolute_time();

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(mavlink_fd, "[inav] poll error on init");
			continue;
		} else if (ret > 0) {
			/* vehicle attitude */
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			attitude_updates++;
		}

		/* landing pad */
		orb_check(vehicle_local_position_system_global_offset_sub, &updated);	// landing pad
		if (updated) {

			orb_copy(ORB_ID(vehicle_local_position_system_global_offset), vehicle_local_position_system_global_offset_sub, &local_pos_coord);	// landing pad
			t1 = hrt_absolute_time();
			if (t2 != 0) {
				freq = 1000000 / (t1-t2);
				printf("freq = %6.2f, Hz \n\n", (double)freq);
			}
			t2 = t1;
			printf("[mavlink_to_orb] vehicle_local_position"
					"\n timestamp: %8.0f,"
					"\n x: %8.4f, y: %8.4f, z: %8.4f, "
					"\n yaw: %8.4f \n\n",
				(double)local_pos_coord.timestamp,
				(double)local_pos_coord.x,
				(double)local_pos_coord.y,
				(double)local_pos_coord.z,
				(double)local_pos_coord.yaw);



		}


		if (t > pub_last + PUB_INTERVAL) {
			pub_last = t;

			/* publish local position */

			local_pos.z_valid = true;
			local_pos.v_z_valid = true;
			local_pos.xy_valid = true; 				// can_estimate_xy;
			local_pos.v_xy_valid = true;			// can_estimate_xy;

			local_pos.x = 0.1f; 					// x_est[0];
			local_pos.vx = 0.0f; 					// x_est[1];
			local_pos.y = 0.0f; 					// y_est[0];
			local_pos.vy = 0.0f; 					// y_est[1];
			local_pos.z = -1.0f; 					// z_est[0];
			local_pos.vz = 0.0f; 					// z_est[1];
			local_pos.landed = false;				// landed;
			local_pos.yaw = 0;						// att.yaw;
			local_pos.eph = 0.0f;					// eph;
			local_pos.epv = 0.0f;					// epv;

			local_pos.xy_global = true; 			// local_pos.xy_valid && use_gps_xy;
			local_pos.z_global = true; 				// local_pos.z_valid && use_gps_z;

			local_pos.ref_lat = 0.0;
			local_pos.ref_lon = 0.0;
			local_pos.ref_alt = 0.0;

			local_pos.dist_bottom_valid = true;		// dist_bottom_valid;

			local_pos.dist_bottom = -z_est[0] - surface_offset;
			local_pos.dist_bottom_rate = -z_est[1] - surface_offset_rate;

			local_pos.timestamp = t;
			local_pos.surface_bottom_timestamp = 0;
			local_pos.ref_timestamp = 0;

//			orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &local_pos);

//			printf("local_pos.xy_valid = %d \n\n",local_pos.xy_valid);

		}
	}

	warnx("stopped");
	mavlink_log_info(mavlink_fd, "[inav] stopped");
	thread_running = false;
	return 0;
}
