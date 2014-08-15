/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * OF USE, local_pos_cam, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file display.c
 * Minimal application example for PX4 autopilot
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/vehicle_local_position_system_global_offset.h>

__EXPORT int display_main(int argc, char *argv[]);

int display_main(int argc, char *argv[])
{
//	printf("Hello Sky!\n");

	/* subscribe to vehicle_local_position_system_global_offset*/

	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	int vehicle_local_position_system_global_offset_sub = orb_subscribe(ORB_ID(vehicle_local_position_system_global_offset));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	struct vehicle_local_position_system_global_offset_s local_pos_cam;
	struct optical_flow_s local_pos_flow;
	struct vehicle_attitude_s att;

	orb_set_interval(vehicle_local_position_system_global_offset_sub, 1000);

	/* advertise attitude topic */
	//struct vehicle_attitude_s att;
	//memset(&att, 0, sizeof(att));
	//orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		{ .fd = vehicle_local_position_system_global_offset_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 10; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 1, 1000);
	 
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us local_pos_cam */
			printf("[display] Got no local_pos_cam within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[display] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
	 
			if (fds[0].revents & POLLIN) {
				/* obtained local_pos_cam for the first file descriptor */

				//up till here

				/* copy sensors raw local_pos_cam into local buffer */
				orb_copy(ORB_ID(vehicle_local_position_system_global_offset), vehicle_local_position_system_global_offset_sub, &local_pos_cam);
				orb_copy(ORB_ID(optical_flow), optical_flow_sub, &local_pos_flow);
				orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);

				printf("[display] timestamp: %d %d, z: %8.4f %8.4f, yaw: %8.4f %8.4f,\n",
					local_pos_cam.timestamp,
					local_pos_flow.timestamp,
					(double)local_pos_cam.z,
					(double)local_pos_flow.ground_distance_m*-1000,
					(double)local_pos_cam.yaw,
					(double)att.yaw);

				//\t%8.4f

				/* set att and publish this information for other apps */
				/*att.roll = raw.accelerometer_m_s2[0];
				att.pitch = raw.accelerometer_m_s2[1];
				att.yaw = raw.accelerometer_m_s2[2];
				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);*/
			}
			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	return 0;
}
