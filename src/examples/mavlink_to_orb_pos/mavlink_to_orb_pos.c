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
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
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
#include<uORB/topics/vehicle_local_position.h>
#include<uORB/topics/vehicle_local_position_system_global_offset.h>

__EXPORT int mavlink_to_orb_pos_main(int argc, char *argv[]);

int mavlink_to_orb_pos_main(int argc, char *argv[])
{
	printf("Hello Sky!\n");

	/* subscribe to vehicle_local_position topic */
	int vehicle_local_position_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	orb_set_interval(vehicle_local_position_sub_fd, 1000);

	/* one could wait for multiple topics with this technique, just using one here */
	struct pollfd fds[] = {
		{ .fd = vehicle_local_position_sub_fd,   .events = POLLIN },
	};

	int error_counter = 0;
	struct vehicle_local_position_s pos;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = poll(fds, 1, 1000);
	 
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			printf("[mavlink_to_orb] Got no data within a second\n");
		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				printf("[mavlink_to_orb] ERROR return value from poll(): %d\n"
					, poll_ret);
			}
			error_counter++;
		} else {
	 
			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub_fd, &pos);
				printf("[mavlink_to_orb] vehicle_local_position"
						"\n timestamp: %8.0f,"
						"\n x: %8.4f, y: %8.4f, z: %8.4f, "
						"\n v_x: %8.4f, v_y: %8.4f, v_z: %8.4f, "
						"\n yaw: %8.4f \n\n",
					(double)pos.timestamp,
					(double)pos.x,
					(double)pos.y,
					(double)pos.z,
					(double)pos.vx,
					(double)pos.vy,
					(double)pos.vz,
					(double)pos.yaw);
			}
		}
	}
	return 0;
}
