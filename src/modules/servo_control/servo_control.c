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
 * @file px4_test_app.c
 * Minimal application example for PX4 autopilot.
 */
 
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <systemlib/systemlib.h>
#include <unistd.h>
#include <poll.h>


#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/i2c.h>
#include <nuttx/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>

#include <arch/board/board.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <drivers/drv_pwm_output.h>


#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>



/* Publisher */

#include "servo_control_topic.h"
/* create topic metadata */
ORB_DEFINE(servo_control, struct servo_control_data);
/* file handle that will be used for publishing */
static int pub_topic_handle;

int publisherInit() {
	/* generate the initial data for first publication */
	struct servo_control_data rd = { .r = 0, };
 
	/* advertise the topic and make the initial publication */
	pub_topic_handle = orb_advertise(ORB_ID(servo_control), &rd);
}
int publisherUpdateTopic(int r) {
	/* generate a new random number for publication */
	struct servo_control_data rd = { .r = r, };
 
	/* publish the new data structure */
	orb_publish(ORB_ID(servo_control), pub_topic_handle, &rd);
}

/* Subscriber */
/* file handle that will be used for subscribing */
static int sub_topic_handle;
/* one could wait for multiple topics with this technique, just using one here */
struct pollfd fds[] = {
	{ .fd = 0,   .events = POLLIN },
	/* there could be more file descriptors here, in the form like:
	 * { .fd = other_sub_fd,   .events = POLLIN },
	 */
};
 
int subscriberInit() {
	/* subscribe to the topic */
	fds[0].fd = sub_topic_handle = orb_subscribe(ORB_ID(servo_control));
	orb_set_interval(sub_topic_handle, 1000);
}
 
struct servo_control_data subsriberCheckTopic(bool *success) {
	bool updated;
	struct servo_control_data rd;

	int poll_ret = poll(fds, 1, 20000);
	/* handle the poll result */
	if (poll_ret == 0) {
		/* this means none of our providers is giving us data */
		// printf("[servo_control] Got no data within 2 seconds\n");
	} else if (poll_ret < 0) {
		/* this is seriously bad - should be an emergency */
		/*if (error_counter < 10 || error_counter % 50 == 0) {
			/* use a counter to prevent flooding (and slowing us down) *
			printf("[px4_simple_app] ERROR return value from poll(): %d\n"
				, poll_ret);
		}*/
		// error_counter++;
	} else {
 
		if (fds[0].revents & POLLIN) {
			/* copy sensors raw data into local buffer */
			orb_copy(ORB_ID(servo_control), sub_topic_handle, &rd);
			*success = true;
			return rd;
		}
 	}
	/* check to see whether the topic has updated since the last time we read it */
	/*orb_check(sub_topic_handle, &updated);
 
	if (updated) {
		// make a local copy of the updated data structure *
		orb_copy(ORB_ID(servo_control), sub_topic_handle, &rd);
		printf("Random integer is now %d\n", rd.r);

	}*/
	*success = false;
	return;
}




 
__EXPORT int servo_control_main(int argc, char *argv[]);

void servo_control_daemon_thread(int argc, char *argv[]);
void control_servo();
int setup_servo();
static void servo_control_usage(const char *reason);


static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */


unsigned pwm_value = 2100;  // PWM pulse width


int servo_control_main(int argc, char *argv[])
{
	if (argc < 1)
		servo_control_usage("missing command");
 
	if (!strcmp(argv[1], "start")) {
 
		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			exit(0);
		}
 
		thread_should_exit = false;
		daemon_task = task_spawn_cmd("daemon",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 2000,
					 servo_control_daemon_thread,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}
 
	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}
 
	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning");
		} else {
			warnx("\tnot started");
		}
		exit(0);
	}

	if (argc > 2 && !strcmp(argv[1], "set")) {
		int pwm = 0;
		if (argc > 2)pwm = atol(argv[2]);
		// if (pwm > 0)pwm_value = pwm;
		if (!thread_running) {
			if (pwm > 0)pwm_value = pwm;
			warnx("\tnot started");
		} else {
			// control_servo(pwm_value);
			printf("Publishing %d\n",pwm);
			publisherInit();
			publisherUpdateTopic(pwm);
		}
		exit(0);
	}

	if (argc > 1 && !strcmp(argv[1], "test")) {
		int pwm = 2100;
		if (argc > 2)pwm = atol(argv[2]);
		setup_servo();
		control_servo(pwm_value);
		exit(0);
	}

	if (!strcmp(argv[1], "pub")) {
		publisherInit();
		for (int i=1; ; i++) {
			publisherUpdateTopic(i);
			sleep(3);
			printf("Publishing %d\n",i);
		}
		exit(0);
	}

	servo_control_usage("unrecognized command");
	exit(1);
}


void servo_control_daemon_thread(int argc, char *argv[]) {
	// warnx("[daemon] starting\n");
 
	thread_running = true;
 
 	setup_servo();
	subscriberInit();
	control_servo(pwm_value);
	
	while (!thread_should_exit) {
		// warnx("Hello daemon!\n");
		bool success = false;
		struct servo_control_data val = subsriberCheckTopic(&success);
		if (success) { pwm_value = val.r; }
		control_servo(pwm_value);
		// sleep(2);
	}
 
	// warnx("[daemon] exiting.\n");
 
	thread_running = false;
 
	return 0;
}



const char *dev = "/dev/px4fmu";
unsigned alt_rate = 50;  // 50 Hz
unsigned group = 0;
uint32_t alt_channel_groups = 1;  // Group 0
bool alt_channels_set = true;
bool print_verbose = true;
unsigned channel = 0;  // Channel 1 (0 indexed)
uint32_t set_mask = 0x01;  // Channel 1 only (0x05 is 1 and 3)

int fd = 0;
unsigned servo_count = 0;

void control_servo(unsigned pwm) {
	pwm_value = pwm;
	
	int fd = open("/dev/px4fmu", 0);

	if (fd < 0) {
		puts("open fail");
		exit(1);
	}

	ioctl(fd, PWM_SERVO_ARM, 0);
	ioctl(fd, PWM_SERVO_SET(channel), pwm_value);

	close(fd);

	return 0;
}

int setup_servo() {
	int ret;

	if (print_verbose && set_mask > 0) {
		warnx("Chose channels: ");
		printf("    ");
		for (unsigned i = 0; i<PWM_OUTPUT_MAX_CHANNELS; i++) {
			if (set_mask & 1<<i)
				printf("%d ", i+1);
		}
		printf("\n");
	}

	/* open for ioctl only */
	int fd = open(dev, 0);
	if (fd < 0)
		err(1, "can't open %s", dev);

	/* get the number of servo channels */
	ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	if (ret != OK)
		err(1, "PWM_SERVO_GET_COUNT");

	// Arm
	{
		/* tell safety that its ok to disable it with the switch */
		ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
		if (ret != OK)
			err(1, "PWM_SERVO_SET_ARM_OK");
		/* tell IO that the system is armed (it will output values if safety is off) */
		ret = ioctl(fd, PWM_SERVO_ARM, 0);
		if (ret != OK)
			err(1, "PWM_SERVO_ARM");

		if (print_verbose)
			warnx("Outputs armed");
	}

	// Rate
	{
		/* change alternate PWM rate */
		if (alt_rate > 0) {
			ret = ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, alt_rate);
			if (ret != OK)
				err(1, "PWM_SERVO_SET_UPDATE_RATE (check rate for sanity)");
		}

		/* assign alternate rate to channel groups */
		if (alt_channels_set) {
			uint32_t mask = 0;

			for (group = 0; group < 32; group++) {
				if ((1 << group) & alt_channel_groups) {
					uint32_t group_mask;

					ret = ioctl(fd, PWM_SERVO_GET_RATEGROUP(group), (unsigned long)&group_mask);
					if (ret != OK)
						err(1, "PWM_SERVO_GET_RATEGROUP(%u)", group);

					mask |= group_mask;
				}
			}

			ret = ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, mask);
			if (ret != OK)
				err(1, "PWM_SERVO_SET_SELECT_UPDATE_RATE");
		}
	}

	close(fd);
	return 0;
}


static void servo_control_usage(const char *reason)
{
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: servo_control {start|stop|status|set|test} [-p <additional params>]\n");
}