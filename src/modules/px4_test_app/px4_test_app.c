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

#include <drivers/drv_rgbled.h>


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

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "drivers/drv_pwm_output.h"

 
__EXPORT int px4_test_app_main(int argc, char *argv[]);

void control_servo();
int setup_servo();

int px4_test_app_main(int argc, char *argv[])
{
	printf("Usage: %s <number>\n",argv[0]);

	/*
	int rgbleds = -1;
	rgbleds = open(RGBLED_DEVICE_PATH, 0);
	if (rgbleds == -1) {
		warnx("No RGB LED found at " RGBLED_DEVICE_PATH);
	}
	ioctl(rgbleds, RGBLED_SET_COLOR, (unsigned long)atoi(argv[1]));
	*/

	setup_servo();
	control_servo(argc>1 ? atol(argv[1]) : 2100);

	printf("Done\n");

	return OK;
}



const char *dev = "/dev/px4fmu";
unsigned alt_rate = 50;  // 50 Hz
unsigned group = 0;
uint32_t alt_channel_groups = 1;  // Group 0
bool alt_channels_set = true;
bool print_verbose = true;
unsigned channel = 0;  // Channel 1 (0 indexed)
uint32_t set_mask = 0x01;  // Channel 1 only (0x05 is 1 and 3)
unsigned pwm_value = 1500;  // PWM pulse width

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

	exit(0);
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
	return 0;
}

/*void control_servo(unsigned pwm) {
	int ret;

	pwm_value = pwm;

	/*
	} else if (!strcmp(argv[1], "min")) {

		if (set_mask == 0) {
			usage("no channels set");
		}
		if (pwm_value == 0)
			usage("no PWM value provided");

		struct pwm_output_values pwm_values = {.values = {0}, .channel_count = 0};

		for (unsigned i = 0; i < servo_count; i++) {
			if (set_mask & 1<<i) {
				pwm_values.values[i] = pwm_value;
				if (print_verbose)
					warnx("Channel %d: min PWM: %d", i+1, pwm_value);
			}
			pwm_values.channel_count++;
		}

		if (pwm_values.channel_count == 0) {
			usage("no PWM values added");
		} else {

			ret = ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);
			if (ret != OK)
				errx(ret, "failed setting min values");
		}
		return 0;

	} else if (!strcmp(argv[1], "max")) {

		if (set_mask == 0) {
			usage("no channels set");
		}
		if (pwm_value == 0)
			usage("no PWM value provided");

		struct pwm_output_values pwm_values = {.values = {0}, .channel_count = 0};

		for (unsigned i = 0; i < servo_count; i++) {
			if (set_mask & 1<<i) {
				pwm_values.values[i] = pwm_value;
				if (print_verbose)
					warnx("Channel %d: max PWM: %d", i+1, pwm_value);
			}
			pwm_values.channel_count++;
		}

		if (pwm_values.channel_count == 0) {
			usage("no PWM values added");
		} else {

			ret = ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);
			if (ret != OK)
				errx(ret, "failed setting max values");
		}
		return 0;
	}
	*/

	// Run servos
	/* get current servo values */
	/*
	struct pwm_output_values last_spos;

	for (unsigned i = 0; i < servo_count; i++) {


		ret = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);
		if (ret != OK)
			err(1, "PWM_SERVO_GET(%d)", i);
	}
	*/

	/* perform PWM output */

	/* Open console directly to grab CTRL-C signal */
/*	struct pollfd fds;
	fds.fd = 0; // stdin
	fds.events = POLLIN;

	warnx("Press CTRL-C or 'c' to abort.");

	while (1) {
		ret = ioctl(fd, PWM_SERVO_SET(1), pwm_value);
		if (ret != OK)
			err(1, "PWM_SERVO_SET(%d)", 1);
	}

	/*
	while (1) {
		for (unsigned i = 0; i < servo_count; i++) {
			if (set_mask & 1<<i) {
				ret = ioctl(fd, PWM_SERVO_SET(i), pwm_value);
				if (ret != OK)
					err(1, "PWM_SERVO_SET(%d)", i);
			}
		}

		// abort on user request
		char c;
		ret = poll(&fds, 1, 0);
		if (ret > 0) {

		read(0, &c, 1);
			if (c == 0x03 || c == 0x63 || c == 'q') {
				// reset output to the last value
				for (unsigned i = 0; i < servo_count; i++) {
								if (set_mask & 1<<i) {
									ret = ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);
									if (ret != OK)
										err(1, "PWM_SERVO_SET(%d)", i);
								}
							}
				warnx("User abort\n");
				return 0;
			}
		}
		usleep(2000);
	}
	*/
/*	return 0;
}
*/
