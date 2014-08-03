/*
 * subscriber.cpp
 *
 *  Created on: Jun 11, 2014
 *      Author: MacBookPro
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>			// required for random()
#include <poll.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>

#include"topic.h"

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int subscriber_task;				/**< Handle of daemon task / thread */

__EXPORT int subscriber_main(int argc, char *argv[]);
int subscriber_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason) {
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: subscriber {start|stop|status} \n\n");
}

int subscriber_main(int argc, char *argv[]) {

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("publisher already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		subscriber_task = task_spawn_cmd("subscriber",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 2000,
					 subscriber_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);

}

int subscriber_thread_main(int argc, char *argv[]) {

	// start subscriber
	warnx("[subscriber] starting... \n");
	thread_running = true;

	// initialization
	struct integer_data my_int;							// data structure
	int topic_handle = orb_subscribe(ORB_ID(integer));	// topic handle;
	orb_set_interval(topic_handle, 1000);				// limit subscription rate to 1Hz

	// subscribe data
	while(!thread_should_exit)  {

		struct pollfd fds[1] = { { .fd=topic_handle, .events = POLLIN } };
		int ret = poll(fds, 1, 1000);

		// check whether topic has been updated
		if (fds[0].revents & POLLIN) {
			// make a copy
			orb_copy(ORB_ID(integer), topic_handle, &my_int);
			printf("my_int = %d\n", my_int.r);
		}
	}

	// exit publisher
	warnx("[subscriber] exiting... \n");
	thread_running = false;

	return 0;
}
