/*
 * publisher.cpp
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

ORB_DEFINE(integer, struct integer_data);


static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int publisher_task;				/**< Handle of daemon task / thread */

__EXPORT int publisher_main(int argc, char *argv[]);
int publisher_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason) {
	if (reason)
		warnx("%s\n", reason);
	errx(1, "usage: publisher {start|stop|status} \n\n");
}

int publisher_main(int argc, char *argv[]) {

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("publisher already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		publisher_task = task_spawn_cmd("publisher",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 2000,
					 publisher_thread_main,
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

int publisher_thread_main(int argc, char *argv[]) {

	// start publisher
	warnx("[publisher] starting... \n");
	thread_running = true;

	// initialization
	int topic_handle;										// topic handle;
	struct integer_data my_int = { my_int.r = rand(), };	// generate initial data.
	topic_handle = orb_advertise(ORB_ID(integer), &my_int);	// advertise the topic.

	// publish data
	while(!thread_should_exit) {
		//warnx("publishing...");
		// generate a new data.
		my_int.r = rand();
		// publish the new data structure
		orb_publish(ORB_ID(integer), topic_handle, &my_int);

		sleep(1);
	}

	// exit publisher
	warnx("[publisher] exiting... \n");
	thread_running = false;

	return 0;
}
