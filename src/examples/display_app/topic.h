/*
 * topic.h
 *
 *  Created on: Jun 11, 2014
 *      Author: MacBookPro
 */

#ifndef TOPIC_H_
#define TOPIC_H_

#include <uORB/uORB.h>

ORB_DECLARE(integer);

static int topic_handle;

struct integer_data {
	int r;
};



#endif /* TOPIC_H_ */
