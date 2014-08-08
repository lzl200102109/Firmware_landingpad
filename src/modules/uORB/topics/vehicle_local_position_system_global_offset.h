/****************************************************************************
 *
 *   Copyright (C) 2014 Berkeley CPCC lab. All rights reserved.
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
 * @file vehicle_local_position_system_global_offset.h
 * Definition of the local fused NED position uORB topic (abuse #89 to have the same function as #32).
 *
 * @author James Lam Yi <jlamyiwork@gmail.com>
 */

#ifndef TOPIC_VEHICLE_LOCAL_POSITION_SYSTEM_GLOBAL_OFFSET_H_
#define TOPIC_VEHICLE_LOCAL_POSITION_SYSTEM_GLOBAL_OFFSET_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Fused local position in NED.
 */
struct vehicle_local_position_system_global_offset_s {
	uint64_t timestamp;		/**< Time of this estimate, in microseconds since system start */
	/* Position in local NED frame */
	float x;				/**< X position in meters in NED earth-fixed frame */
	float y;				/**< X position in meters in NED earth-fixed frame */
	float z;				/**< Z position in meters in NED earth-fixed frame (negative altitude) */
	/* Heading */
	float yaw;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_local_position_system_global_offset);

#endif
