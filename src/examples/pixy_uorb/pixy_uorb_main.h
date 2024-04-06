/****************************************************************************
 *
 *   Copyright 2019 NXP.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * @file pixy_uorb_main.h
 *
 *
 * @author Katrin Moritz
 */
#ifndef PIXY_UORB_START_
#define PIXY_UORB_START_

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

#include <stdio.h>
#include <string.h>
#include <sched.h>
#include <cmath>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/pixy_vector.h>

#include "TPixy2.h"
#include "Pixy2CCC.h"
#include "Pixy2I2C_PX4.h"
#include "Pixy2Line.h"
#include "Pixy2Video.h"

#define IGNORED_THRESHOLD 7
#define CAMERA_ANGLE 0.5f
#define SCREEN_WIDTH 79.0f
#define SCREEN_HEIGHT 52.0f

float transformPointX(float x, float y, float angle, float maxX, float maxY);


#endif /*PIXY_UORB_START_*/
