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
 * @file pixy_uorb_main.cpp
 * Pixy Vector publisher
 *
 * @author Landon Haugh
 */

#include "pixy_uorb_main.h"

using namespace matrix;

static int daemon_task;             /* Handle of deamon task / thread */

bool threadShouldExit_uorb = false;
bool threadIsRunning_uorb = false;

float absoluteValue(float value)
{
	if (value > 0) {
		return value;

	} else { return -value; }
}

int pixy_uorb_thread_main(int argc, char **argv)
{
	threadIsRunning_uorb = true;

	/* Publication of uORB messages */
	uORB::Publication<pixy_vector_s> _pixy_vector_pub{ORB_ID(pixy_vector)};
	struct pixy_vector_s _pixy_vector;

	/* Pixy2 Instance */
	Pixy2 pixy;
	bool wait = 1;		// needed for waiting for valid data
	usleep(5000);		// give pixy time to init

	// Make sure pixy is ready
	if (pixy.init() == 0) {

		// Print Pixy details to confirm Pixy is publishing data over i2c
		pixy.getVersion();
		pixy.version->print();
		pixy.setLamp(1, 1);
		usleep(1000);

		// Loop indefinitely and publish vector data
		while (1) {
			pixy.line.getAllFeatures(LINE_VECTOR, wait);		// get line vectors from pixy

			if (pixy.line.numVectors) {
				if (pixy.line.vectors[0].m_y1 > pixy.line.vectors[0].m_y0) {
					_pixy_vector.m0_x0 = pixy.line.vectors[0].m_x1;
					_pixy_vector.m0_x1 = pixy.line.vectors[0].m_x0;
					_pixy_vector.m0_y0 = pixy.line.vectors[0].m_y1;
					_pixy_vector.m0_y1 = pixy.line.vectors[0].m_y0;

				} else {
					_pixy_vector.m0_x0 = pixy.line.vectors[0].m_x0;
					_pixy_vector.m0_x1 = pixy.line.vectors[0].m_x1;
					_pixy_vector.m0_y0 = pixy.line.vectors[0].m_y0;
					_pixy_vector.m0_y1 = pixy.line.vectors[0].m_y1;
				}

				if (pixy.line.numVectors > 1) {
					if (pixy.line.vectors[1].m_y1 > pixy.line.vectors[1].m_y0) {
						_pixy_vector.m1_x0 = pixy.line.vectors[1].m_x1;
						_pixy_vector.m1_x1 = pixy.line.vectors[1].m_x0;
						_pixy_vector.m1_y0 = pixy.line.vectors[1].m_y1;
						_pixy_vector.m1_y1 = pixy.line.vectors[1].m_y0;

					} else {
						_pixy_vector.m1_x0 = pixy.line.vectors[1].m_x0;
						_pixy_vector.m1_x1 = pixy.line.vectors[1].m_x1;
						_pixy_vector.m1_y0 = pixy.line.vectors[1].m_y0;
						_pixy_vector.m1_y1 = pixy.line.vectors[1].m_y1;
					}

				} else {
					_pixy_vector.m1_x0 = 0;
					_pixy_vector.m1_x1 = 0;
					_pixy_vector.m1_y0 = 0;
					_pixy_vector.m1_y1 = 0;
				}

			} else {
				_pixy_vector.m0_x0 = 0;
				_pixy_vector.m0_x1 = 0;
				_pixy_vector.m0_y0 = 0;
				_pixy_vector.m0_y1 = 0;

				_pixy_vector.m1_x0 = 0;
				_pixy_vector.m1_x1 = 0;
				_pixy_vector.m1_y0 = 0;
				_pixy_vector.m1_y1 = 0;
			}

			printf("\n\n\n--------------------------------------------------------\n\n\nAll vecs:\n");


			float sumX0 = 0;
			float sumX1 = 0;
			float sumY0 = 0;
			float sumY1 = 0;

			float sumX = 0;
			float sumY = 0;

			float pente_normalized = 0;

			for (int i = 0; i < pixy.line.numVectors; i++) {
				char buf[128];
				sprintf(buf, "Vec%d: x0=%d y0=%d, x1=%d y1=%d\n", i, pixy.line.vectors[i].m_x0, pixy.line.vectors[i].m_y0,
					pixy.line.vectors[i].m_x1, pixy.line.vectors[i].m_y1);

				float x0 = (float)pixy.line.vectors[i].m_x0;
				float x1 = (float)pixy.line.vectors[i].m_x1;
				float y0 = (float)pixy.line.vectors[i].m_y0;
				float y1 = (float)pixy.line.vectors[i].m_y1;

				if (y1 > y0) {
					float temp = y0;
					y0 = y1;
					y1 = temp;

					temp = x0;
					x0 = x1;
					x1 = temp;
				}

				pente_normalized += (y1 - y0) / (x0 - x1);

				/*float x = x0 - x1;
				float y = y0 - y1;*/

				float newX = 0;
				float newY = 0;

				/*if (x < 0) {
					newX = x * sin(0.348f) + y * cos(0.348f);
					newY = -x * cos(0.348f) + y * sin(0.348f);

				} else {
					newX = x * sin(0.348f) - y * cos(0.348f);
					newY = x * cos(0.348f) + y * sin(0.348f);
				}*/

				sumX0 += x0;
				sumX1 += x1;
				sumY0 += y0;
				sumY1 += y1;


				sumX += newX;
				sumY += newY;
				char buff[128];
				sprintf(buff, "Vec%d: xold=%.2f yold=%.2f, x=%.2f y=%.2f\n", i, (double)(x0 - x1), (double)(y0 - y1), (double)newX,
					(double)newY);

				printf(buff);
				printf(buf);
			}

			float pente = 0;

			if ((double)(sumX1 - sumX0) > 0 || (double)(sumX1 - sumX0) < 0) {
				pente = (float)(sumY0 - sumY1) / (float)(sumX1 - sumX0);
				//pente = (float)(sumY) / (float)(sumX);
			}

			//pente = pente_normalized;

			_pixy_vector.pente = pente;

			_pixy_vector.timestamp = hrt_absolute_time();
			_pixy_vector_pub.publish(_pixy_vector);

			char buuuf[128];
			sprintf(buuuf, "\nMain vec: x0=%.2f y0=%.2f, x1=%.2f y1=%.2f\n", (double)sumX0, (double)sumX1, (double)sumY0,
				(double)sumY1);
			printf(buuuf);

			float mainVecMag = (sumX0 - sumX1) * (sumX0 - sumX1) + (sumY0 - sumY1) * (sumY0 - sumY1);
			char buuuuuf[128];
			sprintf(buuuuuf, "\nMain vec mag squared: %.2f\n", (double)mainVecMag);
			printf(buuuuuf);

			mainVecMag = sqrt(mainVecMag);
			char buauuf[128];
			sprintf(buauuf, "\nMain vec mag: %.2f\n", (double)mainVecMag);
			printf(buauuf);

			printf("\nPente of the sum");
			char bufff[128];
			sprintf(bufff, "=%.2f", (double)pente);
			printf(bufff);

			printf("\nSteering value");
			char buuufff[128];
			sprintf(buuufff, "=%.2f, without atan: %.2f", (double)(atan(1 / pente) / 1.5708f), (double)(1 / pente));
			printf(buuufff);

			/*printf("Processed vecs:\n");

			char buf[128];
			sprintf(buf, "vec1: x0=%d y0=%d, x1=%d y1=%d\n", _pixy_vector.m0_x0, _pixy_vector.m0_x1, _pixy_vector.m0_y0, _pixy_vector.m0_y1);
			printf(buf);

			char buff[128];
			sprintf(buff, "vec2: x0=%d y0=%d, x1=%d y1=%d\n\n", _pixy_vector.m1_x0, _pixy_vector.m1_x1, _pixy_vector.m1_y0, _pixy_vector.m1_y1);
			printf(buff);*/

			if (threadShouldExit_uorb) {
				threadIsRunning_uorb = false;
				PX4_INFO("Exit Pixy uORB Thread!\n");
				return 1;
			}
		}

	}

	return 0;
}

extern "C" __EXPORT int pixy_uorb_main(int argc, char *argv[]);
int pixy_uorb_main(int argc, char *argv[])
{

	if (argc < 2) {
		PX4_WARN("usage: pixy_uorb {start|stop|status}\n");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (threadIsRunning_uorb) {
			PX4_INFO("already running\n");
			/* this is not an error */
			return 0;
		}

		threadShouldExit_uorb = false;
		daemon_task = px4_task_spawn_cmd("pixy_uorb",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 pixy_uorb_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		threadShouldExit_uorb = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (threadIsRunning_uorb) {
			PX4_INFO("is running\n");

		} else {
			PX4_INFO("not started\n");
		}

		return 0;
	}

	PX4_WARN("usage: pixy_uorb {start|stop|status}\n");
	return 1;
}
