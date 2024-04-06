#include "nxpcup_race.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

double printed_value = 1001;

uint8_t p_vec1_x0 = 0;
uint8_t p_vec1_y0 = 0;
uint8_t p_vec1_x1 = 0;
uint8_t p_vec1_y1 = 0;

uint8_t p_vec2_x0 = 0;
uint8_t p_vec2_y0 = 0;
uint8_t p_vec2_x1 = 0;
uint8_t p_vec2_y1 = 0;

double *powerLookupTable;
uint8_t initialized;

uint8_t get_num_vectors(Vector &vec1, Vector &vec2)
{
	uint8_t numVectors = 0;

	if (!(vec1.m_x0 == 0 && vec1.m_x1 == 0 && vec1.m_y0 == 0 && vec1.m_y1 == 0)) { numVectors++; }

	if (!(vec2.m_x0 == 0 && vec2.m_x1 == 0 && vec2.m_y0 == 0 && vec2.m_y1 == 0)) { numVectors++; }

	return numVectors;
}

Vector copy_vectors(const pixy_vector_s &pixy, uint8_t num)
{
	Vector vec;

	if (num == 1) {
		vec.m_x0 = pixy.m0_x0;
		vec.m_x1 = pixy.m0_x1;
		vec.m_y0 = pixy.m0_y0;
		vec.m_y1 = pixy.m0_y1;
	}

	if (num == 2) {
		vec.m_x0 = pixy.m1_x0;
		vec.m_x1 = pixy.m1_x1;
		vec.m_y0 = pixy.m1_y0;
		vec.m_y1 = pixy.m1_y1;
	}

	return vec;
}

roverControl raceTrack(const pixy_vector_s &pixy, PID_t &PID, PID_t &PID2)
{
	//Vector main_vec;
	onInitialize();
	Vector vec1 = copy_vectors(pixy, 1);
	Vector vec2 = copy_vectors(pixy, 2);
	float pente = pixy.pente;
	//uint8_t frameWidth = 79;
	//uint8_t frameHeight = 52;
	//int16_t window_center = (frameWidth / 2);
	roverControl control{};
	//float x, y;					 // calc gradient and position of main vector
	//static hrt_abstime no_line_time = 0;		// time variable for time since no line detected
	//hrt_abstime time_diff = 0;
	//static bool first_call = true;

	static hrt_abstime old_t = hrt_absolute_time();
	hrt_abstime t = hrt_absolute_time();
	double diff = difftime(t, old_t);
	// milliseconds
	diff = diff / 1000;

	uint8_t num_vectors = get_num_vectors(vec1, vec2);
	uint8_t noVectors = 0;


	switch (num_vectors) {
	case 0:
		/*if(first_call){
			no_line_time = hrt_absolute_time();
			first_call = false;
		}else{
			time_diff = hrt_elapsed_time_atomic(&no_line_time);
			control.steer = 0.0f;
			if(time_diff > 10000){
				// Stopping if no vector is available
				control.steer = 0.0f;
				control.speed = SPEED_STOP;
			}
		}*/
		noVectors = 1;
		break;

	case 2:
		//first_call = true;

		/* Very simple steering angle calculation, get average of the x of top two points and
		   find distance from center of frame */
		/*main_vec.m_x1 = (vec1.m_x1 + vec2.m_x1) / 2;
		control.steer = (float)(main_vec.m_x1 - window_center) / (float)frameWidth;

		control.speed = SPEED_FAST;*/
		break;

	case 1: //default
		//first_call = true;
		/* Following the main vector */
		/*if (vec1.m_x1 > vec1.m_x0) {
			x = (float)(vec1.m_x1 - vec1.m_x0) / (float)frameWidth;
			y = (float)(vec1.m_y1 - vec1.m_y0) / (float)frameHeight;
		} else {
			x = (float)(vec1.m_x0 - vec1.m_x1) / (float)frameWidth;
			y = (float)(vec1.m_y0 - vec1.m_y1) / (float)frameHeight;
		}
		if(vec1.m_x0 != vec1.m_x1){
			control.steer = (-1) * x / y; // Gradient of the main vector
			control.speed = SPEED_NORMAL;
		}else{
			control.steer = 0.0;
			control.speed = SPEED_SLOW;
		}*/
		break;
	}

	p_vec1_x0 = vec1.m_x0;
	p_vec1_x1 = vec1.m_x1;
	p_vec1_y0 = vec1.m_y0;
	p_vec1_y1 = vec1.m_y1;

	p_vec2_x0 = vec2.m_x0;
	p_vec2_x1 = vec2.m_x1;
	p_vec2_y0 = vec2.m_y0;
	p_vec2_y1 = vec2.m_y1;

	float steeringSensitivity = 1;
	control.steer = pente * steeringSensitivity;
	float angle = float(atan(control.steer)) / 1.5708f;
	control.steer = angle * 1.3f;


	double steering_value = control.steer;

	float sign = 0;

	if (control.steer > 0) {
		sign = 1;

	} else if (control.steer < 0) {
		sign = -1;
	}


	double pid1 = pid_calculate(&PID, 0.0f, angle, 0.0f, (float)diff);
	double pid2 = pid_calculate(&PID2, 0.0f, angle, 0.0f, (float)diff);

	if(angle < 0.4f) {
		control.steer = pid1;
	}
	else {
		control.steer = pid2;
		control.steer = sign;
	}

	//control.steer = pid2;

	//control.steer = pid_calculate(&PID, 0.0f, angle, 0.0f, (float)diff);

	printed_value = steering_value;
	printed_value = control.steer;

	//float threshold = 0.0f;
	/*float sign = 0;

	if (control.steer > 0) {
		sign = 1;

	} else if (control.steer < 0) {
		sign = -1;
	}

	control.steer = std::abs(control.steer);
	control.steer = customPower(control.steer);*/
	//control.steer = pow(control.steer, 1.0f);
	//control.steer *= sign;
	//control.steer *= control.steer * sign;
	//control.steer *= control.steer * sign;

	/*if (control.steer < threshold && control.steer > -threshold) {
		control.steer = 0;

	} else if (control.steer > 1) {
		control.steer = 1;

	} else if (control.steer < -1) {
		control.steer = -1;
	}*/

	if (control.steer >= 0.8f || control.steer <= -0.8f) {
		control.speed = 0.8f;

	} else {
		control.speed = 1.0f;
	}

	if (noVectors == 1) {
		control.speed = 0.0f;
	}



	return control;
}

void onInitialize()
{
	if (initialized == 1) { return; }

	//const int numValues = 100;
	//double staticArray[101] = {0.001f, 0.004f, 0.007f, 0.011f, 0.015f, 0.019f, 0.024f, 0.029f, 0.034f, 0.039f, 0.045f, 0.051f, 0.057f, 0.063f, 0.070f, 0.076f, 0.083f, 0.090f, 0.097f, 0.105f, 0.112f, 0.120f, 0.127f, 0.135f, 0.143f, 0.151f, 0.159f, 0.168f, 0.176f, 0.185f, 0.194f, 0.202f, 0.211f, 0.220f, 0.229f, 0.239f, 0.248f, 0.258f, 0.267f, 0.277f, 0.287f, 0.296f, 0.306f, 0.316f, 0.326f, 0.337f, 0.347f, 0.357f, 0.368f, 0.378f, 0.389f, 0.400f, 0.411f, 0.422f, 0.433f, 0.444f, 0.455f, 0.466f, 0.477f, 0.489f, 0.500f, 0.512f, 0.523f, 0.535f, 0.547f, 0.558f, 0.570f, 0.582f, 0.594f, 0.606f, 0.619f, 0.631f, 0.643f, 0.656f, 0.668f, 0.680f, 0.693f, 0.706f, 0.718f, 0.731f, 0.744f, 0.757f, 0.770f, 0.783f, 0.796f, 0.809f, 0.822f, 0.836f, 0.849f, 0.862f, 0.876f, 0.889f, 0.903f, 0.917f, 0.930f, 0.944f, 0.958f, 0.972f, 0.986f, 1.0f};
	//const double stepSize = 1.0 / (numValues - 1); // Step size to cover range from 0 to 1

	//for (int i = 0; i < 101; ++i) {
		/*double x = i * stepSize;
		double value = std::pow(x, 1.4);*/
		//value = 1;
	//	powerLookupTable[i] = staticArray[i];
	//}

	initialized = 1;

}

	double staticArray[101] = {0, 0.001f, 0.004f, 0.007f, 0.011f, 0.015f, 0.019f, 0.024f, 0.029f, 0.034f, 0.039f, 0.045f, 0.051f, 0.057f, 0.063f, 0.070f, 0.076f, 0.083f, 0.090f, 0.097f, 0.105f, 0.112f, 0.120f, 0.127f, 0.135f, 0.143f, 0.151f, 0.159f, 0.168f, 0.176f, 0.185f, 0.194f, 0.202f, 0.211f, 0.220f, 0.229f, 0.239f, 0.248f, 0.258f, 0.267f, 0.277f, 0.287f, 0.296f, 0.306f, 0.316f, 0.326f, 0.337f, 0.347f, 0.357f, 0.368f, 0.378f, 0.389f, 0.400f, 0.411f, 0.422f, 0.433f, 0.444f, 0.455f, 0.466f, 0.477f, 0.489f, 0.500f, 0.512f, 0.523f, 0.535f, 0.547f, 0.558f, 0.570f, 0.582f, 0.594f, 0.606f, 0.619f, 0.631f, 0.643f, 0.656f, 0.668f, 0.680f, 0.693f, 0.706f, 0.718f, 0.731f, 0.744f, 0.757f, 0.770f, 0.783f, 0.796f, 0.809f, 0.822f, 0.836f, 0.849f, 0.862f, 0.876f, 0.889f, 0.903f, 0.917f, 0.930f, 0.944f, 0.958f, 0.972f, 0.986f, 1.0f};


double customPower(double x)
{
	return staticArray[static_cast<int>(floor(x * 100))];
}
