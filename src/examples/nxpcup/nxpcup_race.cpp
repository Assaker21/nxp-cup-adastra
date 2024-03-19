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

uint8_t get_num_vectors(Vector &vec1, Vector &vec2) {
	uint8_t numVectors = 0;
	if(!(vec1.m_x0 == 0 && vec1.m_x1 == 0 && vec1.m_y0 == 0 && vec1.m_y1 == 0)) numVectors++;
	if(!(vec2.m_x0 == 0 && vec2.m_x1 == 0 && vec2.m_y0 == 0 && vec2.m_y1 == 0)) numVectors++;
	return numVectors;
}

Vector copy_vectors(const pixy_vector_s &pixy, uint8_t num) {
	Vector vec;
	if(num == 1) {
		vec.m_x0 = pixy.m0_x0;
		vec.m_x1 = pixy.m0_x1;
		vec.m_y0 = pixy.m0_y0;
		vec.m_y1 = pixy.m0_y1;
	}
	if(num == 2) {
		vec.m_x0 = pixy.m1_x0;
		vec.m_x1 = pixy.m1_x1;
		vec.m_y0 = pixy.m1_y0;
		vec.m_y1 = pixy.m1_y1;
	}
	return vec;
}

roverControl raceTrack(const pixy_vector_s &pixy, PID_t &PID)
{
	Vector main_vec;
	Vector vec1 = copy_vectors(pixy, 1);
	Vector vec2 = copy_vectors(pixy, 2);
	float pente = pixy.pente;
	uint8_t frameWidth = 79;
	uint8_t frameHeight = 52;
	int16_t window_center = (frameWidth / 2);
	roverControl control{};
	float x, y;					 // calc gradient and position of main vector
	//static hrt_abstime no_line_time = 0;		// time variable for time since no line detected
	//hrt_abstime time_diff = 0;
	//static bool first_call = true;

	static hrt_abstime old_t = hrt_absolute_time();
	hrt_abstime t = hrt_absolute_time();
	double diff = difftime(t, old_t);
	// milliseconds
	diff = diff / 1000;

	uint8_t num_vectors = get_num_vectors(vec1, vec2);


	switch (num_vectors) {
	/*case 0:
		if(first_call){
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
		}
		break;*/

	case 2:
		//first_call = true;

		/* Very simple steering angle calculation, get average of the x of top two points and
		   find distance from center of frame */
		main_vec.m_x1 = (vec1.m_x1 + vec2.m_x1) / 2;
		control.steer = (float)(main_vec.m_x1 - window_center) / (float)frameWidth;

		control.speed = SPEED_FAST;
		break;

	case 1: //default
		//first_call = true;
		/* Following the main vector */
		if (vec1.m_x1 > vec1.m_x0) {
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
		}
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

	control.speed = 0.3f; // 0.7f
	float steeringSensitivity = 1; //1
	control.steer = (1 / pente) * steeringSensitivity;
	float angle = float(atan(control.steer)) / 1.5708f;
	control.steer = angle * 1.3f;


	double steering_value = control.steer;

	control.steer = pid_calculate(&PID, 0.0f, angle, 0.0f, (float)diff);
	printed_value = steering_value;
	printed_value = control.steer;

	float threshold = 0.0f;
	if(control.steer < threshold && control.steer > -threshold) {
		control.steer = 0;
	}
	else if(control.steer > 1) {
		control.steer = 1;
	}
	else if(control.steer < -1) {
		control.steer = -1;
	}

	if(control.steer >= 0.8f || control.steer <= -0.8f) {
		control.speed = 0.3f;
	}
	else {
		control.speed = 0.3f;
	}
	//control.steer = 1;

	return control;
}
