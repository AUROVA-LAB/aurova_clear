/*
 * sdkf.cpp
 *
 *  Created on: 14 Mar 2018
 *      Author: idelpino
 */

#include "../headers/sdkf.h"
#include "../headers/configuration_vehicle_hardware.h"
#include "Arduino.h"


sdkf::sdkf(float A, float B, float P0, float Q, float R) {
	// TODO Auto-generated constructor stub
	A_ = A;
	B_ = B;

	covariance_ = P0; //initialize covariance
	process_noise_ = Q;
	sensor_noise_ = R;

	vel_ = 0.0;
	vel_k_minus_one_ = 0.0;

	volts_k_minus_one_ = 0.0;

	dt_ = -1.0;
	t_k_ = 0.0;
	t_k_minus_one_ = 0.0;

	kalman_gain_ = 0.0;
	innovation_ = 0.0;
	innovation_covariance_ = 0.0;

	last_time_prediction_ = micros();
	last_time_correction_ = micros();
}

sdkf::~sdkf() {
	// TODO Auto-generated destructor stub
}

void sdkf::resetSDKF(void)
{
	vel_ = 0.0;
	vel_k_minus_one_ = 0.0;

	volts_k_minus_one_ = 0.0;

	dt_ = -1.0;
	t_k_ = 0.0;
	t_k_minus_one_ = 0.0;

	kalman_gain_ = 0.0;
	innovation_ = 0.0;
	innovation_covariance_ = 0.0;

	last_time_prediction_ = micros();
	last_time_correction_ = micros();
}

float outlierRejection(float current_vel, float last_vel, double dt)
{

	//Saturating the estimate measure

	float filtered_vel = 0.0;

	if((current_vel - last_vel)/dt > ABS_MAX_ACCEL)
	{
		filtered_vel = ABS_MAX_ACCEL*dt + last_vel;
	}
	else if ((current_vel - last_vel)/dt < -1 * ABS_MAX_ACCEL)
	{
		filtered_vel = -ABS_MAX_ACCEL*dt + last_vel;
	}

	else
		filtered_vel = current_vel;

	if(filtered_vel > ABS_MAX_SPEED_METERS_SECOND)
		filtered_vel = ABS_MAX_SPEED_METERS_SECOND;

	else if(filtered_vel < -1 * ABS_MAX_SPEED_METERS_SECOND)
		filtered_vel = -1 * ABS_MAX_SPEED_METERS_SECOND;

	return(filtered_vel);
}

void sdkf::make_prediction(float volts, float& vel, float& covariance)
{
	unsigned long int now = micros();
	unsigned long int timeChange = (now - last_time_prediction_);
	last_time_prediction_ = now;

	double freq = 1000.0;

	if(timeChange > 0) freq = 1000000.0 / (double)(timeChange);

	float volts_diff = (volts - volts_k_minus_one_);

	//Saturate the max accel
	if(SATURATE_ACCEL_MAX && B_ * volts_diff * freq > ABS_MAX_ACCEL)
		vel_ = A_ * vel_ + ABS_MAX_ACCEL / freq;

	else if (SATURATE_ACCEL_MAX && B_ * volts_diff * freq < -1 * ABS_MAX_ACCEL)
		vel_ = A_ * vel_ - ABS_MAX_ACCEL / freq;

	else
		vel_ = A_ * vel_ + B_ * volts_diff; // dt gets cancelled


	covariance_ = covariance_ + process_noise_ / freq;


	covariance = covariance_;
	vel = vel_;

	volts_k_minus_one_ = volts;

}

void sdkf::make_correction(float measured_speed, float& vel, float& covariance)
{
	unsigned long int now = micros();
	unsigned long int timeChange = (now - last_time_correction_);
	last_time_correction_ = now;

	double freq = 1000.0;

	if(timeChange > 0) freq = 1000000.0 / (timeChange);

	double std_dev = sqrt(covariance);


	innovation_ = measured_speed - vel_;

	if(fabs(innovation_) < 3*std_dev) // outlier rejection
	{
		innovation_covariance_ = covariance_ + sensor_noise_ / freq;
		kalman_gain_ = covariance_ / innovation_covariance_;

		vel_ = vel_ + kalman_gain_ * innovation_;

		covariance_ = covariance_ - kalman_gain_ * innovation_covariance_ * kalman_gain_;

		vel = vel_;
		covariance = covariance_;
	}
}

