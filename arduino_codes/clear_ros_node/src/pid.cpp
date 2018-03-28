/*
 * pid.cpp
 *
 *  Created on: Mar 28, 2018
 *      Author: idelpino
 */



#include "../headers/pid.h"
#include "../headers/arduino_ros_interface.h"
#include "Arduino.h"

PID::PID(float *input, float* output, float* setpoint, float kp, float ki, float kd)
{
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;

	proportional_ = 0.0;
	integral_ = 0.0;
	derivative_ = 0.0;

	error_ = 0.0;
	previous_error_ = 0.0;

	input_ = input;
	output_ = output;
	set_point_ = setpoint;


	dt_ = 0.1;
	last_time_ = 0;
	current_time_ = 0;

	max_ = 0.0;
	min_ = 0.0;
}

PID::~PID()
{}

void PID::setOutputLimits(float min, float max)
{
  max_ = max;
  min_ = min;
}

void PID::setPIDGains(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PID::getPIDGains(float& kp, float& ki, float& kd)
{
  kp = kp_;
  ki = ki_;
  kd = kd_;
}

void PID::resetPID(void)
{
	*output_ = 0.0;
	dt_ = 0.1;
	integral_ = 0.0;
	derivative_ = 0.0;
}


void PID::computePID()
{

	current_time_ = micros();
	dt_ = (current_time_ - last_time_) / 1e6;

	float input = *input_;
	float set_point = *set_point_;
	float output = 0.0;

    // Restrict to max/min
    if (input > max_)
    	input = max_;
    else if (input < min_)
    	input = min_;


	error_ = set_point - input;

	proportional_ = kp_ * error_;
	integral_ += ki_*(error_ + previous_error_)*dt_ / 2;
	derivative_ = kd_*(error_ - previous_error_)  / dt_;

	output = proportional_ + integral_ + derivative_;


    // Restrict to max/min
    if (output > max_)
    	output = max_;
    else if (output < min_)
    	output = min_;


    *output_ = output;

    previous_error_ = error_;
    last_time_ = current_time_;

}
