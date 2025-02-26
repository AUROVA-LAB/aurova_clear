#include "pid.h"
#include "arduino_ros_interface.h"
#include "Arduino.h"

PID::PID(float *input, float* output, float* setpoint, float kp, float ki, float kd, float tolerance_to_output_zero)
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
  last_time_ = micros();
  current_time_ = 0;
  first_time_ = true;

  max_ = 0.0;
  min_ = 0.0;

  tolerance_threshold_to_output_zero_ = tolerance_to_output_zero;
}

PID::~PID()
{
}

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
  last_time_ = micros();
  integral_ = 0.0;
  derivative_ = 0.0;
  proportional_ = 0.0;
  error_ = 0.0;
  previous_error_ = 0.0;
  first_time_ = true;
}

void PID::computePID(float scale_input, float scale_setpoint, float scale_output)
{

  current_time_ = micros();
  dt_ = (current_time_ - last_time_) / 1e6;

  float input = *input_;
  float set_point = *set_point_;
  float output = 0.0;

  error_ = (set_point / scale_setpoint) - (input / scale_input);

  proportional_ = kp_ * error_;
  derivative_ = kd_ * (error_ - previous_error_) / dt_;

  integral_ += ki_ * (error_ + previous_error_) * dt_ / 2;

  output = proportional_ + integral_ + derivative_;

  // Saturate output value and integral
  if (output > max_)
  {
    output = max_;
    integral_ -= ki_ * (error_ + previous_error_) * dt_ / 2;
  }
  else if (output < min_)
  {
    output = min_;
    integral_ -= ki_ * (error_ + previous_error_) * dt_ / 2;
  }

  // Scale output value
  if (fabs(output * scale_output) > tolerance_threshold_to_output_zero_)
  {
    *output_ = output * scale_output;
  }
  else
  {
    *output_ = 0.0;
  }

  previous_error_ = error_;
  last_time_ = current_time_;
}
