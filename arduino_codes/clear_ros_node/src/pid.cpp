#include "../headers/pid.h"
#include "../headers/arduino_ros_interface.h"

PID::PID()
{
  dt_ = 0.1; // to avoid division by zero
  max_ = 0.0;
  min_ = 0.0;

  kp_ = 0.0;
  ki_ = 0.0;
  kd_ = 0.0;

  pre_error_ = 0.0;
  integral_ = 0.0;

  error_ = 0.0;
  p_out_ = 0.0;
  i_out_ = 0.0;
  d_out_ = 0.0;
  derivative_ = 0.0;

  error_code_ = NO_ERROR;
}

void PID::setPIDMaxMinValues(float max, float min)
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

PID::~PID()
{
}

int PID::calculate(float setpoint, float pv, float dt, float& output)
{
  if (dt > 0.0)
  {
    error_code_ = NO_ERROR;
  }
  else
  {
    error_code_ = PID_DT_EQUAL_ZERO;
    output = 0.0;
  }

  if (error_code_ == NO_ERROR)
  {
    // Calculate error
    error_ = setpoint - pv;

    // Proportional term
    p_out_ = kp_ * error_;

    // Integral term
    integral_ += error_ * dt_;
    i_out_ = ki_ * integral_;

    // Derivative term
    derivative_ = (error_ - pre_error_) / dt_;
    d_out_ = kd_ * derivative_;

    // Calculate total output
    output = p_out_ + i_out_ + d_out_;

    // Restrict to max/min
    if (output > max_)
    {
      output = max_;
    }

    if (output < min_)
    {
      output = min_;
    }

    // Save error to previous error
    pre_error_ = error_;
  }

  return (error_code_);
}
