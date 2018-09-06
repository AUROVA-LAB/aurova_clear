/*
 * steering_hardware_interface.cpp
 *
 *  Created on: Nov 17, 2017
 *      Author: saul
 */

#include <Arduino.h>
#include <Wire.h>

#include "hardware_description_constants.h"
#include "steering_hardware_interface.h"
#include "configuration_vehicle_hardware.h"

//Error Vars
const int ERROR_ENCODER_COUNT = 0b0001;
const int ERROR_LIMIT_SWITCH = 0b0010;
const int ERROR_DIRECTION = 0b0100;

bool error_encoder_count = false;
bool error_limit_switch = false;
bool error_direction = false;

bool left_limit_flag = false;
bool right_limit_flag = false;
bool limit_switch_flag = false;

SteeringHardwareInterface::SteeringHardwareInterface()

{
  pin_ina_ = PIN_INA;
  pin_inb_ = PIN_INB;
  pin_pwm_ = PIN_PWM;
  pin_limit_switch_left_ = PIN_LSL;
  pin_limit_switch_right_ = PIN_LSR;
  pin_int_limit_switch_ = PIN_INT_LS;

  steering_encoder_ = new EncoderHardwareInterface();

  for (int i = 0; i < 2; i++)
    measures_[i] = 0;

  //Initialized pins like I/O
  pinMode(pin_ina_, OUTPUT);
  pinMode(pin_inb_, OUTPUT);
  pinMode(pin_pwm_, OUTPUT);
  pinMode(pin_limit_switch_left_, INPUT);
  pinMode(pin_limit_switch_right_, INPUT);

  //ISR
  attachInterrupt(digitalPinToInterrupt(pin_int_limit_switch_), doLimitSwitch, RISING);
  //Enable ISRs
  interrupts();
}

SteeringHardwareInterface::~SteeringHardwareInterface()
{
  delete steering_encoder_;
}

void SteeringHardwareInterface::steeringMotor(int desired_pwm)
{
  if (desired_pwm < 0 && desired_pwm >= -1 * ABS_MAX_STEERING_MOTOR_PWM && digitalRead(PIN_LSR) == HIGH)
  {
    digitalWrite(pin_ina_, HIGH);
    digitalWrite(pin_inb_, LOW);
  }
  else if (desired_pwm > 0 && desired_pwm <= ABS_MAX_STEERING_MOTOR_PWM && digitalRead(PIN_LSL) == HIGH)
  {
    digitalWrite(pin_ina_, LOW);
    digitalWrite(pin_inb_, HIGH);
  }
  else if (desired_pwm == 0)
  {
    digitalWrite(pin_ina_, LOW);
    digitalWrite(pin_inb_, LOW);
    desired_pwm = 0;
  }
  else
  {
    error_direction = true;
    digitalWrite(pin_ina_, LOW);
    digitalWrite(pin_inb_, LOW);
    desired_pwm = 0;
  }

  analogWrite(pin_pwm_, fabs(desired_pwm)); //Direction can't be negative

}

void SteeringHardwareInterface::doLimitSwitch(void)
{
  limit_switch_flag = true;
}

float* SteeringHardwareInterface::getSteeringMeasures(void)
{
  steering_encoder_->encoderRead(0, measures_[0]); //pulses
  steering_encoder_->encoderRead(1, measures_[1]); //pulses/s
  return measures_;
}

int SteeringHardwareInterface::getSteeringError(void)
{
  int error_code = 0;

  if (error_encoder_count)
  {
    error_code += ERROR_ENCODER_COUNT;
    error_encoder_count = false; //clean flag
  }

  if (error_direction)
  {
    error_code += ERROR_DIRECTION;
    error_direction = false; //clean flag
  }

  return error_code;
}

int SteeringHardwareInterface::readLimitSwitches(void)
{
  int result = 0;
  if (limit_switch_flag)
  {
    limit_switch_flag = false;
    result = 1;
  }
  return (result);
}
