#include <Arduino.h>
#include <Wire.h>

#include "hardware_description_constants.h"
#include "steering_hardware_interface.h"
#include "configuration_vehicle_hardware.h"
#include "arduino_ros_interface.h"

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

  // Initialized pins like I/O
  pinMode(pin_ina_, OUTPUT);
  pinMode(pin_inb_, OUTPUT);
  pinMode(pin_pwm_, OUTPUT);
  pinMode(pin_limit_switch_left_, INPUT);
  pinMode(pin_limit_switch_right_, INPUT);

  limit_switch_flag = false;

  // ISR
  attachInterrupt(digitalPinToInterrupt(pin_int_limit_switch_), activateLimitSwitchFlag, RISING);

  // Enable ISRs
  interrupts();

  warning_code_ = NO_WARNING;
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
  }
  else
  {
    if(abs(desired_pwm) > ABS_MAX_STEERING_MOTOR_PWM)
    {
      warning_code_ = STEERING_PWM_OUT_OF_RANGE;
    }
    else
    {
      warning_code_ = PWM_TRYING_TO_EXCEED_STEERING_LIMITS;
    }

    digitalWrite(pin_ina_, LOW);
    digitalWrite(pin_inb_, LOW);
    desired_pwm = 0;
  }

  analogWrite(pin_pwm_, fabs(desired_pwm)); // Direction can't be negative

}

void SteeringHardwareInterface::activateLimitSwitchFlag(void)
{
  limit_switch_flag = true;
}

float* SteeringHardwareInterface::getSteeringMeasures(void)
{
  steering_encoder_->encoderRead(0, measures_[0]); // pulses
  steering_encoder_->encoderRead(1, measures_[1]); // pulses/s
  return measures_;
}

int SteeringHardwareInterface::getSteeringWarningCode(void)
{
  return warning_code_;
}

int SteeringHardwareInterface::readAndResetLimitSwitchFlag(void)
{
  int result = 0;
  if (limit_switch_flag)
  {
    limit_switch_flag = false;
    result = 1;
  }
  return (result);
}
