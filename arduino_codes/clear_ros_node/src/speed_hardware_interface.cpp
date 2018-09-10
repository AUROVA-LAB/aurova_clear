/*! \file speed_hardware_interface.cpp
 *
 *  Created on: 6 Nov 2017
 *      Author: idelpino
 */
#include <Wire.h>
#include "speed_hardware_interface.h"
#include "hardware_description_constants.h"
#include "configuration_vehicle_hardware.h"

const unsigned int SAMPLING_TIME_SPEED = (1.0 / SAMPLING_HERTZ_SPEED) * 1000; //ms

int digital = 0;

elapsedMillis refreshDac = 0;
unsigned int interval = SAMPLING_TIME_SPEED * 4.25;

float voltage_ant = ABS_MAX_SPEED_VOLTS + 1;

SpeedHardwareInterface::SpeedHardwareInterface(int ch1, int ch2)
{
  dac_.begin(I2C_SLAVEADDRADC); //Initialize the dac
  ch1_ = ch1;
  ch2_ = ch2;

  speed_encoder_ = new EncoderHardwareInterface();

  for (int i = 0; i < 3; i++)
    measures_[i] = 0;

  pinMode(this->ch1_, OUTPUT); //ch1 and ch2 like OUTPUT pins
  pinMode(this->ch2_, OUTPUT);

  flag_forward_ = true;
}

SpeedHardwareInterface::~SpeedHardwareInterface()
{
  delete speed_encoder_;
}

void SpeedHardwareInterface::actuateMotor(float voltage)
{
  /*!
   * The maximun voltage that the vehicle suports to move is |5|V
   */
  if (voltage < -ABS_MAX_SPEED_VOLTS)
    voltage = -ABS_MAX_SPEED_VOLTS;
  else if (voltage > ABS_MAX_SPEED_VOLTS)
    voltage = ABS_MAX_SPEED_VOLTS;

  /*!
   * Transform the voltage in a digital number with the resolution of the dac (12 bits)
   */
  //Move FORWARD
  if (voltage > 0)
  {
    flag_forward_ = true;
    if (voltage != voltage_ant or refreshDac > interval)
    {
      digital = 4096 * (abs(voltage) / 5.0);
      dac_.setVoltage(digital, false);
      refreshDac = 0;
    }

    digitalWrite(this->ch1_, LOW);
    digitalWrite(this->ch2_, HIGH);
  }
  //Move BACKWARD
  else if (voltage < 0)
  {
    flag_forward_ = false;
    if (voltage != voltage_ant or refreshDac > interval)
    {
      digital = 4096 * (abs(voltage) / 5.0);
      dac_.setVoltage(digital, false);
      refreshDac = 0;
    }

    digitalWrite(this->ch1_, HIGH);
    digitalWrite(this->ch2_, LOW);
  }
  //STOP
  else
  {
    flag_forward_ = true;
    if (voltage != voltage_ant or refreshDac > interval)
    {
      dac_.setVoltage(0, false);
      refreshDac = 0;
    }
    digitalWrite(this->ch1_, HIGH);
    digitalWrite(this->ch2_, HIGH);
  }

  voltage_ant = voltage;
}

float* SpeedHardwareInterface::getSpeedMeasures(void)
{
  speed_encoder_->encoderRead(2, measures_[0]); //vel
  speed_encoder_->encoderRead(3, measures_[1]); //acc
  speed_encoder_->encoderRead(4, measures_[2]); //jerk

  return measures_;
}

bool SpeedHardwareInterface::getFlagForward(void)
{
  return (flag_forward_);
}

