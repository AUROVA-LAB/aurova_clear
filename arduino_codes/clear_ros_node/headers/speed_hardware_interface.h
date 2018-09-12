/*! \file speed_hardware_interface.h
 *
 *  Created on: 6 Nov 2017
 *      Author: idelpino
 */

#ifndef HEADERS_SPEED_HARDWARE_INTERFACE_H_
#define HEADERS_SPEED_HARDWARE_INTERFACE_H_

#include "Adafruit_MCP4725.h"
#include "encoder_hardware_interface.h"

class SpeedHardwareInterface;
typedef SpeedHardwareInterface* SpeedHardwareInterfacePtr;

/*!
 * \class SpeedHardwareInterface
 * \brief Class to control an speed motor
 */
class SpeedHardwareInterface
{
private:
  int ch1_;
  int ch2_;

  Adafruit_MCP4725 dac_; //DAC that transform a digital number to voltage

  EncoderHardwareInterfacePtr speed_encoder_;

  float measures_[3]; // vel, acc, jerk

  bool flag_forward_;

public:
  /*
   * \brief Class constructor.
   * \param ch1 relay output
   * \param ch2 relay output.
   */
  SpeedHardwareInterface(int ch1, int ch2);
  ~SpeedHardwareInterface();

  /*
   * \brief Apply the voltage to the motor
   *
   * \param voltage
   */
  void actuateMotor(float voltage);

  float* getSpeedMeasures(void);bool getFlagForward(void);

};

#endif /* HEADERS_SPEED_HARDWARE_INTERFACE_H_ */
