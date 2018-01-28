/*
 * speed_hardware_interface.h
 *
 *  Created on: 6 Nov 2017
 *      Author: idelpino
 */

#ifndef HEADERS_SPEED_HARDWARE_INTERFACE_H_
#define HEADERS_SPEED_HARDWARE_INTERFACE_H_

#include "Adafruit_MCP4725.h"
#include "encoder_hardware_interface.h"

class SpeedHardwareInterface;
typedef  SpeedHardwareInterface* SpeedHardwareInterfacePtr;

/**
 * Class SpeedHardwareInterface
 * This class have the methods to control the velocity of the autonomous vehicle Blue Barrow
 */
class SpeedHardwareInterface
{
private:
  int ch1_;
  int ch2_;

  Adafruit_MCP4725 dac_; //DAC that transform a digital number to voltage

  EncoderHardwareInterfacePtr speed_encoder_;

  float measures_[3]; // vel, acc, jerk

public:
  /*
   * Class constructor.
   * ch1 and ch2 are the channels of the relay to control the velocity.
   */
  SpeedHardwareInterface(int ch1, int ch2);
    ~SpeedHardwareInterface();

    /*
     * The method that moves the vehicle
     */
   void actuateMotor(float voltage);

   float* getSpeedMeasures(void);

};



#endif /* HEADERS_SPEED_HARDWARE_INTERFACE_H_ */
