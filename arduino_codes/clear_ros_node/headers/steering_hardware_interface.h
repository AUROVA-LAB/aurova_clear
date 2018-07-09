/*
 * steering_hardware_interface.h
 *
 *  Created on: Nov 17, 2017
 *      Author: saul
 */

#ifndef HEADERS_STEERING_HARDWARE_INTERFACE_H_
#define HEADERS_STEERING_HARDWARE_INTERFACE_H_


#include "encoder_hardware_interface.h"


class SteeringHardwareInterface;
typedef  SteeringHardwareInterface* SteeringHardwareInterfacePtr;


/**
 * Class SteeringHardwareInterface
 * This class have the methods to control the direction of the autonomous vehicle Blue Barrow
 */
class SteeringHardwareInterface
{
private:
  int pin_ina_; //MOVES Right
  int pin_inb_; //MOVES LEFT
  int pin_pwm_; //SPIN

  int pin_limit_switch_left_;  //1-> DETECTION 0 -> NO DETECTION
  int pin_limit_switch_right_;
  int pin_int_limit_switch_;   //Interruption Pin, both limit switch are connected

  float measures_[2]; // pulses, vel

public:

  EncoderHardwareInterfacePtr steering_encoder_;
	SteeringHardwareInterface();
     ~SteeringHardwareInterface();


    /*!
     * Moves the vehicle steering
     * @param direction is a PWM value
     * 0 < dir < 256: moves RIGHT, with direction speed
     * -256 < dir < 0: moves LEFT, with Abs(direction) speed
     * other case: stop motor
     */
    void steeringMotor(int direction);

    /*!
     * Calibrate the vehicle steering using the limit switch and the encoder
     */
    bool steeringCalibration(void);

    /*!
     * ISR that update the steering limits reading the sensors
     * Any limit switch sensor call this interruption
     */
    static void doLimitSwitch(void);

    /*!
     * Convert the encoder_count in radians
     * using the hardware_descriptions_constants.h, the motor reduction and the pulses per revolution
     */
    float* getSteeringMeasures(void);

    /*!
     * Verify some important variables and report if there are some error
     * Return the binary error code
     */
    int getSteeringError(void);

    int readLimitSwitches(void);

};




#endif /* HEADERS_STEERING_HARDWARE_INTERFACE_H_ */
