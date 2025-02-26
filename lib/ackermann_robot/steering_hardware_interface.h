/*! \file steering_hardware_interface.h
 *
 *  Created on: Nov 17, 2017
 *      Author: saul
 */

#ifndef HEADERS_STEERING_HARDWARE_INTERFACE_H_
#define HEADERS_STEERING_HARDWARE_INTERFACE_H_

#include "encoder_hardware_interface.h"

class SteeringHardwareInterface;
typedef SteeringHardwareInterface* SteeringHardwareInterfacePtr;

/*!
 * \class SteeringHardwareInterface
 * \brief Class for steering control
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

  int warning_code_;

public:

  EncoderHardwareInterfacePtr steering_encoder_;
  SteeringHardwareInterface();
  ~SteeringHardwareInterface();

  /*!
   * \brief Moves the vehicle steering
   *
   * It checks the limit switches pins to allow
   * or deny the movement.
   *
   * \param desired_pwm is a PWM value with sign
   * 0 < desired_pwm < 256: moves RIGHT,
   * -256 < desired_pwm < 0: moves LEFT,
   * other case: stop motor
   */
  void steeringMotor(int desired_pwm);

  /*!
   * \brief ISR that activates a flag that indicates
   * that any of the two limit switches has been reached
   * It is only used to fire an EKF observation, not to
   * stop the motors if reached.
   */
  static void activateLimitSwitchFlag(void);

  /*!
   * \brief Convert the encoder_count in radians
   * using the hardware_descriptions_constants.h,
   * the motor reduction and the pulses per revolution
   */
  float* getSteeringMeasures(void);

  /*!
   * \brief Verify some important variables and report if there are some error
   * Return the binary error code
   *
   * TODO: Integrate this warnings in the ROS interface
   */
  int getSteeringWarningCode(void);

  /*!
   * \brief This read the limit switch flag state
   * and resets the flag. This makes that only one
   * EKF correction step is applied when a limit
   * switch is reached. TIP: In case of problematic
   * EKF corrections using limit switches, one way to
   * improve the performance could be to launch the EKF
   * correction step within the interruption, so this
   * functions would be placed in the ackermann_robot
   * class.
   */
  int readAndResetLimitSwitchFlag(void);

};

#endif /* HEADERS_STEERING_HARDWARE_INTERFACE_H_ */
