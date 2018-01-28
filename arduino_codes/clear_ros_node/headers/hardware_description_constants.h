/*
 * hardware_description_constants.h
 *
 *  Created on: 7 Nov 2017
 *      Author: idelpino
 */

#ifndef HEADERS_HARDWARE_DESCRIPTION_CONSTANTS_H_
#define HEADERS_HARDWARE_DESCRIPTION_CONSTANTS_H_

#define MAX_SPEED_CONTROLLER_OUTPUT    0.0
#define MIN_SPEED_CONTROLLER_OUTPUT    0.0

#define MAX_STEERING_CONTROLLER_OUTPUT 0.0
#define MIN_STEERING_CONTROLLER_OUTPUT 0.0

//! I2C COMUNICATIONS SLAVES
#define I2C_SLAVEADDRTEENSY 		   0x33 // Slave addres for Teensy
#define I2C_SLAVEADDRADC 			   0x62 // Slave addres for Teensy

//! Arduino Pins

//Steering
#define PIN_INA							12
#define PIN_INB							10
#define PIN_PWM							9
#define PIN_ERRA						8
#define PIN_ERRB						11
#define PIN_LSL							26
#define PIN_LSR							27
#define PIN_INT_LS						18

//Speed
#define PIN_CH1							48
#define PIN_CH2							49

//Hardware
#define EMERGENCY_SWITCH				29


#endif /* HEADERS_HARDWARE_DESCRIPTION_CONSTANTS_H_ */
