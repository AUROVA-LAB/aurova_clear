/*! \file arduino_ros_interface.h
 *
 *  \brief Definitions of constant values used in the arduino ros interface
 *
 * This file contains the definitions of constant values to code operational
 * modes, errors, warnings, etc...
 */
#ifndef ARDUINO_ROS_INTERFACE_H
#define ARDUINO_ROS_INTERFACE_H

#define COMMUNICATION_REFRESH_TIME_IN_MILLIS 100 //10 Hz

#define BUILT_IN_ARDUINO_LED_PORT 13

/*! How many status variables will be send to the ROS topic,
 * at this moment (please keep this comment up to date):
 * - Operation mode
 * - Error code
 * - Ros interface warning code
 * - Robot warning code
 * - Verbose level
 */
#define NUM_OF_CLEAR_STATUS_VARIABLES   5
#define NUM_OF_CONTROLLED_MOTORS        2
#define NUM_OF_PID_GAINS                3
#define NUM_OF_STEERING_LIMIT_SWITCHES  2

/*! Verbose levels to adjust the amount of information sent by the
 *  Arduino to avoid exceeding the communication bandwidth
 */
#define DEBUG_VERBOSE_LEVEL    2
#define REGULAR_VERBOSE_LEVEL  1
#define NO_VERBOSE             0

//! State codes referring to the operational mode and safety state
#define EMERGENCY_STOP          0
#define REMOTE_CONTROL          1
#define ROS_CONTROL             2
#define REMOTE_CONTROL_NOT_SAFE 3

//! Error codes
#define NO_ERROR                            0
#define REMOTE_CONTROL_LOST                 1
#define ON_BOARD_EMERGENCY_SWITCH_ACTIVATED 2
#define RC_EMERGENCY_SWITCH_ACTIVATED       3
#define REACTIVE_SAFETY_TOPIC_NOT_RECEIVED  4
#define ROS_CONTROL_TOPIC_NOT_RECEIVED      5

//! Warning codes
#define NO_WARNING                                           0
#define RECEIVING_ROS_CONTROLS_WHILE_NOT_BEING_IN_ROS_MODE   1 //Callbacks are activated but not listened
#define LIMITING_SPEED_BY_REACTIVE_SAFETY_LAYER              2

#endif
