/*
 * configuration_vehicle_hardware.h
 *
 *  Created on: Jan 28, 2018
 *      Author: saul
 */

#ifndef HEADERS_CONFIGURATION_VEHICLE_HARDWARE_H_
#define HEADERS_CONFIGURATION_VEHICLE_HARDWARE_H_

//------------VEHICLE-----------//
const float WHEELBASE_METERS = 1.05;
const float MIN_CURVATURE_RADIUS_METERS = 4.0;

const float LENGTH_METERS = 1.672;
const float WIDTH_METERS = 0.797;

const float REAR_WHEEL_DIAM_METERS = 0.395; // Manufacturer 0.406
const float FRONT_WHEEL_DIAM_METERS = 0.310; // Manufacturer 0.305


//! Actuators standard values, used for homing during reset state
#define SPEED_ZERO                     0.0
#define STEERING_CENTERED              0.0

const float IMPOSSIBLE_PID_GAIN = -1.0;
const bool  REMOTE_CONTROL_USE_PID = true;

//! Sampling time encoders using I2C
const unsigned int  SAMPLING_TIME_TEENSY = 100; //10Hz

//------------STEERING-----------//
const float ABS_MAX_RIGHT_ANGLE_DEG = 26.0; //Experimentally measured
const float ABS_MAX_LEFT_ANGLE_DEG = 24.0; //Experimentally measured
const float ABS_MAX_STEERING_ANGLE_DEG = 20.0; // To avoid the mechanical limits

const float ABS_MAX_STEERING_SWEEP_DEG = 50.0; //ABS_MAX_RIGHT_ANGLE_DEG+ABS_MAX_LEFT_ANGLE_DEG

const int ABS_MAX_STEERING_PULSES = 9600; //Experimentally measured with constant PWM (speed);

const float PULSES_TO_DEG = 0.005265;  //ABS_MAX_STEERING_ANGLE_DEG/ABS_MAX_STEERING_PULSES

const float PULSES_TO_CENTER_FROM_RIGHT = 4938; //ABS_MAX_RIGHT_ANGLE_DEG/PULSES_TO_DEG

const int ABS_MAX_STEERING_MOTOR_PWM = 100;
const int ABS_MOTOR_PWM_FOR_CALIBRATION = 70;
const int ABS_MOTOR_PWM_FOR_FIND_ZERO_POS = 50;
const int TOLERANCE_PULSES_FIND_ZERO_POS = 20;


//------------SPEED-----------//
const float ABS_MAX_SPEED_METERS_SECOND = 1.4; // aprox 5 Km/h
const float SPEED_ENCODER_PULSES_PER_REV = 24;
const float SPEED_MOTOR_REDUCTION = 1.0;

const float METERS_PER_PULSE = 0.0517; //PI*REAR_WHEEL_DIAM_METERS/(SPEED_ENCODER_PULSES_PER_REV*SPEED_MOTOR_REDUCTION)

const float ABS_MAX_SPEED_VOLTS = 4.9;
const float MIN_VOLTS_TO_RELEASE_BRAKE = 0.5;

const int MAX_TIME_ZERO_VOLTS_TO_BRAKE = 5000; //millis

//------------PID GAINS --------//

const float STEERING_KP = 10;
const float STEERING_KI = 8;
const float STEERING_KD = 0;

const float SPEED_KP = 0;
const float SPEED_KI = 8;
const float SPEED_KD = 0;

#endif /* HEADERS_CONFIGURATION_VEHICLE_HARDWARE_H_ */
