/*
 * configuration_vehicle_hardware.h
 *
 *  Created on: Jan 28, 2018
 *      Author: saul
 */

#ifndef HEADERS_CONFIGURATION_VEHICLE_HARDWARE_H_
#define HEADERS_CONFIGURATION_VEHICLE_HARDWARE_H_

#include <math.h>

//------------VEHICLE-----------//
const float WHEELBASE_METERS = 1.05;
const float MIN_CURVATURE_RADIUS_METERS = 4.0;

const float LENGTH_METERS = 1.672;
const float WIDTH_METERS = 0.797;
const float WIDTH_CENTER_WHEELS_METERS = 0.62;

const float REAR_WHEEL_DIAM_METERS = 0.395 * 0.92; // Manufacturer 0.406
const float FRONT_WHEEL_DIAM_METERS = 0.310; // Manufacturer 0.305


//! Actuators standard values, used for homing during reset state
#define SPEED_ZERO                     0.0
#define STEERING_CENTERED              0.0


//! Sampling time encoders using I2C
const float  SAMPLING_HERTZ_SPEED = 1.0; //Hz
const float  SAMPLING_HERTZ_STEERING = 10.0; //Hz

const float SENSOR_BASE_NOISE_PER_HERTZ = 0.052 * 2.0;

//------------STEERING-----------//
const float ABS_MAX_RIGHT_ANGLE_DEG = 26.0; //Experimentally measured
const float ABS_MAX_LEFT_ANGLE_DEG = 24.0; //Experimentally measured
const float ABS_MAX_STEERING_ANGLE_DEG = 23.0; // To avoid the mechanical limits

const float ABS_MAX_STEERING_SWEEP_DEG = 50.0; //ABS_MAX_RIGHT_ANGLE_DEG+ABS_MAX_LEFT_ANGLE_DEG

const int ABS_MAX_STEERING_PULSES = 9600; //Experimentally measured with constant PWM (speed);

const float PULSES_TO_DEG = 0.005265;  //ABS_MAX_STEERING_ANGLE_DEG/ABS_MAX_STEERING_PULSES

const float PULSES_TO_CENTER_FROM_RIGHT = 4938; //ABS_MAX_RIGHT_ANGLE_DEG/PULSES_TO_DEG

const int ABS_MAX_STEERING_MOTOR_PWM = 100;
const int ABS_MOTOR_PWM_FOR_CALIBRATION = 70;
const int ABS_MOTOR_PWM_FOR_FIND_ZERO_POS = 50;
const int TOLERANCE_PULSES_FIND_ZERO_POS = 20;


//------------SPEED-----------//
const float ABS_MAX_SPEED_METERS_SECOND = 1.8; // aprox 5 Km/h
const float ABS_MAX_ACCEL = 10; // m/s²

const float SPEED_ENCODER_PULSES_PER_REV = 24;
const float SPEED_MOTOR_REDUCTION = 1.0;

const float METERS_PER_PULSE = M_PI*REAR_WHEEL_DIAM_METERS/(SPEED_ENCODER_PULSES_PER_REV*SPEED_MOTOR_REDUCTION);

const float ABS_MAX_SPEED_VOLTS = 4.9;
const float MIN_VOLTS_TO_RELEASE_BRAKE = 0.5;

const float MIN_SETPOINT_TO_USE_PID = 0.05;

const int MAX_TIME_ZERO_VOLTS_TO_BRAKE = 5000; //millis



//------------PID AND SDKF --------//
const bool  REMOTE_CONTROL_USE_PID = true;
const bool  USE_KALMAN_FILTER = true;
const bool  SATURATE_ACCEL_MAX = true;

const float STEERING_KP = 1.0;
const float STEERING_KI = 3.0;
const float STEERING_KD = 0.0;

const float SPEED_KP = 0.7;
const float SPEED_KI = 1.0;
const float SPEED_KD = 0.0;

const float IMPOSSIBLE_PID_GAIN = -1.0;
const float NORMALIZING_PID_MAX_VALUE = 1.0;
const float NORMALIZING_PID_MIN_VALUE = -1.0;

const float KALMAN_A = 1.0;
const float KALMAN_B =  0.3; // G_gato = 0.5  G_llano = 0.3
const float KALMAN_Q_COVARIANCE = 0.005;
const float KALMAN_R_COVARIANCE = (SENSOR_BASE_NOISE_PER_HERTZ * SAMPLING_HERTZ_SPEED)*
							      (SENSOR_BASE_NOISE_PER_HERTZ * SAMPLING_HERTZ_SPEED);

#endif /* HEADERS_CONFIGURATION_VEHICLE_HARDWARE_H_ */
