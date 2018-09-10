/*! \file configuration_vehicle_hardware.h
 *
 *  Created on: Jan 28, 2018
 *      Author: saul
 */

#ifndef HEADERS_CONFIGURATION_VEHICLE_HARDWARE_H_
#define HEADERS_CONFIGURATION_VEHICLE_HARDWARE_H_

//------------SAFETY------------//
const int MAX_TIME_WITHOUT_REACTIVE_MILLIS = 200;

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
const float SAMPLING_HERTZ_SPEED = 10; //2.0; //Hz
const float SENSOR_HALL_BASE_NOISE_PER_HERTZ = 0.052 * 2.0;

//------------STEERING-----------//
const float MAX_STEERING_VELOCITY = 5.0; //degrees per second
const float MIN_PWM_TO_ACTUATE_MOTOR = 0.0;

const float SAMPLING_HERTZ_STEERING = 20.0;

const float STEERING_CALIBRATION_OFFSET = 0.76;
const float ABS_MAX_RIGHT_ANGLE_DEG = 24.45 + STEERING_CALIBRATION_OFFSET; //Experimentally measured
const float ABS_MAX_LEFT_ANGLE_DEG = 27.48 - STEERING_CALIBRATION_OFFSET; //Experimentally measured

const float ABS_MAX_STEERING_ANGLE_DEG = 30.0; // To allow observing the mechanical limits

const float STEERING_REDUCTION_RATIO = 60.0;
const float STEERING_ENCODER_BASE_RESOLUTION = 1024.0; //

const float PULSES_TO_DEG = 360.0 / (STEERING_ENCODER_BASE_RESOLUTION * STEERING_REDUCTION_RATIO);

const float STEERING_SENSOR_BASE_NOISE_PER_HERTZ = PULSES_TO_DEG;

const int ABS_MAX_STEERING_MOTOR_PWM = 255;

//------------SPEED-----------//
const float ABS_MAX_SPEED_METERS_SECOND = 1.3; // aprox 5 Km/h

const float SPEED_ENCODER_PULSES_PER_REV = 24;
const float SPEED_MOTOR_REDUCTION = 1.0;

const float METERS_PER_PULSE = M_PI * REAR_WHEEL_DIAM_METERS / (SPEED_ENCODER_PULSES_PER_REV * SPEED_MOTOR_REDUCTION);

const float ABS_MAX_SPEED_VOLTS = 4.9;
const float MIN_VOLTS_TO_RELEASE_BRAKE = 0.5;
const float MIN_VOLTS_TO_ACTUATE_MOTOR = 0.0;

const float MIN_SETPOINT_TO_USE_PID = 0.05;

const int MAX_TIME_ZERO_VOLTS_TO_BRAKE = 2000; //millis

//------------PID AND EKF --------//
const bool SATURATE_ACCEL_MAX = true;

const float STEERING_KP = 1.0;
const float STEERING_KI = 3.0;
const float STEERING_KD = 0.0;

const float SPEED_KP = 0.7;
const float SPEED_KI = 1.0;
const float SPEED_KD = 0.0;

const float IMPOSSIBLE_PID_GAIN = -1.0;
const float NORMALIZING_PID_MAX_VALUE = 1.0;
const float NORMALIZING_PID_MIN_VALUE = -1.0;

const float EKF_Q_COVARIANCE = 0.007;
const float EKF_R_COVARIANCE = (SENSOR_HALL_BASE_NOISE_PER_HERTZ * SAMPLING_HERTZ_SPEED)
    * (SENSOR_HALL_BASE_NOISE_PER_HERTZ * SAMPLING_HERTZ_SPEED);

const float SPEED_PREDICTION_GAIN = 0.3;

const float MAX_STEERING_RATE = 50.0; //degrees per second

const float STEERING_ENCODER_R_COVARIANCE = (STEERING_SENSOR_BASE_NOISE_PER_HERTZ * SAMPLING_HERTZ_STEERING)
    * (STEERING_SENSOR_BASE_NOISE_PER_HERTZ * SAMPLING_HERTZ_STEERING);

const float STEERING_LIMIT_SWITCH_COVARIANCE = 2.0 * 2.0; //Uncertainty in our calibration process (degrees)

#endif /* HEADERS_CONFIGURATION_VEHICLE_HARDWARE_H_ */
