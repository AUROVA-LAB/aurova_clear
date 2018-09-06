/*! \file vehicle.h
 *  \brief Interface with the arduino functionalities
 *
 *  This class reads the sensors, the remote control commands
 *  and calculates the voltages outputs from desired ackermann
 *  states or RC commands
 *
 *  Created on: 27 Sep 2017
 *      Author: idelpino
 */

#ifndef HEADERS_VEHICLE_H_
#define HEADERS_VEHICLE_H_

#include "arduino_ros_interface.h"
#include "std_msgs/Float32MultiArray.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "DJI_DBUS.h"
#include "speed_hardware_interface.h"
#include "steering_hardware_interface.h"
#include "pid.h"
#include "EKF.h"
#include "nav_msgs/Odometry.h"
#include "configuration_vehicle_hardware.h"

struct State
{
  float steering_angle;
  float steering_angle_velocity;
  float speed;
  float acceleration;
  float jerk;
};

struct RemoteControl
{
  float speed_volts;
  float steering_angle_pwm;

  State desired_state;
};

class Vehicle;
typedef Vehicle* VehiclePtr;

class Vehicle
{
private:

  const unsigned int SAMPLING_TIME_SPEED_ = (int)((1.0 / SAMPLING_HERTZ_SPEED) * 1000.0);       //ms
  const unsigned int SAMPLING_TIME_STEERING_ = (int)((1.0 / SAMPLING_HERTZ_STEERING) * 1000.0); //ms
  const unsigned int TIME_TO_PREDICT_MILLIS_ = 50; //ms

  elapsedMillis millis_since_last_steering_reading_ = 0; //!< Timer to refresh steering information (trough the Teensy)
  elapsedMillis millis_since_last_speed_reading_ = 0; //!< Timer to refresh speed information (trough the Teensy)

  elapsedMillis zero_volts_millis_before_braking_ = 0; //!< Timer to activate the brakes after MAX_TIME_ZERO_VOLTS_TO_BRAKE

  elapsedMillis millis_since_last_EKF_prediction_ = 0; //!< Timer to control the EKF prediction rate (adjusted by means of TIME_TO_PREDICT_MILLIS)

  float* speed_measures_;
  float* steering_measures_;

  State estimated_state_;
  State measured_state_;
  State desired_state_;

  bool flag_limiting_speed_by_reactive_;bool remote_control_use_PID_;

  RemoteControl remote_control_;

  int operational_mode_;
  int last_operational_mode_;
  int error_code_;

  PIDPtr speed_controller_;
  PIDPtr steering_controller_;

  float speed_volts_;
  float steering_angle_pwm_;

  float speed_volts_pid_;
  float steering_angle_pwm_pid_;

  float left_steering_limit_switch_position_;
  float right_steering_limit_switch_position_;

  DJI_DBUSPtr dBus_;

  SpeedHardwareInterfacePtr speed_actuator_;
  SteeringHardwareInterfacePtr steering_actuator_;

  byte led_rgb_value_[3];

  EKFPtr state_estimator_;

  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  void saturateSetpointIfNeeded(float &speed);

  void resetSpeed(void);

  void resetSteering(void);

public:

  Vehicle();
  ~Vehicle();

  /*!
   * Maps the variables in the ROS message to the inner State struct variables: Speed, Acceleration,
   * Jerk, Steering Angle and Steering Angle Velocity
   * @param desired_ackermann_state The state variables in ROS message format
   */
  void updateROSDesiredState(const ackermann_msgs::AckermannDriveStamped& desired_ackermann_state);

  /*!
   * Reads the variables that store the odometers readings
   * and calculates all the state variables: speed, acceleration,
   * jerk, steering angle and steering angle velocity
   */
  void updateState(ackermann_msgs::AckermannDriveStamped& measured_ackermann_state,
                   ackermann_msgs::AckermannDriveStamped& covariance_ackermann_state);

  /*!
   * Reads the buttons and switches wired in the own vehicle, that affects
   * to the finite state machine (transition between Reset, RC, ROS control, and Emergency stop)
   * and other variables (lighting on / off etc...)
   */
  void readOnBoardUserInterface(void);

  /*!
   * Reads the RC mapping all the channels to inner variables
   */
  void readRemoteControl(void);

  /*!
   * Implements the logic to change between states: Reset, RC, ROS control and Emergency Stop
   */
  void updateFiniteStateMachine(int millisSinceLastReactiveUpdate);

  /*!
   * Use the PID controllers to calculate the voltage and PWM values to control the hardware devices
   * forward velocity and steering actuators. The inputs are the measured and desired states.
   * This function is used in ROS control and Reset modes
   */
  void calculateCommandOutputs(float max_recommended_speed);

  /*!
   * Writes to the Arduino outputs the required voltages and PWM values
   */
  void writeCommandOutputs(std_msgs::Float32MultiArray& speed_volts_and_steering_pwm_being_applicated);

  /*!
   * Returns the desired state for debugging
   */
  void getDesiredState(ackermann_msgs::AckermannDriveStamped& desired_ackermann_state_echo);

  /*!
   * Maps variables in the ROS messages to the inner PID gains variables
   * @param desired_vel_pid_gains
   * @param desired_ste_pid_gains
   */
  void setSpeedAndSteeringPIDGains(const std_msgs::Float32MultiArray& desired_vel_pid_gains,
                                   const std_msgs::Float32MultiArray& desired_ste_pid_gains);

  /*!
   * Copy to inner velocity PID variables the values in the ROS message
   * @param desired_pid_gains
   */
  void setSpeedPIDGains(const std_msgs::Float32MultiArray& desired_pid_gains);

  /*!
   * Pass the inner velocity PID gain values to ROS message format
   * @param current_pid_gains
   */
  void getSpeedPIDGains(std_msgs::Float32MultiArray& current_pid_gains);

  /*!
   * Copy to inner steering PID variables the values in the ROS message
   * @param desired_pid_gains
   */
  void setSteeringPIDGains(const std_msgs::Float32MultiArray& desired_pid_gains);

  /*!
   * Pass the inner steering PID gain values to ROS message format
   * @param current_pid_gains
   */
  void getSteeringPIDGains(std_msgs::Float32MultiArray& current_pid_gains);

  void setLimitSwitchesPositionLR(std_msgs::Float32MultiArray& desired_limit_switches_position);

  void getLimitSwitchesPositionLR(std_msgs::Float32MultiArray& current_limit_switches_position);

  /*!
   * returns the current operational mode, that is the same that the current
   * state of the finite state machine: Reset, RC, ROS control or Emergency Stop
   * @param current_operational_mode
   */
  void getOperationalMode(int& current_operational_mode);
  int getOperationalMode(void);

  /*!
   * Returns the error code that can express different causes to be in Emergency
   * Stop state, (sensor failure... ) or give NO_ERROR code.
   * @param requested_error_code
   */
  void getErrorCode(int& requested_error_code);
  int getErrorCode(void);
};

#endif /* HEADERS_VEHICLE_H_ */
