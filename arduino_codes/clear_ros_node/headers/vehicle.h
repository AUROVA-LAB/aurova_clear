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
#include "../headers/DJI_DBUS.h"
#include "../headers/speed_hardware_interface.h"
#include "../headers/steering_hardware_interface.h"
#include "../headers/PID_v1.h"

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
};

class Vehicle;
typedef Vehicle* VehiclePtr;

class Vehicle
{
private:
  State measured_state_;
  State desired_state_;

  bool desired_steering_state_reached_;
  bool desired_traslational_state_reached_;

  RemoteControl remote_control_;

  int operational_mode_;
  int error_code_;

  PIDPtr speed_controller_;
  PIDPtr steering_controller_;

  float speed_volts_;
  float steering_angle_pwm_;

  float speed_volts_pid_;
  float steering_angle_pwm_pid_;

  DJI_DBUSPtr dBus_;

  SpeedHardwareInterfacePtr speed_actuator_;
  SteeringHardwareInterfacePtr steering_actuator_;

  byte led_rgb_value_[3];


  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
  {
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

public:

  Vehicle();
  ~Vehicle();

  /*!
   * Reads the variables that store the odometers readings
   * and calculates all the state variables: speed, acceleration,
   * jerk, steering angle and steering angle velocity
   */
  void updateMeasuredState(ackermann_msgs::AckermannDriveStamped& measured_ackermann_state);

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
  void updateFiniteStateMachine(void);

  /*!
   * Maps the variables in the ROS message to the inner State struct variables: Speed, Acceleration,
   * Jerk, Steering Angle and Steering Angle Velocity
   * @param desired_ackermann_state The state variables in ROS message format
   */
  void updateDesiredState(const ackermann_msgs::AckermannDriveStamped& desired_ackermann_state);

  /*!
   * Returns the desired state for debugging
   */
  void getDesiredState(ackermann_msgs::AckermannDriveStamped& desired_ackermann_state_echo);

  /*!
   * Maps variables in the ROS messages to the inner PID gains variables
   * @param desired_vel_pid_gains
   * @param desired_ste_pid_gains
   */
  void updatePIDGains(const std_msgs::Float32MultiArray& desired_vel_pid_gains,
                      const std_msgs::Float32MultiArray& desired_ste_pid_gains);

  /*!
   * Use the PID controllers to calculate the voltage and PWM values to control the hardware devices
   * forward velocity and steering actuators. The inputs are the measured and desired states.
   * This function is used in ROS control and Reset modes
   */
  void calculateCommandOutputs(void);

  /*!
   * Writes to the Arduino outputs the required voltages and PWM values
   */
  void writeCommandOutputs(std_msgs::Float32MultiArray& speed_volts_and_steering_pwm_being_applicated);

  /*!
   * Makes the first calibration of the vehicle components before run
   */
  bool componentsCalibration(void);

  /*!
   * Copy to inner velocity PID variables the values in the ROS message
   * @param desired_pid_gains
   */
  void setVelocityPIDGains(const std_msgs::Float32MultiArray& desired_pid_gains);

  /*!
   * Pass the inner velocity PID gain values to ROS message format
   * @param current_pid_gains
   */
  void getVelocityPIDGains(std_msgs::Float32MultiArray& current_pid_gains);

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

  /*!
   * returns the current operational mode, that is the same that the current
   * state of the finite state machine: Reset, RC, ROS control or Emergency Stop
   * @param current_operational_mode
   */
  void getOperationalMode(int& current_operational_mode);
  int  getOperationalMode(void);

  /*!
   * Returns the error code that can express different causes to be in Emergency
   * Stop state, (sensor failure... ) or give NO_ERROR code.
   * @param requested_error_code
   */
  void getErrorCode(int& requested_error_code);
  int  getErrorCode(void);

};

#endif /* HEADERS_VEHICLE_H_ */
