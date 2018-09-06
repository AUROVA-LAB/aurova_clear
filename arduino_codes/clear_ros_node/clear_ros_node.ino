/*! \file arduino_ros_node.ino
 *  \brief CLEAR Arduino ROS node main file
 *
 *  This file contains the interface between Arduino and the ROS based system
 *  through the on-board PC, it is intended to receive the commands from the on-board
 *  PC and to communicate the operation mode, safety signals, sensor readings and
 *  configuration values.
 *  The main loop executes the communications, sensor readings and low level
 *  controllers (steering and forward velocity).
 *  Non standard naming conventions in this file:
 *  - CallBacks start with "cb_"
 */

#include <ros.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16MultiArray.h"
#include "Arduino.h"
#include <avr/wdt.h>
#include <Wire.h> // I2C library

#include "headers/arduino_ros_interface.h"
#include "headers/vehicle.h"
#include "headers/configuration_vehicle_hardware.h"

///////////////////////////////////////////////////
// Declaration of Arduino global variables
///////////////////////////////////////////////////

////////////////////////
// General
bool communicate_with_ROS = false;
unsigned long int current_time  = 0; //!< Variable used to publish topics at desired rate (set in COMMUNICATION_REFRESH_TIME_IN_MILLIS)
unsigned long int previous_time = 0; //!< The other variable used to publish topics at desired rate

elapsedMillis reactive_watchdog = 0; //!< Watchdog to enter into EMERGENCY mode if the safety reactive callback is not activated

Vehicle AckermannVehicle;

ros::NodeHandle nh;

////////////////////////
// Inputs
int verbose_level = MAX_VERBOSE_LEVEL;
float max_recommended_speed = 0.0;   //!< To store the maximum speed recommended by the reactive systems

// Messages to store input data in callbacks
ackermann_msgs::AckermannDriveStamped desired_ackermann_state;
std_msgs::Float32MultiArray desired_vel_pid_gains; //!< No need of initializing it since this gains are only used after the callback execution
std_msgs::Float32MultiArray desired_ste_pid_gains; //!< No need of initializing it since this gains are only used after the callback execution
std_msgs::Float32MultiArray desired_limit_switches_position_LR;


///////////////////////
// Outputs
int warning_code = NO_WARNINGS;

ackermann_msgs::AckermannDriveStamped estimated_ackermann_state; //!< mean values of EKF estimated state: steering angle (deg) and speed (m/s)
ackermann_msgs::AckermannDriveStamped variances_of_estimated_ackermann_state; //!< variances of EKF estimated values
std_msgs::Float32MultiArray speed_volts_and_steering_pwm_being_applicated;
std_msgs::Float32MultiArray arduino_status;

// Echoes to check communications
ackermann_msgs::AckermannDriveStamped desired_ackermann_state_echo;
std_msgs::Float32MultiArray vel_pid_gains_echo;
std_msgs::Float32MultiArray ste_pid_gains_echo;
std_msgs::Float32MultiArray echo_desired_limit_switches_position_LR;


//////////////////////////////////////////////////////
// Callbacks
//////////////////////////////////////////////////////

/*! \brief Callback to store the desired verbose level
 *
 * @param desired_verbose_level_msg a message that contains an integer code indicating
 * the desired verbose level defined in arduino_ros_interface.h
 */
void cb_desiredVerboseLevel(const std_msgs::Int16& desired_verbose_level_msg)
{
  verbose_level = desired_verbose_level_msg.data;
}

/*! \brief Callback to store the maximum recommended speed, it sets a flag to indicate that the safety system is alive
 *  and resets the watchdog
 */
void cb_maxRecommendedSpeed(const std_msgs::Float32& max_recommended_speed_msg)
{
  max_recommended_speed = max_recommended_speed_msg.data;
  reactive_watchdog = 0;
}

/*! \brief CallBack to read the desired ackermann state coming from the on-board PC
 *
 * This is the interface between the PC (high level) and Arduino (low level) to control the robot actuators
 *
 * @param desired_ackermann_state_msg a desired ackermann state message that should include
 * the steering angle (degrees) and the forward velocity (m/s). The rest of the values are unused.
 */
void cb_desiredAckermannState(const ackermann_msgs::AckermannDriveStamped& desired_ackermann_state_msg)
{
  if (AckermannVehicle.getOperationalMode() == ROS_CONTROL)
  {
    //We copy the incoming message
    desired_ackermann_state.header = desired_ackermann_state_msg.header;
    desired_ackermann_state.drive = desired_ackermann_state_msg.drive;
  }
  else
  {
    warning_code = RECEIVING_ROS_CONTROLS_WHILE_NOT_BEING_IN_ROS_MODE;
  }
}

/*! \brief CallBack to store the velocity control PID gains (kp, ki, and kd) coming from the on-board PC
 *
 * @param pid_gains_msg a message containing three floats, in order : kp, ki and kd
 */
void cb_velPIDGains(const std_msgs::Float32MultiArray& vel_pid_gains_msg)
{
  desired_vel_pid_gains = vel_pid_gains_msg;
  desired_vel_pid_gains.data_length = 3;

  AckermannVehicle.setSpeedPIDGains(desired_vel_pid_gains);
}

/*! \brief CallBack to read the steering control PID gains (kp, ki, and kd)
 *
 * @param pid_gains_msg a message containing three floats, in order: kp, ki and kd
 */
void cb_steeringPIDGains(const std_msgs::Float32MultiArray& ste_pid_gains_msg)
{
  desired_ste_pid_gains = ste_pid_gains_msg;
  desired_ste_pid_gains.data_length = 3;

  AckermannVehicle.setSteeringPIDGains(desired_ste_pid_gains);

}

/*! \brief CallBack to calibrate dynamically the steering limit switches angular position
 *
 * @param limit_switches_msg a message containing two floats, in order: left position and right position (expressed in degrees)
 */
void cb_limitSwitchesCalibration(const std_msgs::Float32MultiArray& limit_switches_msg)
{
  desired_limit_switches_position_LR = limit_switches_msg;
  desired_limit_switches_position_LR.data_length = 2;

  AckermannVehicle.setLimitSwitchesPositionLR(desired_limit_switches_position_LR);

}

/////////////////////////////////////////////
// Subscribers
/////////////////////////////////////////////

/*!
 * Subscriber that receives a code to adequate the amount of debugging information that the Arduino will produce
 */
ros::Subscriber<std_msgs::Int16> verbose_level_subscriber("desired_verbose_level", &cb_desiredVerboseLevel);

/*!
 * Subscriber that receives the maximum speed for the platform, it comes from the safety system
 */
ros::Subscriber<std_msgs::Float32> max_recommended_speed_subscriber(
    "velocity_recommender_alg_node/forward_recommended_velocity", &cb_maxRecommendedSpeed);

/*!
 * Subscriber that receives the setpoint for the PID controllers (steering in deg and speed in m/s)
 */
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermann_subscriber("desired_ackermann_state",
                                                                            &cb_desiredAckermannState);

/*!
 * Subscriber that receives the gains for the PID speed controller
 */
ros::Subscriber<std_msgs::Float32MultiArray> vel_pid_gains_subscriber("desired_vel_pid_gains", &cb_velPIDGains);

/*!
 * Subscriber that receives the gains for the PID steering controller
 */
ros::Subscriber<std_msgs::Float32MultiArray> steering_pid_gains_subscriber("desired_steering_pid_gains",
                                                                           &cb_steeringPIDGains);

/*!
 * Subscriber that receives the angular position of the limit switches, used for online calibration
 */
ros::Subscriber<std_msgs::Float32MultiArray> limit_switches_calibration_subscriber("desired_limit_switches_position_LR",
                                                                                   &cb_limitSwitchesCalibration);


///////////////////////////////////////////////
// Publishers
///////////////////////////////////////////////

/////////////
// Echoes
/*!
 * Publisher to produce an echo of the desired ackermann state to check communications
 */
ros::Publisher required_ackermann_publisher("echo_desired_ackermann_state", &desired_ackermann_state_echo);

/*!
 * Publisher to produce an echo of the desired vel pid gains to check communications
 */
ros::Publisher required_vel_pid_gains_publisher("echo_desired_vel_pid_gains", &vel_pid_gains_echo);

/*!
 * Publisher to produce an echo of the desired steering pid gains to check communications
 */
ros::Publisher required_steering_pid_gains_publisher("echo_desired_steering_pid_gains", &ste_pid_gains_echo);

/*!
 * Publisher to produce an echo of the desired limit switches position to check communications
 */
ros::Publisher required_limit_switches_position_publisher("echo_desired_limit_switches_position_LR",
                                                          &echo_desired_limit_switches_position_LR);


////////////////
// Non-echoes
/*!
 * Publisher to communicate the mean of the EKF estimated ackermann state (using the speed and steering encoders)
 */
ros::Publisher estimated_ackermann_publisher("estimated_ackermann_state", &estimated_ackermann_state);

/*!
 * Publisher to communicate the EKF variances of the estimated ackermann state
 */
ros::Publisher covariance_ackermann_publisher("variances_of_estimated_ackermann_state",
                                              &variances_of_estimated_ackermann_state);

/*!
 * Publisher to communicate to ROS the actual signals applied to the actuators (speed and steering)
 */
ros::Publisher speed_volts_and_steering_pwm("speed_volts_and_steering_pwm",
                                            &speed_volts_and_steering_pwm_being_applicated);

/*!
 * \brief write the status message with current values
 * This message should be updated if more status codes are added!
 */
ros::Publisher arduino_status_publisher("arduino_status", &arduino_status);


/////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////

/*!
 * \brief Fill the status data and publish the message
 */
void sendArduinoStatus(void)
{
  arduino_status.data[0] = AckermannVehicle.getOperationalMode();
  arduino_status.data[1] = AckermannVehicle.getErrorCode();
  arduino_status.data[2] = warning_code;
  arduino_status.data[3] = verbose_level;

  arduino_status_publisher.publish(&arduino_status);
}

/*!
 * \brief Dynamic memory reserve for all array messages
 */
void reserveDynamicMemory(void)
{
  arduino_status.data_length = NUM_OF_ARDUINO_STATUS_VARIABLES;
  arduino_status.data = (float *)malloc(sizeof(float) * NUM_OF_ARDUINO_STATUS_VARIABLES);

  speed_volts_and_steering_pwm_being_applicated.data_length = NUM_OF_CONTROLLED_MOTORS;
  speed_volts_and_steering_pwm_being_applicated.data = (float *)malloc(sizeof(float) * NUM_OF_CONTROLLED_MOTORS);

  desired_vel_pid_gains.data_length = NUM_OF_PID_GAINS;
  desired_vel_pid_gains.data = (float *)malloc(sizeof(float) * NUM_OF_PID_GAINS);

  vel_pid_gains_echo.data_length = NUM_OF_PID_GAINS;
  vel_pid_gains_echo.data = (float *)malloc(sizeof(float) * NUM_OF_PID_GAINS);

  desired_ste_pid_gains.data_length = NUM_OF_PID_GAINS;
  desired_ste_pid_gains.data = (float *)malloc(sizeof(float) * NUM_OF_PID_GAINS);

  ste_pid_gains_echo.data_length = NUM_OF_PID_GAINS;
  ste_pid_gains_echo.data = (float *)malloc(sizeof(float) * NUM_OF_PID_GAINS);

  desired_limit_switches_position_LR.data_length = NUM_OF_STEERING_LIMIT_SWITCHES;
  desired_limit_switches_position_LR.data = (float *)malloc(sizeof(float) * NUM_OF_STEERING_LIMIT_SWITCHES);

  echo_desired_limit_switches_position_LR.data_length = NUM_OF_STEERING_LIMIT_SWITCHES;
  echo_desired_limit_switches_position_LR.data = (float *)malloc(sizeof(float) * NUM_OF_STEERING_LIMIT_SWITCHES);
}

/*!
 * \brief Initialising the ROS node, subscribers and publishers
 */
void setup()
{
  Serial.begin(115200);

  wdt_disable();           //!< watchdog timer configuration
  wdt_enable(WDTO_500MS);  //!< it is set to automatically reboot the arduino in case of main loop hanging

  Wire.begin();
  Wire.setClock(100000); //100Kbps

  // Changes PWM frecuency to 31KHz (scaler 1)
  // Default 490Hz (scaler 3)
  // https://playground.arduino.cc/Code/PwmFrequency
  // https://forum.arduino.cc/index.php?topic=72092.0
  // PWM PIN 10 Mega -> Timer 2
  TCCR2B = (TCCR2B & 0b11111000) | 1;

  reserveDynamicMemory();

  //! Initialise ROS node and interface
  nh.initNode();

  nh.subscribe(verbose_level_subscriber);

  nh.subscribe(ackermann_subscriber);
  nh.advertise(required_ackermann_publisher);

  nh.subscribe(vel_pid_gains_subscriber);
  nh.advertise(required_vel_pid_gains_publisher);

  nh.subscribe(steering_pid_gains_subscriber);
  nh.advertise(required_steering_pid_gains_publisher);

  nh.subscribe(limit_switches_calibration_subscriber);
  nh.advertise(required_limit_switches_position_publisher);

  nh.advertise(arduino_status_publisher);
  nh.advertise(speed_volts_and_steering_pwm);

  nh.advertise(estimated_ackermann_publisher);
  nh.advertise(covariance_ackermann_publisher);

  nh.subscribe(max_recommended_speed_subscriber);
}

/*!
 * \brief Function to activate a flag to communicate with ROS periodically
 *
 * @return check that is a flag set to true when the time elapsed since the
 * previous communication is greater than the specified in the
 * COMMUNICATION_REFRESH_TIME_IN_MILLIS constant, is reset to false when
 * this condition is not satisfied
 */
bool checkIfItsTimeToInterfaceWithROS(void)
{
  bool check = false;

  current_time = millis();

  if (current_time - previous_time > COMMUNICATION_REFRESH_TIME_IN_MILLIS)
  {
    check = true;
    previous_time = current_time;
  }
  return (check);
}

/*!
 * \brief Attends the callbacks and pass the input values to the vehicle
 */
void receiveROSInputs(void)
{
  //! Attending callbacks
  nh.spinOnce();

  //! passing ROS inputs to the vehicle
  if (AckermannVehicle.getOperationalMode() == ROS_CONTROL)
  {
    AckermannVehicle.updateROSDesiredState(desired_ackermann_state);
  }
}

/*!
 * \brief Send information to ROS, the level of detail is managed through
 * the 'verbose_level' variable
 */
void sendOutputsToROS(void)
{
  //! Publishing Arduino inner information to ease debug and monitoring
  if (verbose_level == MAX_VERBOSE_LEVEL)
  {
    AckermannVehicle.getDesiredState(desired_ackermann_state_echo);
    required_ackermann_publisher.publish(&desired_ackermann_state_echo);

    AckermannVehicle.getSpeedPIDGains(vel_pid_gains_echo);
    required_vel_pid_gains_publisher.publish(&vel_pid_gains_echo);

    AckermannVehicle.getSteeringPIDGains(ste_pid_gains_echo);
    required_steering_pid_gains_publisher.publish(&ste_pid_gains_echo);

    AckermannVehicle.getLimitSwitchesPositionLR(echo_desired_limit_switches_position_LR);
    required_limit_switches_position_publisher.publish(&echo_desired_limit_switches_position_LR);
  }
  if (verbose_level >= MIN_VERBOSE_LEVEL)
  {
    sendArduinoStatus();
    estimated_ackermann_publisher.publish(&estimated_ackermann_state);
    covariance_ackermann_publisher.publish(&variances_of_estimated_ackermann_state);
    speed_volts_and_steering_pwm.publish(&speed_volts_and_steering_pwm_being_applicated);
  }
}

/*!
 * \brief Arduino main loop, from here are managed all communications, sensors and actuators
 */
void loop()
{
  wdt_reset(); //!< each new loop we reset the hanging watchdog timer

  communicate_with_ROS = checkIfItsTimeToInterfaceWithROS(); //!< Flag to control the ROS reading and publishing rate

  if (communicate_with_ROS) receiveROSInputs(); //!< Attending callbacks

  AckermannVehicle.updateState(estimated_ackermann_state, variances_of_estimated_ackermann_state); //!< EKF iteration

  AckermannVehicle.readOnBoardUserInterface(); //!< Read the controls available on-board (currently only the emergency switch)

  AckermannVehicle.readRemoteControl(); //!< Decode and map the RC signals

  int millisSinceLastReactiveUpdate = reactive_watchdog; //!< Created to pass the safety watchdog value to the finite state machine
  AckermannVehicle.updateFiniteStateMachine(millisSinceLastReactiveUpdate);

  AckermannVehicle.calculateCommandOutputs(max_recommended_speed); //!< It uses the reactive safety system speed limitation

  AckermannVehicle.writeCommandOutputs(speed_volts_and_steering_pwm_being_applicated); //!< Here we actuate the motors

  if (communicate_with_ROS) sendOutputsToROS(); //!< publishing the ROS interface topics
}
