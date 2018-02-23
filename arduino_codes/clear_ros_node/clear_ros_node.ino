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
 *  - CallBacks end with "CB"
 */

#include <ros.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "Arduino.h"
#include <avr/wdt.h>
#include <Wire.h> // I2C library

#include "headers/arduino_ros_interface.h"
#include "headers/vehicle.h"

int warning_code = NO_WARNINGS;

Vehicle AckermannVehicle;

ros::NodeHandle nh;

/*! \brief Callback to read the desired verbose level
 *
 * @param desired_verbose_level_msg a message that contains an int indicating the desired verbose level
 */
int verbose_level = MAX_VERBOSE_LEVEL;
void desiredVerboseLevelCB(const std_msgs::Int16& desired_verbose_level_msg)
{
  // blink the led to check that the callback is being called
  digitalWrite(BUILT_IN_ARDUINO_LED_PORT, HIGH - digitalRead(BUILT_IN_ARDUINO_LED_PORT));

  verbose_level = desired_verbose_level_msg.data;
}
ros::Subscriber<std_msgs::Int16> verbose_level_subscriber("desired_verbose_level", &desiredVerboseLevelCB);




/*! \brief CallBack to read the desired ackermann state coming from the on-board PC
 *
 * @param desired_ackermann_state_msg a desired ackermann state message that should include at least
 * the steering angle (radians) and the forward velocity (m/s), this is the interface between the
 * PC (high level) and Arduino (low level) controllers that finally control the Blue Barrow actuators
 */
ackermann_msgs::AckermannDriveStamped desired_ackermann_state;
ackermann_msgs::AckermannDriveStamped desired_ackermann_state_echo;
void desiredAckermannStateCB(const ackermann_msgs::AckermannDriveStamped& desired_ackermann_state_msg)
{
  if (AckermannVehicle.getOperationalMode() == ROS_CONTROL)
  {
    // blink the led to check that the callback is being called
    digitalWrite(BUILT_IN_ARDUINO_LED_PORT, HIGH - digitalRead(BUILT_IN_ARDUINO_LED_PORT));

    //Copy the incoming message
    desired_ackermann_state.header = desired_ackermann_state_msg.header;
    desired_ackermann_state.drive = desired_ackermann_state_msg.drive;

    /*
    if (verbose_level == MAX_VERBOSE_LEVEL)
    {

      //Copy to echo the incoming command
      desired_ackermann_state_echo = desired_ackermann_state; // just repeat the input to check communications
    }
    */
  }
  else
  {
    warning_code = RECEIVING_ROS_CONTROLS_WHILE_NOT_BEING_IN_ROS_MODE;
  }
}
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermann_subscriber("desired_ackermann_state",
                                                                            &desiredAckermannStateCB);

ros::Publisher required_ackermann_publisher("echo_desired_ackermann_state", &desired_ackermann_state_echo);




/*!
 * Publisher to communicate the ackermann state measured with the sensor hall and the encoder
 */
ackermann_msgs::AckermannDriveStamped measured_ackermann_state;
ros::Publisher measured_ackermann_publisher("measured_ackermann_state", &measured_ackermann_state);




/*! \brief CallBack to read the velocity control PID gains (kp, ki, and kd) coming from the on-board PC
 *
 * @param pid_gains_msg a message containing three floats, in order : kp, ki and kd
 */
std_msgs::Float32MultiArray desired_vel_pid_gains;
std_msgs::Float32MultiArray vel_pid_gains_echo;
void velPIDGainsCB(const std_msgs::Float32MultiArray& vel_pid_gains_msg)
{
  // blink the led to check that the callback is being called
  digitalWrite(BUILT_IN_ARDUINO_LED_PORT, HIGH - digitalRead(BUILT_IN_ARDUINO_LED_PORT));

  desired_vel_pid_gains = vel_pid_gains_msg;
  desired_vel_pid_gains.data_length = 3;
  /*
  if (verbose_level == MAX_VERBOSE_LEVEL)
  {
    vel_pid_gains_echo = desired_vel_pid_gains;
  }
  */
}
ros::Subscriber<std_msgs::Float32MultiArray> vel_pid_gains_subscriber("desired_vel_pid_gains", &velPIDGainsCB);

ros::Publisher required_vel_pid_gains_publisher("echo_desired_vel_pid_gains", &vel_pid_gains_echo);




/*! \brief CallBack to read the steering control PID gains (kp, ki, and kd)
 *
 * @param pid_gains_msg a message containing three floats, in order : kp, ki and kd
 */
std_msgs::Float32MultiArray desired_ste_pid_gains;
std_msgs::Float32MultiArray ste_pid_gains_echo;
void steeringPIDGainsCB(const std_msgs::Float32MultiArray& ste_pid_gains_msg)
{
  // blink the led to check that the callback is being called
  digitalWrite(BUILT_IN_ARDUINO_LED_PORT, HIGH - digitalRead(BUILT_IN_ARDUINO_LED_PORT));

  desired_ste_pid_gains = ste_pid_gains_msg;
  desired_ste_pid_gains.data_length = 3;
  /*
  if (verbose_level == MAX_VERBOSE_LEVEL)
  {
    ste_pid_gains_echo = desired_ste_pid_gains;
  }
  */
}
ros::Subscriber<std_msgs::Float32MultiArray> steering_pid_gains_subscriber("desired_steering_pid_gains",
                                                                           &steeringPIDGainsCB);

ros::Publisher required_steering_pid_gains_publisher("echo_desired_steering_pid_gains", &ste_pid_gains_echo);



/*!
 * Publisher to communicate to ROS the actual signals applied to the actuators (speed and steering)
 */
std_msgs::Float32MultiArray speed_volts_and_steering_pwm_being_applicated;
ros::Publisher speed_volts_and_steering_pwm("speed_volts_and_steering_pwm", &speed_volts_and_steering_pwm_being_applicated);




/*!
 * \brief write the status message with current values
 * This message should be updated if more status codes are added!
 */
std_msgs::Int16MultiArray arduino_status;
ros::Publisher arduino_status_publisher("arduino_status", &arduino_status);
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
  arduino_status.data = (int *)malloc(sizeof(int) * NUM_OF_ARDUINO_STATUS_VARIABLES);

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
}




/*!
 * \brief Initialising the ROS node, subscribers and publishers
 */
void setup()
{
  wdt_disable();
  Serial.begin(57600);

  Wire.begin();
  Wire.setClock(100000); //100Kbps

  // Changes PWM frecuency to 31KHz (scaler 1)
  // Default 490Hz (scaler 3)
  // https://playground.arduino.cc/Code/PwmFrequency
  // https://forum.arduino.cc/index.php?topic=72092.0
  // PWM PIN 10 Mega -> Timmer 2
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

  nh.advertise(arduino_status_publisher);
  nh.advertise(speed_volts_and_steering_pwm);

  nh.advertise(measured_ackermann_publisher);

  wdt_enable(WDTO_500MS);
}




/*!
 * \brief Function to activate a flag to communicate with ROS periodically
 *
 * @return check that is a flag set to true when the time elapsed since the
 * previous communication is greater than the specified in the
 * COMMUNICATION_REFRESH_TIME_IN_MILLIS constant, is reset to false when
 * this condition is not satisfied
 */
unsigned long current_time = 0;
unsigned long previous_time = 0;
bool checkIfItsTimeToInterfaceWithROS(void)
{
  bool check = false;

  current_time = millis();

  if (current_time - previous_time > COMMUNICATION_REFRESH_TIME_IN_MILLIS)
  {
    check = true;
    previous_time = current_time;
  }
  return(check);
}




/*!
 * \brief Attends the callbacks and pass the input values to the vehicle
 */
void receiveROSInputs(void)
{
    //! Attending callbacks
    nh.spinOnce();

    //! passing ROS inputs to the vehicle
    AckermannVehicle.updateDesiredState(desired_ackermann_state);
    AckermannVehicle.updatePIDGains(desired_vel_pid_gains, desired_ste_pid_gains);
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

    AckermannVehicle.getVelocityPIDGains(vel_pid_gains_echo);
    required_vel_pid_gains_publisher.publish(&vel_pid_gains_echo);

    AckermannVehicle.getSteeringPIDGains(ste_pid_gains_echo);
    required_steering_pid_gains_publisher.publish(&ste_pid_gains_echo);
  }
  if (verbose_level >= MIN_VERBOSE_LEVEL)
  {
    sendArduinoStatus();
    measured_ackermann_publisher.publish(&measured_ackermann_state);
    speed_volts_and_steering_pwm.publish(&speed_volts_and_steering_pwm_being_applicated);
  }
}




/*!
 * \brief Arduino main loop, from here are managed all communications, sensors and actuators
 */
bool communicate_with_ROS = false;
void loop()
{
  wdt_reset();

  communicate_with_ROS = checkIfItsTimeToInterfaceWithROS();

  if (communicate_with_ROS) receiveROSInputs();

  AckermannVehicle.updateMeasuredState(measured_ackermann_state);

  AckermannVehicle.readOnBoardUserInterface();

  if(AckermannVehicle.getOperationalMode() != CALIBRATION)
	  AckermannVehicle.readRemoteControl();

  AckermannVehicle.updateFiniteStateMachine();

  AckermannVehicle.calculateCommandOutputs();

  if(AckermannVehicle.getOperationalMode() != CALIBRATION)
	  AckermannVehicle.writeCommandOutputs(speed_volts_and_steering_pwm_being_applicated);

  if (communicate_with_ROS) sendOutputsToROS();
}
