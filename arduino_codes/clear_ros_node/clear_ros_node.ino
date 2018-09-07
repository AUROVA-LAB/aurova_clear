/*! \file clear_ros_node.ino
 *
 *  \author Ivan del Pino, Saul Cova and Francisco Candelas
 *
 *  \brief CLEAR: Control Logic for Easy Ackermann Robotization
 *
 *  This file contains an Arduino ROS node, implemented using Rosserial,
 *  that constitutes the interface between the Low and High level systems.
 *  The code here implements the interface, logic and algorithms
 *  required to supervise, control and adjust dynamically an Ackermann
 *  mobile robot, that should have two motors (one to control the speed
 *  and other to control the steering) both equipped with incremental encoders.
 *
 *  To estimate the state of the platform we use an Extended Kalman Filter.
 *  The control of the motors is performed using two separated PID controllers.
    In addition to the ROS interface, CLEAR also implements a Remote Control
 *  interface that enables the user to control manually --either changing the
 *  PIDs setpoint or changing directly the voltage (speed) and pwm (steering)--
 *
 *  There are 4 operational modes: (The transition between different
 *                                  operational modes is controlled
 *                                  trough the RC switches)
 *
 *  -ROS_CONTROL: In this mode the robot is controlled using the
 *                "desired_ackermann_state" ROS topic.
 *                In this mode a reactive High Level safety layer is required,
 *                to avoid collisions. This information is received through the
 *                velocity_recommender node, that sets the maximum speeds allowed
 *                depending on the distance to surrounding obstacles.
 *                If the safety topics are not received, the robot goes to Emergency
 *                state, disabling motors and activating the brakes.
 *
 *  -REMOTE_CONTROL: In this mode the PIDs setpoints are set using the RC, mapping the control
 *                   rods positions into meters per second (for the speed controller) and steering
 *                   angle (in degrees) for the steering controller. The safety layer works
 *                   exactly as in ROS_CONTROL mode.
 *
 *  -REMOTE_CONTROL_NOT_SAFE: This mode allows fully manual control of the platform,
 *                            without safety layer, and even permits to disable the PID
 *                            controllers, mapping directly the RC control rods positions
 *                            to voltages and PWM values to drive the motors.
 *
 *  -EMERGENCY_STOP: In this mode, the robot is stopped, disabling motors and activating
 *                   the brakes.
 *                   CLEAR enter in this mode if any of the emergency switches
 *                   (RC or on-board) is activated, also if the Remote Control signal is lost
 *                   or if the Reactive safety layer topics are not received (unless if the
 *                   current operational mode is REMOTE_CONTROL_NOT_SAFE).
 *                   To exit from EMERGENCY_STOP mode all emergency switches must be reset, the
 *                   RC operational mode switch must be set to REMOTE_CONTROL_NOT_SAFE and finally
 *                   the rearm/horn wheel must be turned until reach the rearm position.
 *                   More details of the RC interface are available in the GitHub repository.
 */

#include <ackermann_robot.h>
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
#include "headers/configuration_vehicle_hardware.h"

///////////////////////////////////////////////////
// Declaration of Arduino global variables
///////////////////////////////////////////////////

////////////////////////
// General
bool communicate_with_ROS = false;   //!< Flag set in COMMUNICATION_REFRESH_TIME_IN_MILLIS periods to refresh the ROS interface
unsigned long int current_time  = 0; //!< Time counter to set the communication refresh flag properly
unsigned long int previous_time = 0; //!< Second time counter for communication refresh (we need to counters to calculate time increments)

elapsedMillis reactive_watchdog = 0; //!< Watchdog to enter into EMERGENCY mode if the safety reactive callback is not activated

AckermannRobot myRobot; //!< The class AckermannRobot implements all the algorithms needed to control the machine

ros::NodeHandle nh; //!< The ros node implements the high level interface

////////////////////////
// Inputs
int verbose_level = MAX_VERBOSE_LEVEL; //!< Variable to set the amount of debugging information that is sent trough the ROS interface
float max_recommended_forward_speed  = 0.0; //!< Variable to store the maximum forward speed recommended by the reactive systems
float max_recommended_backward_speed = 0.0; //!< Variable to store the maximum backward speed recommended by the reactive systems

// Messages to store input topics data, there is no need of initialising them
// because they are only used after the callback executions.
ackermann_msgs::AckermannDriveStamped desired_ackermann_state; //!< Message to store the setpoints for the PID controllers
std_msgs::Float32MultiArray desired_speed_pid_gains; //!< Message to store the kp, ki, kd gains of the speed PID controller
std_msgs::Float32MultiArray desired_steering_pid_gains; //!< Message to store the kp, ki, kd gains of the steering PID controller
std_msgs::Float32MultiArray desired_limit_switches_position_LR; //!< Message to store the values of both LS angular position, useful during calibration


///////////////////////
// Outputs
int ros_interface_warning_code = NO_WARNING; //!< Variable sent in the robot_status message

ackermann_msgs::AckermannDriveStamped estimated_ackermann_state; //!< mean values of EKF estimated state: steering angle (deg) and speed (m/s)
ackermann_msgs::AckermannDriveStamped variances_of_estimated_ackermann_state; //!< variances of EKF estimated values
std_msgs::Float32MultiArray speed_volts_and_steering_pwm_being_applicated; //!< current outputs of both PID controllers
std_msgs::Float32MultiArray CLEAR_status; //!< message with five values, indicating operational mode, error code (only non zero in emergency mode), two warning codes and verbose level

// Echoes to check communications
ackermann_msgs::AckermannDriveStamped desired_ackermann_state_echo;
std_msgs::Float32MultiArray speed_pid_gains_echo;
std_msgs::Float32MultiArray steering_pid_gains_echo;
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

/*! \brief Callback to store the maximum forward recommended speed, it sets a flag to indicate that the safety system is alive
 *  and resets the watchdog
 */
void cb_maxForwardRecommendedSpeed(const std_msgs::Float32& max_recommended_forward_speed_msg)
{
  max_recommended_forward_speed = max_recommended_forward_speed_msg.data;
  reactive_watchdog = 0;
}

/*! \brief Callback to store the maximum recommended speed, it sets a flag to indicate that the safety system is alive
 *  and resets the watchdog
 */
void cb_maxBackwardRecommendedSpeed(const std_msgs::Float32& max_backward_recommended_speed_msg)
{
  max_recommended_backward_speed = max_backward_recommended_speed_msg.data;
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
  if (myRobot.getOperationalMode() == ROS_CONTROL)
  {
    //We copy the incoming message
    desired_ackermann_state.header = desired_ackermann_state_msg.header;
    desired_ackermann_state.drive = desired_ackermann_state_msg.drive;
    ros_interface_warning_code = NO_WARNING;
  }
  else
  {
    ros_interface_warning_code = RECEIVING_ROS_CONTROLS_WHILE_NOT_BEING_IN_ROS_MODE;
  }
}

/*! \brief CallBack to store the velocity control PID gains (kp, ki, and kd) coming from the on-board PC
 *
 * @param pid_gains_msg a message containing three floats, in order : kp, ki and kd
 */
void cb_velPIDGains(const std_msgs::Float32MultiArray& vel_pid_gains_msg)
{
  desired_speed_pid_gains = vel_pid_gains_msg;
  desired_speed_pid_gains.data_length = 3;

  myRobot.setSpeedPIDGains(desired_speed_pid_gains);
}

/*! \brief CallBack to read the steering control PID gains (kp, ki, and kd)
 *
 * @param pid_gains_msg a message containing three floats, in order: kp, ki and kd
 */
void cb_steeringPIDGains(const std_msgs::Float32MultiArray& ste_pid_gains_msg)
{
  desired_steering_pid_gains = ste_pid_gains_msg;
  desired_steering_pid_gains.data_length = 3;

  myRobot.setSteeringPIDGains(desired_steering_pid_gains);

}

/*! \brief CallBack to calibrate dynamically the steering limit switches angular position
 *
 * @param limit_switches_msg a message containing two floats, in order: left position and right position (expressed in degrees)
 */
void cb_limitSwitchesCalibration(const std_msgs::Float32MultiArray& limit_switches_msg)
{
  desired_limit_switches_position_LR = limit_switches_msg;
  desired_limit_switches_position_LR.data_length = 2;

  myRobot.setLimitSwitchesPositionLR(desired_limit_switches_position_LR);

}

/////////////////////////////////////////////
// Subscribers
/////////////////////////////////////////////

/*!
 * Subscriber that receives a code to adequate the amount of debugging information that the Arduino will produce
 */
ros::Subscriber<std_msgs::Int16> verbose_level_subscriber("desired_verbose_level", &cb_desiredVerboseLevel);

/*!
 * Subscriber that receives the maximum forward speed for the platform, it comes from the safety system
 */
ros::Subscriber<std_msgs::Float32> max_recommended_forward_speed_subscriber(
    "velocity_recommender_alg_node/forward_recommended_velocity", &cb_maxForwardRecommendedSpeed);

/*!
 * Subscriber that receives the maximum backward speed for the platform, it comes from the safety system
 */
ros::Subscriber<std_msgs::Float32> max_recommended_backward_speed_subscriber(
    "velocity_recommender_alg_node/backward_recommended_velocity", &cb_maxBackwardRecommendedSpeed);

/*!
 * Subscriber that receives the setpoint for the PID controllers (steering in deg and speed in m/s)
 */
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermann_subscriber("desired_ackermann_state",
                                                                            &cb_desiredAckermannState);

/*!
 * Subscriber that receives the gains for the PID speed controller
 */
ros::Subscriber<std_msgs::Float32MultiArray> vel_pid_gains_subscriber("desired_speed_controller_gains", &cb_velPIDGains);

/*!
 * Subscriber that receives the gains for the PID steering controller
 */
ros::Subscriber<std_msgs::Float32MultiArray> steering_pid_gains_subscriber("desired_steering_controller_gains",
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
ros::Publisher required_vel_pid_gains_publisher("echo_desired_vel_pid_gains", &speed_pid_gains_echo);

/*!
 * Publisher to produce an echo of the desired steering pid gains to check communications
 */
ros::Publisher required_steering_pid_gains_publisher("echo_desired_steering_pid_gains", &steering_pid_gains_echo);

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
ros::Publisher CLEAR_status_publisher("CLEAR_status", &CLEAR_status);


/////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////

/*!
 * \brief Fill the status data and publish the message
 */
void sendCLEARStatus(void)
{
  CLEAR_status.data[0] = myRobot.getOperationalMode();
  CLEAR_status.data[1] = myRobot.getErrorCode();
  CLEAR_status.data[2] = ros_interface_warning_code;
  CLEAR_status.data[3] = myRobot.getWarningCode();
  CLEAR_status.data[4] = verbose_level;

  CLEAR_status_publisher.publish(&CLEAR_status);
}

/*!
 * \brief Dynamic memory reserve for all array messages
 */
void reserveDynamicMemory(void)
{
  CLEAR_status.data_length = NUM_OF_CLEAR_STATUS_VARIABLES;
  CLEAR_status.data = (float *)malloc(sizeof(float) * NUM_OF_CLEAR_STATUS_VARIABLES);

  speed_volts_and_steering_pwm_being_applicated.data_length = NUM_OF_CONTROLLED_MOTORS;
  speed_volts_and_steering_pwm_being_applicated.data = (float *)malloc(sizeof(float) * NUM_OF_CONTROLLED_MOTORS);

  desired_speed_pid_gains.data_length = NUM_OF_PID_GAINS;
  desired_speed_pid_gains.data = (float *)malloc(sizeof(float) * NUM_OF_PID_GAINS);

  speed_pid_gains_echo.data_length = NUM_OF_PID_GAINS;
  speed_pid_gains_echo.data = (float *)malloc(sizeof(float) * NUM_OF_PID_GAINS);

  desired_steering_pid_gains.data_length = NUM_OF_PID_GAINS;
  desired_steering_pid_gains.data = (float *)malloc(sizeof(float) * NUM_OF_PID_GAINS);

  steering_pid_gains_echo.data_length = NUM_OF_PID_GAINS;
  steering_pid_gains_echo.data = (float *)malloc(sizeof(float) * NUM_OF_PID_GAINS);

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

  nh.advertise(CLEAR_status_publisher);
  nh.advertise(speed_volts_and_steering_pwm);

  nh.advertise(estimated_ackermann_publisher);
  nh.advertise(covariance_ackermann_publisher);

  nh.subscribe(max_recommended_forward_speed_subscriber);
  nh.subscribe(max_recommended_backward_speed_subscriber);
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
  if (myRobot.getOperationalMode() == ROS_CONTROL)
  {
    myRobot.updateROSDesiredState(desired_ackermann_state);
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
    myRobot.getDesiredState(desired_ackermann_state_echo);
    required_ackermann_publisher.publish(&desired_ackermann_state_echo);

    myRobot.getSpeedPIDGains(speed_pid_gains_echo);
    required_vel_pid_gains_publisher.publish(&speed_pid_gains_echo);

    myRobot.getSteeringPIDGains(steering_pid_gains_echo);
    required_steering_pid_gains_publisher.publish(&steering_pid_gains_echo);

    myRobot.getLimitSwitchesPositionLR(echo_desired_limit_switches_position_LR);
    required_limit_switches_position_publisher.publish(&echo_desired_limit_switches_position_LR);
  }
  if (verbose_level >= MIN_VERBOSE_LEVEL)
  {
    sendCLEARStatus();
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

  myRobot.updateState(estimated_ackermann_state, variances_of_estimated_ackermann_state); //!< EKF iteration

  myRobot.readOnBoardUserInterface(); //!< Read the controls available on-board (currently only the emergency switch)

  myRobot.readRemoteControl(); //!< Decode and map the RC signals

  int millisSinceLastReactiveUpdate = reactive_watchdog; //!< Created to pass the safety watchdog value to the finite state machine
  myRobot.updateFiniteStateMachine(millisSinceLastReactiveUpdate);

  myRobot.calculateCommandOutputs(max_recommended_forward_speed, max_recommended_backward_speed); //!< It uses the reactive safety system speed limitation

  myRobot.writeCommandOutputs(speed_volts_and_steering_pwm_being_applicated); //!< Here we actuate the motors

  if (communicate_with_ROS) sendOutputsToROS(); //!< publishing the ROS interface topics
}
