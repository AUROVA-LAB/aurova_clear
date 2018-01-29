/*
 * vehicle.cpp
 *
 *  Created on: 3 Nov 2017
 *      Author: idelpino
 */

#include <elapsedMillis.h>

#include "../headers/vehicle.h"
#include "../headers/arduino_ros_interface.h"
#include "../headers/hardware_description_constants.h"
#include "../headers/configuration_vehicle_hardware.h"


elapsedMillis timeLastComputeSteering = SAMPLING_TIME_TEENSY/2;
elapsedMillis timeLastComputeSpeed = 0;

float* speed_measures;
float* steering_measures;


Vehicle::Vehicle()
{
  operational_mode_ = CALIBRATION;

  measured_state_.steering_angle = 0.0;           // radians
  measured_state_.steering_angle_velocity = 0.0;  // r/s

  measured_state_.speed = 0.0;                    // m/s
  measured_state_.acceleration = 0.0;             // m/s^2
  measured_state_.jerk = 0.0;                     // m/s^3

  desired_state_.steering_angle = 0.0;            // radians
  desired_state_.steering_angle_velocity = 0.0;   // r/s

  desired_state_.speed = 0.0;                     // m/s
  desired_state_.acceleration = 0.0;              // m/s^2
  desired_state_.jerk = 0.0;                      // m/s^3

  desired_steering_state_reached_     = false;
  desired_traslational_state_reached_ = false;

  remote_control_.speed_volts = 0.0;
  remote_control_.steering_angle_pwm = 0.0;

  error_code_ = NO_ERROR;

  speed_volts_ = 0.0;
  steering_angle_pwm_ = 0.0;

  dBus_ = new DJI_DBUS(RC_PORT);
  dBus_->begin();

  speed_actuator_ = new SpeedHardwareInterface(PIN_CH1, PIN_CH2);
  steering_actuator_ = new SteeringHardwareInterface();

  speed_controller_.setPIDMaxMinValues(MAX_SPEED_CONTROLLER_OUTPUT, MIN_SPEED_CONTROLLER_OUTPUT);

  steering_controller_.setPIDMaxMinValues(MAX_STEERING_CONTROLLER_OUTPUT, MIN_STEERING_CONTROLLER_OUTPUT);

}

Vehicle::~Vehicle()
{

}

bool Vehicle::componentsCalibration()
{
	if(steering_actuator_->steeringCalibration())
		return true;
	else
		return false;
	return true;
}

void Vehicle::setVelocityPIDGains(const std_msgs::Float32MultiArray& desired_pid_gains)
{
  float kp = desired_pid_gains.data[0];
  float ki = desired_pid_gains.data[1];
  float kd = desired_pid_gains.data[2];

  speed_controller_.setPIDGains(kp, ki, kd);
}

void Vehicle::getVelocityPIDGains(std_msgs::Float32MultiArray& current_pid_gains)
{
  float kp, ki, kd;

  speed_controller_.getPIDGains(kp, ki, kd);

  current_pid_gains.data[0] = kp;
  current_pid_gains.data[1] = ki;
  current_pid_gains.data[2] = kd;
}

void Vehicle::setSteeringPIDGains(const std_msgs::Float32MultiArray& desired_pid_gains)
{
  float kp = desired_pid_gains.data[0];
  float ki = desired_pid_gains.data[1];
  float kd = desired_pid_gains.data[2];

  steering_controller_.setPIDGains(kp, ki, kd);
}

void Vehicle::getSteeringPIDGains(std_msgs::Float32MultiArray& current_pid_gains)
{
  float kp, ki, kd;

  steering_controller_.getPIDGains(kp, ki, kd);

  current_pid_gains.data[0] = kp;
  current_pid_gains.data[1] = ki;
  current_pid_gains.data[2] = kd;
}

void Vehicle::getOperationalMode(int& current_operational_mode)
{
  current_operational_mode = operational_mode_;
}

void Vehicle::updateFiniteStateMachine(void)
{
  switch ( operational_mode_ )
  {
    case RESET:

      if ( desired_steering_state_reached_ )
      {
        operational_mode_ = REMOTE_CONTROL;
      }
      break;

    case EMERGENCY_STOP:

      break;

    case REMOTE_CONTROL:

      break;

    case ROS_CONTROL:

      break;

    case CALIBRATION:
    	if( componentsCalibration() )
          operational_mode_ = REMOTE_CONTROL;

    	break;

  }
}

int Vehicle::getOperationalMode(void)
{
  return (operational_mode_);
}

void Vehicle::updateMeasuredState(ackermann_msgs::AckermannDriveStamped& measured_ackermann_state)
{
  if(timeLastComputeSteering > SAMPLING_TIME_TEENSY)
  {
	steering_measures = steering_actuator_->getSteeringMeasures();
	timeLastComputeSteering = 0;
  }
  if(timeLastComputeSpeed > SAMPLING_TIME_TEENSY)
  {
    speed_measures = speed_actuator_->getSpeedMeasures();
	timeLastComputeSpeed = 0;
  }

  measured_state_.steering_angle = steering_measures[0]*PULSES_TO_DEG;
  measured_state_.steering_angle_velocity = steering_measures[1]*PULSES_TO_DEG;

  measured_state_.speed = speed_measures[0]*PULSES_PER_METER;
  measured_state_.acceleration = speed_measures[1]*PULSES_PER_METER;
  measured_state_.jerk = speed_measures[2]*PULSES_PER_METER;

  measured_ackermann_state.drive.speed = measured_state_.speed;
  measured_ackermann_state.drive.acceleration = measured_state_.acceleration;
  measured_ackermann_state.drive.jerk = measured_state_.jerk;

  measured_ackermann_state.drive.steering_angle = measured_state_.steering_angle;
  measured_ackermann_state.drive.steering_angle_velocity = measured_state_.steering_angle_velocity;


}

void Vehicle::updateDesiredState(const ackermann_msgs::AckermannDriveStamped& desired_ackermann_state)
{
  desired_state_.steering_angle = desired_ackermann_state.drive.steering_angle;
  desired_state_.steering_angle_velocity = desired_ackermann_state.drive.steering_angle_velocity;

  desired_state_.speed = desired_ackermann_state.drive.speed;
  desired_state_.acceleration = desired_ackermann_state.drive.acceleration;
  desired_state_.jerk = desired_ackermann_state.drive.jerk;
}

void Vehicle::calculateCommandOutputs(void)
{
  switch (operational_mode_)
  {
    case REMOTE_CONTROL:

      speed_volts_ = remote_control_.speed_volts;
      steering_angle_pwm_ = remote_control_.steering_angle_pwm;

      break;

    case ROS_CONTROL:

      //TODO: Some PID control (using velocity, acceleration, jerk...)
      speed_volts_ = SPEED_ZERO;
      steering_angle_pwm_ = STEERING_CENTERED;

      break;

    case EMERGENCY_STOP:

      speed_volts_ = SPEED_ZERO;
      steering_angle_pwm_ = STEERING_CENTERED;

      break;

    case RESET:

      speed_volts_ = SPEED_ZERO;
      //TODO: Use the PID controllers to centre the steering
      steering_angle_pwm_ = STEERING_CENTERED;

      break;
  }
}

void Vehicle::readRemoteControl(void)
{
  dBus_->FeedLine();
  if (dBus_->toChannels == 1)
  {
    dBus_->UpdateChannels();
    dBus_->toChannels = 0;

    /*
     * The velocity is controlled by the Rudder of the remote control
     * The range of the Rudder is 364 (backward) and 1684 (forward) both with the maximun speed
     * Any other number out of this range stop the vehicle
     */
    if (dBus_->failsafe_status == DBUS_SIGNAL_OK)
    {
      remote_control_.speed_volts = mapFloat(dBus_->channels[2], 364.0, 1684.0, -ABS_MAX_SPEED_VOLTS, ABS_MAX_SPEED_VOLTS);
      remote_control_.steering_angle_pwm = mapFloat(dBus_->channels[0], 364.0, 1684.0, -ABS_MAX_STEERING_MOTOR_PWM, ABS_MAX_STEERING_MOTOR_PWM);

    }
    else
    {
      remote_control_.speed_volts = 0.0;
      remote_control_.steering_angle_pwm = 0.0;
    }
  }
}

void Vehicle::readOnBoardUserInterface(void)
{

}

void Vehicle::writeCommandOutputs(std_msgs::Float32MultiArray& speed_volts_and_steering_pwm_being_applicated)
{
  //bool ste_error = false;
  //bool speed_error = false;

  if ( operational_mode_ != EMERGENCY_STOP && operational_mode_ != CALIBRATION)
  {
    speed_actuator_->actuateMotor(speed_volts_);
    steering_actuator_->steeringMotor(steering_angle_pwm_);
    //ste_error = steering_actuator_->steering_motor(steering_angle_pwm_);

    //if(ste_error) error_code_ = STEERING_CONTROL_ERROR;

    //Passing the applied values to communicate them to ROS
    speed_volts_and_steering_pwm_being_applicated.data[0] = speed_volts_;
    speed_volts_and_steering_pwm_being_applicated.data[1] = steering_angle_pwm_;
  }
}

void Vehicle::updatePIDGains(const std_msgs::Float32MultiArray& desired_vel_pid_gains,
                             const std_msgs::Float32MultiArray& desired_ste_pid_gains)
{
  setVelocityPIDGains(desired_vel_pid_gains);
  setSteeringPIDGains(desired_ste_pid_gains);
}

void Vehicle::getErrorCode(int& requested_error_code)
{
  requested_error_code = error_code_;
}

int Vehicle::getErrorCode(void)
{
  return (error_code_);
}
