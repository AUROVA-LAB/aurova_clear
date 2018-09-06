/*
 * vehicle.cpp
 *
 *  Created on: 3 Nov 2017
 *      Author: idelpino
 */

#include <elapsedMillis.h>
#include "vehicle.h"
#include "arduino_ros_interface.h"
#include "hardware_description_constants.h"
#include "configuration_vehicle_hardware.h"
#include "pid.h"

Vehicle::Vehicle()
{
  operational_mode_                        = REMOTE_CONTROL_NOT_SAFE;
  last_operational_mode_                   = operational_mode_;

  estimated_state_.steering_angle          = 0.0;  // deg
  estimated_state_.steering_angle_velocity = 0.0;  // deg/s

  estimated_state_.speed                   = 0.0;  // m/s
  estimated_state_.acceleration            = 0.0;  // m/s^2
  estimated_state_.jerk                    = 0.0;  // m/s^3

  measured_state_.steering_angle           = 0.0;  // deg
  measured_state_.steering_angle_velocity  = 0.0;  // deg/s

  measured_state_.speed                    = 0.0;  // m/s
  measured_state_.acceleration             = 0.0;  // m/s^2
  measured_state_.jerk                     = 0.0;  // m/s^3

  desired_state_.steering_angle            = 0.0;  // deg
  desired_state_.steering_angle_velocity   = 0.0;  // deg/s

  desired_state_.speed                     = 0.0;  // m/s
  desired_state_.acceleration              = 0.0;  // m/s^2
  desired_state_.jerk                      = 0.0;  // m/s^3

  remote_control_.speed_volts                           = 0.0;
  remote_control_.steering_angle_pwm                    = 0.0;

  remote_control_.desired_state.steering_angle          = 0.0;   // deg
  remote_control_.desired_state.steering_angle_velocity = 0.0;   // deg/s

  remote_control_.desired_state.speed                   = 0.0;   // m/s
  remote_control_.desired_state.acceleration            = 0.0;   // m/s^2
  remote_control_.desired_state.jerk                    = 0.0;   // m/s^3

  error_code_ = NO_ERROR;

  speed_volts_        = 0.0;
  steering_angle_pwm_ = 0.0;

  speed_volts_pid_        = 0.0;
  steering_angle_pwm_pid_ = 0.0;

  left_steering_limit_switch_position_  = ABS_MAX_LEFT_ANGLE_DEG;
  right_steering_limit_switch_position_ = ABS_MAX_RIGHT_ANGLE_DEG;

  flag_limiting_speed_by_reactive_  = false;
  remote_control_use_PID_ = true;

  dBus_ = new DJI_DBUS(RC_PORT);
  dBus_->begin();

  speed_measures_    = NULL;
  steering_measures_ = NULL;

  speed_actuator_    = new SpeedHardwareInterface(PIN_CH1, PIN_CH2);
  steering_actuator_ = new SteeringHardwareInterface();

  speed_actuator_->  actuateMotor(0.0);  // To ensure that in the start the output voltage is equal to zero

  steering_actuator_->steeringMotor(0);
  steering_actuator_->steering_encoder_->encoderReset(11);

  speed_controller_ = new PID(&estimated_state_.speed, &speed_volts_pid_, &desired_state_.speed, SPEED_KP, SPEED_KI,
                              SPEED_KD, MIN_VOLTS_TO_ACTUATE_MOTOR);

  speed_controller_->setOutputLimits(NORMALIZING_PID_MIN_VALUE, NORMALIZING_PID_MAX_VALUE);

  steering_controller_ = new PID(&estimated_state_.steering_angle, &steering_angle_pwm_pid_,
                                 &desired_state_.steering_angle, STEERING_KP, STEERING_KI, STEERING_KD,
                                 MIN_PWM_TO_ACTUATE_MOTOR);

  steering_controller_->setOutputLimits(NORMALIZING_PID_MIN_VALUE, NORMALIZING_PID_MAX_VALUE);

  state_estimator_ = new EKF();

  led_rgb_value_[0] = 0;
  led_rgb_value_[1] = 0;
  led_rgb_value_[2] = 0;

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(HORN, OUTPUT);
  digitalWrite(HORN, HIGH); //! Horn in silence

  pinMode(ENABLE_MOTORS, OUTPUT);
  digitalWrite(ENABLE_MOTORS, HIGH); //! Motors disabled

  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH); //! Brakes activated

  //desynchronising sampling times for I2C
  if (SAMPLING_TIME_SPEED_ > SAMPLING_TIME_STEERING_)
  {
    millis_since_last_steering_reading_ = SAMPLING_TIME_STEERING_ * (float)(fabs(SAMPLING_TIME_STEERING_ - SAMPLING_TIME_SPEED_))
        / SAMPLING_TIME_SPEED_;
  }
  else if (SAMPLING_TIME_STEERING_ > SAMPLING_TIME_SPEED_)
  {
    millis_since_last_speed_reading_ = SAMPLING_TIME_SPEED_ * (float)(fabs(SAMPLING_TIME_STEERING_ - SAMPLING_TIME_SPEED_))
        / SAMPLING_TIME_STEERING_;
  }
  else
    millis_since_last_steering_reading_ = SAMPLING_TIME_STEERING_ / 2.0;

}

Vehicle::~Vehicle()
{

}

//////////////////////////
// Private interface
//////////////////////////
void Vehicle::saturateSetpointIfNeeded(float &speed)
{
  if (speed > ABS_MAX_SPEED_METERS_SECOND)
    speed = ABS_MAX_SPEED_METERS_SECOND;
  else if (speed < -1 * ABS_MAX_SPEED_METERS_SECOND)
    speed = -1 * ABS_MAX_SPEED_METERS_SECOND;
}

void Vehicle::resetSpeed(void)
{
  speed_controller_->resetPID();
}

void Vehicle::resetSteering(void)
{
  steering_controller_->resetPID();
}


////////////////////////////////////////
// Public interface
////////////////////////////////////////
void Vehicle::updateROSDesiredState(const ackermann_msgs::AckermannDriveStamped& desired_ackermann_state)
{
  desired_state_.steering_angle = desired_ackermann_state.drive.steering_angle;
  desired_state_.steering_angle_velocity = desired_ackermann_state.drive.steering_angle_velocity;

  desired_state_.speed = desired_ackermann_state.drive.speed;
  saturateSetpointIfNeeded(desired_state_.speed);

  desired_state_.acceleration = desired_ackermann_state.drive.acceleration;
  desired_state_.jerk = desired_ackermann_state.drive.jerk;

}

void Vehicle::updateState(ackermann_msgs::AckermannDriveStamped& estimated_ackermann_state,
                          ackermann_msgs::AckermannDriveStamped& covariance_ackermann_state)
{
  if (millis_since_last_EKF_prediction_ >= TIME_TO_PREDICT_MILLIS_)
  {
    state_estimator_->predict(speed_volts_); //!< Make EKF prediction step
    millis_since_last_EKF_prediction_ = 0; //!< Reset the counter
  }

  if (millis_since_last_steering_reading_ >= SAMPLING_TIME_STEERING_)
  {
    steering_measures_ = steering_actuator_->getSteeringMeasures(); //!< Read the steering encoder pulses

    measured_state_.steering_angle_velocity = steering_measures_[1] * PULSES_TO_DEG; //!< Pass to degrees
    measured_state_.steering_angle = steering_measures_[0] * PULSES_TO_DEG;
    millis_since_last_steering_reading_ = 0; //!< Reset the counter

    state_estimator_->correctEnc(measured_state_.steering_angle); //!< Make EKF correction using the incremental steering observation
  }

  if (millis_since_last_speed_reading_ >= SAMPLING_TIME_SPEED_)
  {
    speed_measures_ = speed_actuator_->getSpeedMeasures(); //!< Read the speed encoder pulses
    millis_since_last_speed_reading_ = 0; //!< Reset the counter

    float direction = 1.0;
    if (!speed_actuator_->getFlagForward())
      direction = -1.0;

    float speed_left_rear_wheel = speed_measures_[0] * METERS_PER_PULSE * direction; //!< Pass to meters per second

    measured_state_.speed = speed_left_rear_wheel; //!< The encoder is attached to the left rear wheel
    measured_state_.acceleration = speed_measures_[1] * METERS_PER_PULSE;
    measured_state_.jerk = speed_measures_[2] * METERS_PER_PULSE;

    state_estimator_->correctHall(speed_left_rear_wheel); //!< Make EKF correction using speed observation
  }

  if (steering_actuator_->readLimitSwitches()) //!< If any steering limit switch is reached
  {
    float observed_theta = 0.0;
    if (estimated_state_.steering_angle > 0.0)
    {
      observed_theta = left_steering_limit_switch_position_;
    }
    else
    {
      observed_theta = -1 * right_steering_limit_switch_position_;
    }
    state_estimator_->correctLs(observed_theta); //!< Make EKF correction using absolute steering observation
  }
  /*! EKF state consists in calibration error (or steering in k=0),
   * accumulated steering angle (integrating the incremental encoder) and speed
   * */
  float steering_calibration_error = 0.0;
  float steering_cummulated_increment = 0.0;
  state_estimator_->getState(steering_calibration_error, steering_cummulated_increment, estimated_state_.speed);

  estimated_state_.steering_angle = steering_calibration_error + steering_cummulated_increment;
  estimated_state_.steering_angle_velocity = measured_state_.steering_angle_velocity;

  float steering_calibration_error_variance = 0.0;
  float steering_cummulated_increment_variance = 0.0;
  state_estimator_->getVariances(steering_calibration_error_variance, steering_cummulated_increment_variance,
                                 covariance_ackermann_state.drive.speed);

  covariance_ackermann_state.drive.steering_angle = steering_calibration_error_variance
      + steering_cummulated_increment_variance;
  covariance_ackermann_state.drive.steering_angle_velocity = 0.0;

  estimated_state_.acceleration = steering_calibration_error;
  estimated_state_.jerk = steering_cummulated_increment;
  covariance_ackermann_state.drive.acceleration = steering_calibration_error_variance;
  covariance_ackermann_state.drive.jerk = steering_cummulated_increment_variance;

  // Passing to messages
  estimated_ackermann_state.drive.speed = estimated_state_.speed;
  estimated_ackermann_state.drive.acceleration = estimated_state_.acceleration;
  estimated_ackermann_state.drive.jerk = estimated_state_.jerk;

  estimated_ackermann_state.drive.steering_angle = estimated_state_.steering_angle;
  estimated_ackermann_state.drive.steering_angle_velocity = estimated_state_.steering_angle_velocity;
}

void Vehicle::readOnBoardUserInterface(void)
{
  if (digitalRead(ON_BOARD_EMERGENCY_SWITCH) == LOW)
    operational_mode_ = EMERGENCY_STOP;
}

void Vehicle::readRemoteControl(void)
{
  dBus_->FeedLine(); //! Makes a memcopy of the RC buffer content

  dBus_->UpdateSignalState(); //! Check if the Remote Controller is alive
  if (dBus_->failsafe_status != DBUS_SIGNAL_OK)
  {
    operational_mode_ = EMERGENCY_STOP;
  }
  else
  {
    if (dBus_->toChannels == RC_NEW_DATA_AVAILABLE) //! If there are new data to read
    {
      dBus_->UpdateChannels(); //! Load the new data
      dBus_->toChannels = RC_NEW_DATA_READED; //! Reset the flag to indicate "ready to receive new data"


      // We map the RC control rod positions to volts, pwm, meters per second and steering degrees, depending on the flag use_PID_
      // the motors will use either the raw volts and pwm or the output of the PID controllers, that use the RC speed and steering
      // as setpoints

      remote_control_.speed_volts = mapFloat(dBus_->channels[RC_SPEED_CONTROL_ROD],
                                             RC_MIN_CONTROL_ROD_VALUE, RC_MAX_CONTROL_ROD_VALUE,
                                             -ABS_MAX_SPEED_VOLTS, ABS_MAX_SPEED_VOLTS);

      remote_control_.steering_angle_pwm = mapFloat(dBus_->channels[RC_STEERING_CONTROL_ROD],
                                                    RC_MIN_CONTROL_ROD_VALUE, RC_MAX_CONTROL_ROD_VALUE,
                                                    ABS_MAX_STEERING_MOTOR_PWM, -1 * ABS_MAX_STEERING_MOTOR_PWM);

      remote_control_.desired_state.speed = mapFloat(dBus_->channels[RC_SPEED_CONTROL_ROD],
                                                     RC_MIN_CONTROL_ROD_VALUE, RC_MAX_CONTROL_ROD_VALUE,
                                                     -ABS_MAX_SPEED_METERS_SECOND, ABS_MAX_SPEED_METERS_SECOND);

      remote_control_.desired_state.steering_angle = mapFloat(dBus_->channels[RC_STEERING_CONTROL_ROD],
                                                              RC_MIN_CONTROL_ROD_VALUE, RC_MAX_CONTROL_ROD_VALUE,
                                                              ABS_MAX_STEERING_ANGLE_DEG, -ABS_MAX_STEERING_ANGLE_DEG);
      // Operational mode switching
      if (operational_mode_ != EMERGENCY_STOP) //! During normal operation we switch between modes just reading the RC switches
      {
        if (dBus_->channels[RC_EMERGENCY_SWITCH_AND_DISABLE_PID] == RC_EMERGENCY)
          operational_mode_ = EMERGENCY_STOP;
        else if (dBus_->channels[RC_OPERATIONAL_MODE_SWITCH] == RC_ROS_MODE)
          operational_mode_ = ROS_CONTROL;
        else if (dBus_->channels[RC_OPERATIONAL_MODE_SWITCH] == RC_SAFETY_SYSTEM_DISABLED)
          operational_mode_ = REMOTE_CONTROL_NOT_SAFE;
        else
          operational_mode_ = REMOTE_CONTROL;
      }else{ //! To exit from emergency mode we need to check four conditions
        if (operational_mode_ == EMERGENCY_STOP and
            dBus_->channels[RC_OPERATIONAL_MODE_SWITCH] == RC_SAFETY_SYSTEM_DISABLED and
            dBus_->channels[RC_EMERGENCY_SWITCH_AND_DISABLE_PID] == RC_NO_EMERGENCY and
            dBus_->channels[RC_REARM_AND_HORN_CONTROL] == RC_REARM and
            digitalRead(ON_BOARD_EMERGENCY_SWITCH) == HIGH) // The first three are from the RC while the last one reads the on-board emergency switch
        {
          operational_mode_ = REMOTE_CONTROL_NOT_SAFE; //! We always go to full manual when exiting from emergency
        }
      }

      if (dBus_->channels[RC_EMERGENCY_SWITCH_AND_DISABLE_PID] == RC_DISABLE_PID && operational_mode_ == REMOTE_CONTROL_NOT_SAFE)
      {
        remote_control_use_PID_ = false;
      }
      else
      {
        remote_control_use_PID_ = true;
      }

      // Horn
      if (dBus_->channels[RC_REARM_AND_HORN_CONTROL] == RC_ACTIVATE_HORN)
      {
        digitalWrite(HORN, LOW); // Let's horn!
      }
      else
      {
        digitalWrite(HORN, HIGH); // Keep silence
      }
    }
  }
}

void Vehicle::updateFiniteStateMachine(int millisSinceLastReactiveUpdate)
{
  if (operational_mode_ != last_operational_mode_) //! Every time that a mode change happens we reset the PID controllers, for a fresh new start
  {
    resetSpeed();
    resetSteering();
    last_operational_mode_ = operational_mode_;
  }

  if (operational_mode_ != REMOTE_CONTROL_NOT_SAFE &&
      millisSinceLastReactiveUpdate > MAX_TIME_WITHOUT_REACTIVE_MILLIS)
  {
    operational_mode_ = EMERGENCY_STOP;
  }

  switch (operational_mode_)
  {
    case EMERGENCY_STOP:
      led_rgb_value_[0] = 255;
      led_rgb_value_[1] = 0;
      led_rgb_value_[2] = 0;
      break;

    case REMOTE_CONTROL:
      led_rgb_value_[0] = 0;
      led_rgb_value_[1] = 255;
      led_rgb_value_[2] = 0;
      break;

    case REMOTE_CONTROL_NOT_SAFE:
      led_rgb_value_[0] = 255;
      led_rgb_value_[1] = 255;
      led_rgb_value_[2] = 255;
      break;

    case ROS_CONTROL:
      led_rgb_value_[0] = 0;
      led_rgb_value_[1] = 0;
      led_rgb_value_[2] = 255;
      break;
  }

  if (flag_limiting_speed_by_reactive_ || !remote_control_use_PID_)
  {
    led_rgb_value_[0] = 255;
    led_rgb_value_[1] = 255;
    led_rgb_value_[2] = 0;
  }

  analogWrite(LED_R, led_rgb_value_[0]);
  analogWrite(LED_G, led_rgb_value_[1]);
  analogWrite(LED_B, led_rgb_value_[2]);
}

void Vehicle::calculateCommandOutputs(float max_recommended_speed)
{
  if (operational_mode_ == REMOTE_CONTROL_NOT_SAFE || operational_mode_ == REMOTE_CONTROL)
  {
    desired_state_.speed = remote_control_.desired_state.speed;
    desired_state_.steering_angle = remote_control_.desired_state.steering_angle;
  }

  flag_limiting_speed_by_reactive_ = false;
  if (operational_mode_ != REMOTE_CONTROL_NOT_SAFE && desired_state_.speed > max_recommended_speed)
  {
    desired_state_.speed = max_recommended_speed;
    flag_limiting_speed_by_reactive_ = true;
  }
  saturateSetpointIfNeeded(desired_state_.speed);
  if (fabs(desired_state_.speed) <= MIN_SETPOINT_TO_USE_PID)
  {
    desired_state_.speed = 0.0;
    speed_volts_pid_ = 0.0;
    resetSpeed();
  }
  else
  {
    speed_controller_->computePID(ABS_MAX_SPEED_METERS_SECOND, ABS_MAX_SPEED_METERS_SECOND, ABS_MAX_SPEED_VOLTS);
  }
  steering_controller_->computePID(ABS_MAX_STEERING_ANGLE_DEG, ABS_MAX_STEERING_ANGLE_DEG,
                                   ABS_MAX_STEERING_MOTOR_PWM);
}

void Vehicle::writeCommandOutputs(std_msgs::Float32MultiArray& speed_volts_and_steering_pwm_being_applicated)
{
  if (operational_mode_ == EMERGENCY_STOP)
  {
    digitalWrite(ENABLE_MOTORS, HIGH); //! Disable motors
    digitalWrite(BRAKE, HIGH); //! Activate brakes
    speed_volts_ = 0; //! Just in case
    steering_angle_pwm_ = 0; //! Just in case
  }
  else
  {
    digitalWrite(ENABLE_MOTORS, LOW); //! Enable motors

    if (operational_mode_ == REMOTE_CONTROL_NOT_SAFE && !remote_control_use_PID_)
    {
      speed_volts_ = remote_control_.speed_volts;
      steering_angle_pwm_ = remote_control_.steering_angle_pwm;
    }
    else
    {
      speed_volts_ = speed_volts_pid_;
      steering_angle_pwm_ = steering_angle_pwm_pid_;
    }

    if (fabs(speed_volts_) > MIN_VOLTS_TO_RELEASE_BRAKE)
    {
      digitalWrite(BRAKE, LOW); //! Release brakes
      zero_volts_millis_before_braking_ = 0;
    }
  }

  if (zero_volts_millis_before_braking_ > MAX_TIME_ZERO_VOLTS_TO_BRAKE)
  {
    digitalWrite(BRAKE, HIGH);
  }

  //! Actuating motors!
  speed_actuator_->actuateMotor(speed_volts_);
  steering_actuator_->steeringMotor(steering_angle_pwm_);

  //Passing the applied values to communicate them to ROS
  speed_volts_and_steering_pwm_being_applicated.data[0] = speed_volts_;
  speed_volts_and_steering_pwm_being_applicated.data[1] = steering_angle_pwm_;

}



/////////////////////////////////////////////
// Setters and getters
/////////////////////////////////////////////
void Vehicle::getDesiredState(ackermann_msgs::AckermannDriveStamped& desired_ackermann_state_echo)
{
  desired_ackermann_state_echo.drive.steering_angle = desired_state_.steering_angle;
  desired_ackermann_state_echo.drive.steering_angle_velocity = desired_state_.steering_angle_velocity;

  desired_ackermann_state_echo.drive.speed = desired_state_.speed;
  desired_ackermann_state_echo.drive.acceleration = desired_state_.acceleration;
  desired_ackermann_state_echo.drive.jerk = desired_state_.jerk;
}

void Vehicle::setSpeedAndSteeringPIDGains(const std_msgs::Float32MultiArray& desired_vel_pid_gains,
                             const std_msgs::Float32MultiArray& desired_ste_pid_gains)
{
  setSpeedPIDGains(desired_vel_pid_gains);
  setSteeringPIDGains(desired_ste_pid_gains);
}

void Vehicle::setSpeedPIDGains(const std_msgs::Float32MultiArray& desired_pid_gains)
{
  float kp = desired_pid_gains.data[0];
  float ki = desired_pid_gains.data[1];
  float kd = desired_pid_gains.data[2];

  speed_controller_->setPIDGains(kp, ki, kd);
}

void Vehicle::getSpeedPIDGains(std_msgs::Float32MultiArray& current_pid_gains)
{
  speed_controller_->getPIDGains(current_pid_gains.data[0], current_pid_gains.data[1], current_pid_gains.data[2]);
}

void Vehicle::setSteeringPIDGains(const std_msgs::Float32MultiArray& desired_pid_gains)
{
  float kp = desired_pid_gains.data[0];
  float ki = desired_pid_gains.data[1];
  float kd = desired_pid_gains.data[2];

  steering_controller_->setPIDGains(kp, ki, kd);
}

void Vehicle::getSteeringPIDGains(std_msgs::Float32MultiArray& current_pid_gains)
{
  steering_controller_->getPIDGains(current_pid_gains.data[0], current_pid_gains.data[1], current_pid_gains.data[2]);
}

void Vehicle::setLimitSwitchesPositionLR(std_msgs::Float32MultiArray& desired_limit_switches_position)
{
  left_steering_limit_switch_position_ = desired_limit_switches_position.data[0];
  right_steering_limit_switch_position_ = desired_limit_switches_position.data[1];
}

void Vehicle::getLimitSwitchesPositionLR(std_msgs::Float32MultiArray& current_limit_switches_position)
{
  current_limit_switches_position.data[0] = left_steering_limit_switch_position_;
  current_limit_switches_position.data[1] = right_steering_limit_switch_position_;
}

void Vehicle::getOperationalMode(int& current_operational_mode)
{
  current_operational_mode = operational_mode_;
}

int Vehicle::getOperationalMode(void)
{
  return (operational_mode_);
}

void Vehicle::getErrorCode(int& requested_error_code)
{
  requested_error_code = error_code_;
}

int Vehicle::getErrorCode(void)
{
  return (error_code_);
}
