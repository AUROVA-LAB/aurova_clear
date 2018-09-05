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
  operational_mode_                        = REMOTE_CONTROL;
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
  flag_speed_recommendation_active_ = false;

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
  digitalWrite(HORN, HIGH);

  pinMode(ENABLE_MOTORS, OUTPUT);
  digitalWrite(ENABLE_MOTORS, LOW);

  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

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
    state_estimator_->predict(speed_volts_);
    millis_since_last_EKF_prediction_ = 0;
  }

  if (millis_since_last_steering_reading_ >= SAMPLING_TIME_STEERING_)
  {
    steering_measures_ = steering_actuator_->getSteeringMeasures();

    measured_state_.steering_angle_velocity = steering_measures_[1] * PULSES_TO_DEG;
    measured_state_.steering_angle = steering_measures_[0] * PULSES_TO_DEG;
    millis_since_last_steering_reading_ = 0;
    if (fabs(steering_measures_[0]) < 170)
      Serial.println(steering_measures_[0]);
    state_estimator_->correctEnc(measured_state_.steering_angle);
  }

  if (millis_since_last_speed_reading_ >= SAMPLING_TIME_SPEED_)
  {
    speed_measures_ = speed_actuator_->getSpeedMeasures();
    millis_since_last_speed_reading_ = 0;

    float direction = 1.0;
    if (!speed_actuator_->getFlagForward())
      direction = -1.0;

    float speed_left_rear_wheel = speed_measures_[0] * METERS_PER_PULSE * direction;

    measured_state_.speed = speed_left_rear_wheel;
    measured_state_.acceleration = speed_measures_[1] * METERS_PER_PULSE; // TODO--> check if direction is also needed here
    measured_state_.jerk = speed_measures_[2] * METERS_PER_PULSE;

    state_estimator_->correctHall(speed_left_rear_wheel);
  }

  //Serial.println(steering_actuator_->readLimitSwitches());
  //int ls = 0;
  //              steering_actuator_->readLimitSwitches();
  if (steering_actuator_->readLimitSwitches())
  {
    //Serial.print("Limit!!");
    float observed_theta = 0.0;
//        if(ls==1)
//        {
//                observed_theta = left_steering_limit_switch_position_;
//                //measured_state_.steering_angle = observed_theta;
//                //Serial.println("left limit!!");
//        }
//        if(ls==2)
//        {
//                observed_theta = -1 * right_steering_limit_switch_position_;
//                //measured_state_.steering_angle = observed_theta;
//                //Serial.println("right limit!!");
//        }
//      //Serial.print("Steering angle = ");
//      //Serial.println(measured_state_.steering_angle);
//        //state_estimator_->correctLs(observed_theta);
    if (estimated_state_.steering_angle > 0.0)
    {
      observed_theta = left_steering_limit_switch_position_;
      //Serial.println("left limit!!");
    }
    else
    {
      observed_theta = -1 * right_steering_limit_switch_position_;
      //Serial.println("right limit!!");
    }
    Serial.println(measured_state_.steering_angle);
    //Serial.println(steering_measures[0]);
    state_estimator_->correctLs(observed_theta);

  }

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

  if (!USE_KALMAN_FILTER)
  {
    estimated_state_.steering_angle = measured_state_.steering_angle;
    estimated_state_.steering_angle_velocity = measured_state_.steering_angle_velocity;
    estimated_state_.speed = measured_state_.speed;
  }

  // Passing to messages
  estimated_ackermann_state.drive.speed = estimated_state_.speed;
  estimated_ackermann_state.drive.acceleration = estimated_state_.acceleration;
  estimated_ackermann_state.drive.jerk = estimated_state_.jerk;

  estimated_ackermann_state.drive.steering_angle = estimated_state_.steering_angle;
  estimated_ackermann_state.drive.steering_angle_velocity = estimated_state_.steering_angle_velocity;
}

void Vehicle::readOnBoardUserInterface(void)
{
  if (digitalRead(EMERGENCY_SWITCH) == LOW)
    operational_mode_ = EMERGENCY_STOP;
}

void Vehicle::readRemoteControl(void)
{
  dBus_->FeedLine();
  dBus_->UpdateSignalState();

  if (dBus_->failsafe_status == DBUS_SIGNAL_OK)
  {
    if (dBus_->toChannels == 1)

    {
      dBus_->UpdateChannels();
      dBus_->toChannels = 0;

      if (operational_mode_ != EMERGENCY_STOP)
      {
        if (!REMOTE_CONTROL_USE_PID)
        {
          remote_control_.speed_volts = mapFloat(dBus_->channels[2], 364.0, 1684.0, -ABS_MAX_SPEED_VOLTS,
                                                 ABS_MAX_SPEED_VOLTS);
          remote_control_.steering_angle_pwm = mapFloat(dBus_->channels[0], 364.0, 1684.0, ABS_MAX_STEERING_MOTOR_PWM,
                                                        -1 * ABS_MAX_STEERING_MOTOR_PWM);
        }
        else
        {
          remote_control_.desired_state.speed = mapFloat(dBus_->channels[2], 364.0, 1684.0,
                                                         -ABS_MAX_SPEED_METERS_SECOND, ABS_MAX_SPEED_METERS_SECOND);
          remote_control_.desired_state.steering_angle = mapFloat(dBus_->channels[0], 364.0, 1684.0,
                                                                  ABS_MAX_STEERING_ANGLE_DEG,
                                                                  -ABS_MAX_STEERING_ANGLE_DEG);
        }

        //Serial.println(dBus_->channels[5]);
        if (dBus_->channels[6] != 1024)
          operational_mode_ = EMERGENCY_STOP;
        else if (dBus_->channels[5] == 1541)
          operational_mode_ = ROS_CONTROL;
        else if (dBus_->channels[5] == 511)
          operational_mode_ = REMOTE_CONTROL_NOT_SAFE;
        else
          operational_mode_ = REMOTE_CONTROL;
      }

      else
      {
        if (!REMOTE_CONTROL_USE_PID)
        {
          remote_control_.speed_volts = 0.0;
          remote_control_.steering_angle_pwm = 0.0;
        }
        else
        {
          remote_control_.desired_state.speed = 0.0;
          remote_control_.desired_state.steering_angle = 0.0;
        }

        if (dBus_->channels[6] == 1024 and dBus_->channels[4] == 364 and digitalRead(EMERGENCY_SWITCH) == HIGH)
        {
          operational_mode_ = REMOTE_CONTROL;
          digitalWrite(ENABLE_MOTORS, LOW);
        }
      }

      if (dBus_->channels[4] == 1684)
      {
        //digitalWrite(HORN, LOW);
      }
      else
      {
        digitalWrite(HORN, HIGH);
      }
    }
  }

  else
  {
    operational_mode_ = EMERGENCY_STOP;
    if (!REMOTE_CONTROL_USE_PID)
    {
      remote_control_.speed_volts = 0.0;
      remote_control_.steering_angle_pwm = 0.0;
    }
    else
    {
      remote_control_.desired_state.speed = 0.0;
      remote_control_.desired_state.steering_angle = 0.0;
    }
  }

}

void Vehicle::updateFiniteStateMachine(int millisSinceLastReactiveUpdate)
{
  if (operational_mode_ != last_operational_mode_)
  {
    resetSpeed();
    resetSteering();
    last_operational_mode_ = operational_mode_;
  }

  if (millisSinceLastReactiveUpdate > MAX_TIME_WITHOUT_REACTIVE_MILLIS && flag_speed_recommendation_active_)
  {
    operational_mode_ = EMERGENCY_STOP;
  }

  switch (operational_mode_)
  {
    case EMERGENCY_STOP:

      digitalWrite(ENABLE_MOTORS, HIGH);
      digitalWrite(BRAKE, HIGH);

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
      //if(!ros::master::check()) operational_mode_ = EMERGENCY_STOP;
      led_rgb_value_[0] = 0;
      led_rgb_value_[1] = 0;
      led_rgb_value_[2] = 255;
      break;
  }

  if (flag_limiting_speed_by_reactive_ && flag_speed_recommendation_active_)
  {
    led_rgb_value_[0] = 0;
    led_rgb_value_[1] = 255;
    led_rgb_value_[2] = 255;
  }

  analogWrite(LED_R, led_rgb_value_[0]);
  analogWrite(LED_G, led_rgb_value_[1]);
  analogWrite(LED_B, led_rgb_value_[2]);
}

void Vehicle::calculateCommandOutputs(float max_recommended_speed)
{
  switch (operational_mode_)
  {
    case REMOTE_CONTROL:
      if (!REMOTE_CONTROL_USE_PID)
      {
        speed_volts_pid_ = remote_control_.speed_volts;
        steering_angle_pwm_pid_ = remote_control_.steering_angle_pwm;
      }
      else
      {
        if (remote_control_.desired_state.speed < max_recommended_speed || !flag_speed_recommendation_active_)
        {
          desired_state_.speed = remote_control_.desired_state.speed;
          flag_limiting_speed_by_reactive_ = false;
        }
        else
        {
          desired_state_.speed = max_recommended_speed;
          flag_limiting_speed_by_reactive_ = true;
        }
        saturateSetpointIfNeeded(desired_state_.speed);

        desired_state_.steering_angle = remote_control_.desired_state.steering_angle;

        if (fabs(desired_state_.speed) <= MIN_SETPOINT_TO_USE_PID)
        {
          desired_state_.speed = 0.0;
          //estimated_state_.speed = 0.0;

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
      break;

    case REMOTE_CONTROL_NOT_SAFE:
      if (!REMOTE_CONTROL_USE_PID)
      {
        speed_volts_pid_ = remote_control_.speed_volts;
        steering_angle_pwm_pid_ = remote_control_.steering_angle_pwm;
      }
      else
      {
        desired_state_.speed = remote_control_.desired_state.speed;
        flag_limiting_speed_by_reactive_ = false;

        saturateSetpointIfNeeded(desired_state_.speed);

        desired_state_.steering_angle = remote_control_.desired_state.steering_angle;

        if (fabs(desired_state_.speed) <= MIN_SETPOINT_TO_USE_PID)
        {
          desired_state_.speed = 0.0;
          //estimated_state_.speed = 0.0;

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
      break;

    case ROS_CONTROL:
      if (desired_state_.speed < max_recommended_speed || !flag_speed_recommendation_active_)
      {
        flag_limiting_speed_by_reactive_ = false;
      }
      else
      {
        desired_state_.speed = max_recommended_speed;
        flag_limiting_speed_by_reactive_ = true;
      }

      if (fabs(desired_state_.speed) <= MIN_SETPOINT_TO_USE_PID)
      {
        desired_state_.speed = 0.0;
        estimated_state_.speed = 0.0;

        speed_volts_pid_ = 0.0;
        resetSpeed();
      }
      else
      {
        speed_controller_->computePID(ABS_MAX_SPEED_METERS_SECOND, ABS_MAX_SPEED_METERS_SECOND, ABS_MAX_SPEED_VOLTS);
      }
      steering_controller_->computePID(ABS_MAX_STEERING_ANGLE_DEG, ABS_MAX_STEERING_ANGLE_DEG,
                                       ABS_MAX_STEERING_MOTOR_PWM);
      break;

    case EMERGENCY_STOP:
      speed_volts_ = SPEED_ZERO;
      steering_angle_pwm_ = STEERING_CENTERED;
      break;
  }
}

void Vehicle::writeCommandOutputs(std_msgs::Float32MultiArray& speed_volts_and_steering_pwm_being_applicated)
{
  //bool ste_error = false;
  //bool speed_error = false;

  if (operational_mode_ == EMERGENCY_STOP)
  {
    speed_volts_ = 0;
    steering_angle_pwm_ = 0;
  }
  else
  {
    speed_volts_ = speed_volts_pid_;

    if (fabs(speed_volts_) > MIN_VOLTS_TO_RELEASE_BRAKE)
      digitalWrite(BRAKE, LOW);

    steering_angle_pwm_ = steering_angle_pwm_pid_;
  }

  if (zero_volts_millis_before_braking_ > MAX_TIME_ZERO_VOLTS_TO_BRAKE)
  {
    digitalWrite(BRAKE, HIGH);
    zero_volts_millis_before_braking_ = 0;
  }

  speed_actuator_->actuateMotor(speed_volts_);
  steering_actuator_->steeringMotor(steering_angle_pwm_);
  //ste_error = steering_actuator_->steering_motor(steering_angle_pwm_);

  //if(ste_error) error_code_ = STEERING_CONTROL_ERROR;

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

void Vehicle::setFlagSpeedRecommendationActive(bool flag_state)
{
  flag_speed_recommendation_active_ = flag_state;
}
