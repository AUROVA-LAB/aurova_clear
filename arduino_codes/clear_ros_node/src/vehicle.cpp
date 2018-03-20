/*
 * vehicle.cpp
 *
 *  Created on: 3 Nov 2017
 *      Author: idelpino
 */

#include <elapsedMillis.h>

//#include "/opt/ros/kinetic/include/ros/master.h"
#include "../headers/vehicle.h"
#include "../headers/arduino_ros_interface.h"
#include "../headers/hardware_description_constants.h"
#include "../headers/configuration_vehicle_hardware.h"
#include "../headers/PID_v1.h"


elapsedMillis timeLastComputeSteering = SAMPLING_TIME_TEENSY/2;
elapsedMillis timeLastComputeSpeed = 0;

elapsedMillis timeBeforeBrake = 0;

float* speed_measures;
float* steering_measures;


Vehicle::Vehicle()
{
  operational_mode_ = CALIBRATION;

  estimated_state_.steering_angle = 0.0;           // deg
  estimated_state_.steering_angle_velocity = 0.0;  // deg/s

  estimated_state_.speed = 0.0;                    // m/s
  estimated_state_.acceleration = 0.0;             // m/s^2
  estimated_state_.jerk = 0.0;                     // m/s^3

  measured_state_.steering_angle = 0.0;           // deg
  measured_state_.steering_angle_velocity = 0.0;  // deg/s

  measured_state_.speed = 0.0;                    // m/s
  measured_state_.acceleration = 0.0;             // m/s^2
  measured_state_.jerk = 0.0;                     // m/s^3

  desired_state_.steering_angle = 0.0;            // deg
  desired_state_.steering_angle_velocity = 0.0;   // deg/s

  desired_state_.speed = 0.0;                     // m/s
  desired_state_.acceleration = 0.0;              // m/s^2
  desired_state_.jerk = 0.0;                      // m/s^3

  desired_steering_state_reached_     = false;
  desired_traslational_state_reached_ = false;

  remote_control_.speed_volts = 0.0;
  remote_control_.steering_angle_pwm = 0.0;

  remote_control_.desired_state.steering_angle = 0.0;            // deg
  remote_control_.desired_state.steering_angle_velocity = 0.0;   // deg/s

  remote_control_.desired_state.speed = 0.0;                     // m/s
  remote_control_.desired_state.acceleration = 0.0;              // m/s^2
  remote_control_.desired_state.jerk = 0.0;                      // m/s^3

  error_code_ = NO_ERROR;

  speed_volts_ = 0.0;
  steering_angle_pwm_ = 0.0;

  speed_volts_pid_ = 0.0;
  steering_angle_pwm_pid_ = 0.0;


  dBus_ = new DJI_DBUS(RC_PORT);
  dBus_->begin();

  speed_actuator_ = new SpeedHardwareInterface(PIN_CH1, PIN_CH2);
  steering_actuator_ = new SteeringHardwareInterface();

  speed_actuator_->actuateMotor(0.0);  // To ensure that in the start the output voltage is equal to zero
  steering_actuator_->steeringMotor(0);

  speed_controller_ = new PID(&estimated_state_.speed, &speed_volts_pid_, &desired_state_.speed, SPEED_KP, SPEED_KI, SPEED_KD, 0);
  //speed_controller_->SetOutputLimits(-1 * ABS_MAX_SPEED_VOLTS, ABS_MAX_SPEED_VOLTS);
  speed_controller_->SetOutputLimits(-1, 1);

  speed_stimator_ = new sdkf(1.0, 0.3, 0.25, 0.0064, 0.05);


  steering_controller_ = new PID(&estimated_state_.steering_angle, &steering_angle_pwm_pid_, &desired_state_.steering_angle, STEERING_KP, STEERING_KI, STEERING_KD, 0);
  steering_controller_->SetOutputLimits(-ABS_MAX_STEERING_MOTOR_PWM, ABS_MAX_STEERING_MOTOR_PWM);

  speed_controller_->SetMode(AUTOMATIC); // Activate PID controllers
  steering_controller_->SetMode(AUTOMATIC);

  led_rgb_value_[0] = 0;
  led_rgb_value_[1] = 0;
  led_rgb_value_[2] = 0;

  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);

  pinMode(HORN,OUTPUT);
  digitalWrite(HORN,HIGH);

  pinMode(ENABLE_MOTORS,OUTPUT);
  digitalWrite(ENABLE_MOTORS,LOW);

  pinMode(BRAKE,OUTPUT);
  digitalWrite(BRAKE,HIGH);

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

  speed_controller_->SetTunings(kp,ki,kd);
}

void Vehicle::getVelocityPIDGains(std_msgs::Float32MultiArray& current_pid_gains)
{
  current_pid_gains.data[0] = speed_controller_->GetKp();
  current_pid_gains.data[1] = speed_controller_->GetKi();
  current_pid_gains.data[2] = speed_controller_->GetKd();
}

void Vehicle::setSteeringPIDGains(const std_msgs::Float32MultiArray& desired_pid_gains)
{
  float kp = desired_pid_gains.data[0];
  float ki = desired_pid_gains.data[1];
  float kd = desired_pid_gains.data[2];

  steering_controller_->SetTunings(kp, ki, kd);

}

void Vehicle::getSteeringPIDGains(std_msgs::Float32MultiArray& current_pid_gains)
{
  current_pid_gains.data[0] = steering_controller_->GetKp();
  current_pid_gains.data[1] = steering_controller_->GetKi();
  current_pid_gains.data[2] = steering_controller_->GetKd();
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

      digitalWrite(ENABLE_MOTORS,LOW);

      led_rgb_value_[0] = 0;
	  led_rgb_value_[1] = 0;
	  led_rgb_value_[2] = 0;
      if ( desired_steering_state_reached_ )
      {
        operational_mode_ = REMOTE_CONTROL;
      }
      break;

    case EMERGENCY_STOP:

      digitalWrite(ENABLE_MOTORS,HIGH);
      digitalWrite(BRAKE,HIGH);

      led_rgb_value_[0] = 255;
  	  led_rgb_value_[1] = 0;
  	  led_rgb_value_[2] = 0;
      break;

    case REMOTE_CONTROL:
  	  led_rgb_value_[0] = 0;
  	  led_rgb_value_[1] = 255;
  	  led_rgb_value_[2] = 0;
      break;

    case ROS_CONTROL:
    	//if(!ros::master::check()) operational_mode_ = EMERGENCY_STOP;
  	  led_rgb_value_[0] = 0;
  	  led_rgb_value_[1] = 0;
  	  led_rgb_value_[2] = 255;
      break;

    case CALIBRATION:
  	  led_rgb_value_[0] = 255;
  	  led_rgb_value_[1] = 255;
  	  led_rgb_value_[2] = 0;
	  if( componentsCalibration() )
	  {
	      operational_mode_ = REMOTE_CONTROL;
	  }
	  break;

  }

  analogWrite(LED_R, led_rgb_value_[0]);
  analogWrite(LED_G, led_rgb_value_[1]);
  analogWrite(LED_B, led_rgb_value_[2]);
}

int Vehicle::getOperationalMode(void)
{
  return (operational_mode_);
}

void Vehicle::updateState(ackermann_msgs::AckermannDriveStamped& estimated_ackermann_state)
{
  // We always make the prediction step
  float covariance;
  speed_stimator_->make_prediction(speed_volts_, estimated_state_.speed, covariance);

  if(timeLastComputeSteering > SAMPLING_TIME_TEENSY)
  {
	steering_measures = steering_actuator_->getSteeringMeasures();
	timeLastComputeSteering = 0;

	measured_state_.steering_angle = steering_measures[0]*PULSES_TO_DEG;
	measured_state_.steering_angle_velocity = steering_measures[1]*PULSES_TO_DEG;
  }
  if(timeLastComputeSpeed > SAMPLING_TIME_TEENSY * 10)
  {
    speed_measures = speed_actuator_->getSpeedMeasures();
	timeLastComputeSpeed = 0;

	float direction = 1.0;
	if(!speed_actuator_->getFlagForward()) direction = -1.0;

	measured_state_.speed = speed_measures[0]*METERS_PER_PULSE*direction;
	measured_state_.acceleration = speed_measures[1]*METERS_PER_PULSE; // TODO--> check if direction is also needed here
	measured_state_.jerk = speed_measures[2]*METERS_PER_PULSE;

	speed_stimator_->make_correction(measured_state_.speed, estimated_state_.speed, covariance);
  }

  //without Kalman for speed estimation

  //estimated_state_.speed = measured_state_.speed / ABS_MAX_SPEED_METERS_SECOND;
  estimated_state_.speed = estimated_state_.speed / ABS_MAX_SPEED_METERS_SECOND;
  estimated_state_.acceleration = measured_state_.acceleration;
  estimated_state_.jerk = measured_state_.jerk;


  estimated_state_.steering_angle = measured_state_.steering_angle;
  estimated_state_.steering_angle_velocity = measured_state_.steering_angle_velocity;


  estimated_ackermann_state.drive.speed = estimated_state_.speed;
  estimated_ackermann_state.drive.acceleration = estimated_state_.acceleration;
  estimated_ackermann_state.drive.jerk = estimated_state_.jerk;

  estimated_ackermann_state.drive.steering_angle = estimated_state_.steering_angle;
  estimated_ackermann_state.drive.steering_angle_velocity = estimated_state_.steering_angle_velocity;


}

void Vehicle::updateROSDesiredState(const ackermann_msgs::AckermannDriveStamped& desired_ackermann_state)
{
  desired_state_.steering_angle = desired_ackermann_state.drive.steering_angle;
  desired_state_.steering_angle_velocity = desired_ackermann_state.drive.steering_angle_velocity;

  desired_state_.speed = desired_ackermann_state.drive.speed / ABS_MAX_SPEED_METERS_SECOND;
  desired_state_.acceleration = desired_ackermann_state.drive.acceleration;
  desired_state_.jerk = desired_ackermann_state.drive.jerk;
}

void Vehicle::getDesiredState(ackermann_msgs::AckermannDriveStamped& desired_ackermann_state_echo)
{
	desired_ackermann_state_echo.drive.steering_angle = desired_state_.steering_angle;
	desired_ackermann_state_echo.drive.steering_angle_velocity = desired_state_.steering_angle_velocity;

    desired_ackermann_state_echo.drive.speed = desired_state_.speed;
    desired_ackermann_state_echo.drive.acceleration = desired_state_.acceleration;
    desired_ackermann_state_echo.drive.jerk = desired_state_.jerk;
}

void Vehicle::calculateCommandOutputs(void)
{
  switch (operational_mode_)
  {
    case REMOTE_CONTROL:
	  if(!REMOTE_CONTROL_USE_PID)
	  {
		  speed_volts_pid_ = remote_control_.speed_volts;
		  steering_angle_pwm_pid_ = remote_control_.steering_angle_pwm;
	  }else{
		  desired_state_.speed = remote_control_.desired_state.speed / ABS_MAX_SPEED_METERS_SECOND;
		  desired_state_.steering_angle = remote_control_.desired_state.steering_angle;
		  speed_controller_->Compute();
		  steering_controller_->Compute();
	  }
      break;

    case ROS_CONTROL:

      speed_controller_->Compute();
      steering_controller_->Compute();
      break;

    case EMERGENCY_STOP:

      speed_volts_ = SPEED_ZERO;
      steering_angle_pwm_ = STEERING_CENTERED;
      break;

    case RESET:
      desired_state_.speed = SPEED_ZERO;
      desired_state_.steering_angle = STEERING_CENTERED;
	  //speed_controller_->Compute();
	  //steering_controller_->Compute();
      break;
  }
}

void Vehicle::readRemoteControl(void)
{
  dBus_->FeedLine();
  dBus_->UpdateSignalState();

  if(dBus_->failsafe_status == DBUS_SIGNAL_OK)
  {
	  if (dBus_->toChannels == 1)

	  {
		dBus_->UpdateChannels();
		dBus_->toChannels = 0;

		  if(operational_mode_ != EMERGENCY_STOP)
		  {
			  if(!REMOTE_CONTROL_USE_PID)
			  {
				  //remote_control_.speed_volts = mapFloat(dBus_->channels[2], 364.0, 1684.0, -ABS_MAX_SPEED_VOLTS, ABS_MAX_SPEED_VOLTS);
				  remote_control_.speed_volts = mapFloat(dBus_->channels[2], 364.0, 1684.0, -1.0, 1.0);
				  remote_control_.steering_angle_pwm = mapFloat(dBus_->channels[0], 364.0, 1684.0, ABS_MAX_STEERING_MOTOR_PWM, -1 * ABS_MAX_STEERING_MOTOR_PWM);
			  }
			  else
			  {
				  remote_control_.desired_state.speed = mapFloat(dBus_->channels[2], 364.0, 1684.0, -ABS_MAX_SPEED_METERS_SECOND, ABS_MAX_SPEED_METERS_SECOND);
				  remote_control_.desired_state.steering_angle = mapFloat(dBus_->channels[0], 364.0, 1684.0, ABS_MAX_STEERING_ANGLE_DEG, -ABS_MAX_STEERING_ANGLE_DEG);
			  }


			  if(dBus_->channels[6] != 1024)
				  operational_mode_ = EMERGENCY_STOP;
			  else if(dBus_->channels[5] == 1541)
				  operational_mode_ = ROS_CONTROL;
			  else
				  operational_mode_ = REMOTE_CONTROL;
		  }

		  else
		  {
			  if(!REMOTE_CONTROL_USE_PID)
			  {
				  remote_control_.speed_volts = 0.0;
				  remote_control_.steering_angle_pwm = 0.0;
			  }else{
				  remote_control_.desired_state.speed = 0.0;
				  remote_control_.desired_state.steering_angle = 0.0;
			  }

			  if(dBus_->channels[6] == 1024 and dBus_->channels[4] == 364 and digitalRead(EMERGENCY_SWITCH) == HIGH)
			  {
				  operational_mode_ = RESET;
			  }
		  }

		  if(dBus_->channels[4] == 1684)
			  digitalWrite(HORN,LOW);
		  else
			  digitalWrite(HORN,HIGH);

		  Serial.println(dBus_->channels[4]);

		}
  }

  else
  {
	operational_mode_ = EMERGENCY_STOP;
	if(!REMOTE_CONTROL_USE_PID)
	{
	  remote_control_.speed_volts = 0.0;
	  remote_control_.steering_angle_pwm = 0.0;
	}else{
	  remote_control_.desired_state.speed = 0.0;
	  remote_control_.desired_state.steering_angle = 0.0;
	}
  }

}

void Vehicle::readOnBoardUserInterface(void)
{
	if(digitalRead(EMERGENCY_SWITCH) == LOW)
		operational_mode_ = EMERGENCY_STOP;
}

void Vehicle::writeCommandOutputs(std_msgs::Float32MultiArray& speed_volts_and_steering_pwm_being_applicated)
{
  //bool ste_error = false;
  //bool speed_error = false;

  if(operational_mode_ == EMERGENCY_STOP)
  {
	  speed_volts_ = 0;
	  steering_angle_pwm_ = 0;
  }

  else if(operational_mode_)
  {
	  speed_volts_ = speed_volts_pid_ * ABS_MAX_SPEED_VOLTS;
	  if(fabs(speed_volts_) > MIN_VOLTS_TO_RELEASE_BRAKE) digitalWrite(BRAKE,LOW);
	  steering_angle_pwm_ = steering_angle_pwm_pid_;
  }

  if(timeBeforeBrake > MAX_TIME_ZERO_VOLTS_TO_BRAKE)
  {
	  digitalWrite(BRAKE, HIGH);
	  timeBeforeBrake = 0;
  }

  speed_actuator_->actuateMotor(speed_volts_);
  steering_actuator_->steeringMotor(steering_angle_pwm_);
  //ste_error = steering_actuator_->steering_motor(steering_angle_pwm_);

  //if(ste_error) error_code_ = STEERING_CONTROL_ERROR;

  //Passing the applied values to communicate them to ROS
  speed_volts_and_steering_pwm_being_applicated.data[0] = speed_volts_;
  speed_volts_and_steering_pwm_being_applicated.data[1] = steering_angle_pwm_;

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
