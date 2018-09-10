/*! \file pid.h
 *
 *  Created on: Mar 28, 2018
 *      Author: idelpino
 */

#ifndef HEADERS_PID_H_
#define HEADERS_PID_H_

class PID;
typedef PID* PIDPtr;

class PID
{
private:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable

  const float STD_LOOP_TIME = 0.001;

  float kp_, ki_, kd_;

  float proportional_;
  float integral_;
  float derivative_;

  float error_;
  float previous_error_;

  double dt_;
  unsigned long int current_time_, last_time_;

  float max_, min_;
  float tolerance_threshold_to_output_zero_; //if the absolute value of the output is below
                                             //this threshold, the output is set to zero

  float* input_;
  float* output_;
  float* set_point_;

  bool first_time_;

public:

  PID(float* input, float* output, float* setpoint, float kp, float ki, float kd, float tolerance_to_output_zero);
  ~PID();

  /*!
   * Returns the manipulated variable given a setpoint, the current process value
   * and the time elapsed since previous control
   */
  void computePID(float scale_input, float scale_setpoint, float scale_output);

  /*!
   * Set the maximum and minimum values for the output
   * @param max
   * @param min
   */
  void setOutputLimits(float min, float max);

  /*!
   * Set the PID gains
   * @param kp
   * @param ki
   * @param kd
   */
  void setPIDGains(float kp, float ki, float kd);

  void getPIDGains(float& kp, float& ki, float& kd);

  void resetPID(void);

};

#endif /* HEADERS_PID_H_ */
