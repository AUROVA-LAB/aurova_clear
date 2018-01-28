#ifndef _PID_H_
#define _PID_H_

class PID
{
private:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable

  float dt_, max_, min_;
  float kp_, ki_, kd_;
  float pre_error_;
  float integral_;

  float error_, derivative_;
  float p_out_, i_out_, d_out_;

  int error_code_;

public:

  PID();

  /*!
   * Returns the manipulated variable given a setpoint, the current process value
   * and the time elapsed since previous control
   */
  int calculate(float setpoint, float pv, float dt, float& output);

  /*!
   * Set the maximum and minimum values for the output
   * @param max
   * @param min
   */
  void setPIDMaxMinValues(float max, float min);

  /*!
   * Set the PID gains
   * @param kp
   * @param ki
   * @param kd
   */
  void setPIDGains(float kp, float ki, float kd);

  void getPIDGains(float& kp, float& ki, float& kd);

  ~PID();
};

#endif
