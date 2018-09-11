/*! \file EKF.h
 *
 *  \brief An Ad-hoc Extended Kalman Filter implementation for speed and steering angle
 *  estimation by means of incremental encoders and limit switches
 *
 *  Created on: 8 Jun 2018
 *      Author: idelpino
 */

#ifndef HEADERS_EKF_H_
#define HEADERS_EKF_H_

#include "math.h"
#include "MatrixMath.h"

/*!
 *  \class EKF
 *  \brief This class estimates the speed (m/s), initial steering angle (deg)
 *  and steering angle increment.
 *
 *  It has three kind of sensors, a low resolution incremental speed encoder,
 *  a high resolution incremental steering encoder and two limit switches that
 *  provide absolute steering angle information
 */
class EKF;
typedef EKF* EKFPtr;

class EKF
{
private:

  float F_x[3][3];
  float F_q[3][2];
  float base_Q[2][2];
  float Q[2][2];
  float y;
  float z;
  float H_hall[1][3];
  float H_enc[1][3];
  float H_ls[1][3];
  float Z;
  float R_hall;
  float R_enc;
  float R_ls;
  float K[3][1];

  float G_v;

  float u_v_k_minus_one;

  unsigned long int current_time_;
  unsigned long int last_time_prediction_;
  unsigned long int last_time_correction_hall_;
  unsigned long int last_time_correction_enc_;

public:
  float X[3][1];
  float P[3][3];

  EKF(void);

  /*!
   * \brief EKF prediction, it requires the current output of the speed controller
   * to enhance the speed prediction, due to the low resolution of the encoder and the
   * linearity of the electric motor
   *
   * \param u_v Voltage applied to the speed motor
   */
  void predict(float u_v);

  /*!
   * \brief EKF correction using a low resolution incremental encoder for measuring the speed
   *
   * \param observed_speed the measured speed in meters per second
   */
  void correctHall(float observed_speed);

  /*!
   * \brief EKF correction using a high resolution incremental encoder for measuring
   * the steering angle increment
   *
   * \param observed_steering_increment the measured steering angle increment in degrees
   */
  void correctEnc(float observed_steering_increment);

  /*!
   * \brief EKF correction using a limit switch that gives absolute steering position information
   *
   * \param observed_theta the measured steering angle in degrees
   */
  void correctLs(float observed_theta);

  /*!
   * \brief Returns the estimated mean of the three state variables
   *
   * \param initial_steering_angle_deg
   * \param steering_angle_increment_deg
   * \param speed_ms
   */
  void getState(float& initial_steering_angle_deg, float& steering_angle_increment_deg, float& speed_ms);

  /*!
   * \brief Returns the estimated variances of the three state variables
   *
   * \param initial_steering_angle_variance
   * \param steering_angle_increment_variance
   * \param speed_variance
   */
  void getVariances(float& initial_steering_angle_variance, float& steering_angle_increment_variance,
                    float& speed_variance);

};

#endif /* HEADERS_EKF_H_ */
