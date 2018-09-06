/*
 * EKF.h
 *
 *  Created on: 8 Jun 2018
 *      Author: idelpino
 */

#ifndef HEADERS_EKF_H_
#define HEADERS_EKF_H_

#include "math.h"
#include "MatrixMath.h"

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

  void predict(float u_v);

  void correctHall(float observed_speed);

  void correctEnc(float observed_steering_increment);

  void correctLs(float observed_theta);

  void getState(float& initial_steering_angle_deg, float& steering_angle_increment_deg, float& speed_ms);

  void getVariances(float& initial_steering_angle_variance, float& steering_angle_increment_variance,
                    float& speed_variance);

};

#endif /* HEADERS_EKF_H_ */
