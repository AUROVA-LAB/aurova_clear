/*! \file EKF.cpp
 *
 *  Created on: 8 Jun 2018
 *      Author: idelpino
 */

#include "EKF.h"
#include "configuration_vehicle_hardware.h"

EKF::EKF(void)
{
  //Initializations:

  //State
  X[0][0] = 0.0; //Initial steering angle (calibration error)
  X[1][0] = 0.0; //Increment of the steering angle (measured by the incremental encoder)
  X[2][0] = 0.0; //Speed of the platform

  //Covariance matrix
  P[0][0] = 3.0 * 3.0;   // We set as 3 degrees the std of out calibration method
  P[0][1] = 0.0;
  P[0][2] = 0.0;

  P[1][0] = 0.0;
  P[1][1] = 0.01 * 0.01; // we use a small value just to not put zero
  P[1][2] = 0.0;

  P[2][0] = 0.0;
  P[2][1] = 0.0;
  P[2][2] = 0.01 * 0.01; // we use a small value just to not put zero

  //Jacobian of the transition matrix
  F_x[0][0] = 1.0;
  F_x[0][1] = 0.0;
  F_x[0][2] = 0.0;

  F_x[1][0] = 0.0;
  F_x[1][1] = 1.0;
  F_x[1][2] = 0.0;

  F_x[2][0] = 0.0;
  F_x[2][1] = 0.0;
  F_x[2][2] = 1.0;

  //Jacobian of the process noise
  F_q[0][0] = 0.0;
  F_q[0][1] = 0.0;

  F_q[1][0] = 1.0;
  F_q[1][1] = 0.0;

  F_q[2][0] = 0.0;
  F_q[2][1] = 1.0;

  base_Q[0][0] = MAX_STEERING_RATE; // degrees per second
  base_Q[0][1] = 0.0;

  base_Q[1][0] = 0.0;
  base_Q[1][1] = EKF_Q_COVARIANCE; //Speed prediction covariance (in ICINCO paper is 0.005)

  Q[0][0] = 0.0;
  Q[0][1] = 0.0;

  Q[1][0] = 0.0;
  Q[1][1] = 0.0;

  y = 0.0;
  z = 0.0;

  Z = 0.0;
  R_hall = EKF_R_COVARIANCE;
  R_enc = STEERING_ENCODER_R_COVARIANCE;
  R_ls = STEERING_LIMIT_SWITCH_COVARIANCE;

  K[0][0] = 0.0;
  K[1][0] = 0.0;
  K[2][0] = 0.0;

  //H_hall must be calculated online, using the estimated velocity and the delta_t
  H_hall[0][0] = 0.0; //This value depend on the velocity and the delta_t so it is computed online
  H_hall[0][1] = 0.0; //This value is zero
  H_hall[0][2] = 0.0; //This value depensds on delta_t --> computed online

  //H_enc is independent, so it does not need to be recalculated
  H_enc[0][0] = 0.0;
  H_enc[0][1] = 1.0;
  H_enc[0][2] = 0.0;

  //H_ls is independent, so it does not need to be recalculated
  H_ls[0][0] = 1.0;
  H_ls[0][1] = 0.0;
  H_ls[0][2] = 0.0;

  G_v = SPEED_PREDICTION_GAIN;

  u_v_k_minus_one = 0.0;

  current_time_ = 0;
  last_time_prediction_ = micros();
  last_time_correction_hall_ = last_time_prediction_;
  last_time_correction_enc_ = last_time_prediction_;
}

void EKF::predict(float u_v)
{
  unsigned long int time_change;
  current_time_ = micros();

  if (current_time_ > last_time_prediction_)
    time_change = current_time_ - last_time_prediction_;
  else
    time_change = 4294967295 - last_time_prediction_;

  last_time_prediction_ = current_time_;

  float delta_t = (float)(time_change) / 1000000.0;

  //State prediction
  //X[0][0] = X[0][0]; // initial steering angle does not depend on time
  //X[1][0] = X[1][0]; // constant position model plus perturbation noise
  X[2][0] = X[2][0] + G_v * (u_v - u_v_k_minus_one);
  u_v_k_minus_one = u_v;

  // Covariance prediction
  float aux[3][3];
  float F_x_transposed[3][3];

  Q[0][0] = base_Q[0][0] * delta_t * base_Q[0][0] * delta_t; //max steering rate times delta_t gives us the max displacement in theta
  Q[1][1] = base_Q[1][1] * delta_t;

  Matrix.Multiply((float*)F_x, (float*)P, 3, 3, 3, (float*)aux);
  Matrix.Transpose((float*)F_x, 3, 3, (float*)F_x_transposed);
  Matrix.Multiply((float*)aux, (float*)F_x_transposed, 3, 3, 3, (float*)P);

  //Noise term
  float aux_noise[3][2];
  float F_q_transposed[2][3];
  float additive_prediction_noise[3][3];
  Matrix.Multiply((float*)F_q, (float*)Q, 3, 2, 2, (float*)aux_noise);
  Matrix.Transpose((float*)F_q, 3, 2, (float*)F_q_transposed);
  Matrix.Multiply((float*)aux_noise, (float*)F_q_transposed, 3, 2, 3, (float*)additive_prediction_noise);

  Matrix.Add((float*)P, (float*)additive_prediction_noise, 3, 3, (float*)P);
}

void EKF::correctHall(float observed_speed)
{
  unsigned long int time_change;
  current_time_ = micros();

  if (current_time_ > last_time_correction_hall_)
    time_change = current_time_ - last_time_correction_hall_;
  else
    time_change = 4294967295 - last_time_correction_hall_;

  last_time_correction_hall_ = current_time_;

  float delta_t = (float)(time_change) / 1000000.0;
  float steering_angle_radians = (X[0][0] + X[1][0]) * M_PI / 180.0;

  // Update the H matrix
  H_hall[0][0] = (-1 * X[2][0] * WIDTH_CENTER_WHEELS_METERS)
      / (2 * WHEELBASE_METERS * cos(steering_angle_radians) * cos(steering_angle_radians));
  H_hall[0][1] = H_hall[0][0];
  H_hall[0][2] = 1 - ((WIDTH_CENTER_WHEELS_METERS * tan(steering_angle_radians)) / (2 * WHEELBASE_METERS));

  //Innovation
  y = observed_speed;
  z = y - X[2][0] * (1 - (WIDTH_CENTER_WHEELS_METERS * tan(steering_angle_radians) / (2 * WHEELBASE_METERS)));

  //Innovation covariance
  float aux[1][3];
  float H_hall_transposed[3][1];
  Matrix.Multiply((float*)H_hall, (float*)P, 1, 3, 3, (float*)aux);
  Matrix.Transpose((float*)H_hall, 1, 3, (float*)H_hall_transposed);
  Matrix.Multiply((float*)aux, (float*)H_hall_transposed, 1, 3, 1, &Z);

  Z = Z + R_hall / delta_t;

  Matrix.Multiply((float*)P, (float*)H_hall_transposed, 3, 3, 1, (float*)K);
  K[0][0] = K[0][0] / Z;
  K[1][0] = K[1][0] / Z;
  K[2][0] = K[2][0] / Z;

  //Correction
  X[0][0] = X[0][0] + K[0][0] * z;
  X[1][0] = X[1][0] + K[1][0] * z;
  X[2][0] = X[2][0] + K[2][0] * z;

  float aux_K[3][1];
  float K_transposed[1][3];
  float substractive_term[3][3];
  Matrix.Multiply((float*)K, &Z, 3, 1, 1, (float*)aux_K);
  Matrix.Transpose((float*)K, 3, 1, (float*)K_transposed);
  Matrix.Multiply((float*)aux_K, (float*)K_transposed, 3, 1, 3, (float*)substractive_term);

  Matrix.Subtract((float*)P, (float*)substractive_term, 3, 3, (float*)P);

}

void EKF::correctEnc(float observed_steering_increment)
{
  unsigned long int time_change;
  current_time_ = micros();

  if (current_time_ > last_time_correction_enc_)
    time_change = current_time_ - last_time_correction_enc_;
  else
    time_change = 4294967295 - last_time_correction_enc_;

  last_time_correction_enc_ = current_time_;

  float delta_t = (float)(time_change) / 1000000.0;

  //Innovation
  y = observed_steering_increment;
  z = y - X[1][0];

  //Innovation covariance
  float aux[1][3];
  float H_enc_transposed[3][1];
  Matrix.Multiply((float*)H_enc, (float*)P, 1, 3, 3, (float*)aux);
  Matrix.Transpose((float*)H_enc, 1, 3, (float*)H_enc_transposed);
  Matrix.Multiply((float*)aux, (float*)H_enc_transposed, 1, 3, 1, &Z);

  Z = Z + R_enc / delta_t;

  Matrix.Multiply((float*)P, (float*)H_enc_transposed, 3, 3, 1, (float*)K);
  K[0][0] = K[0][0] / Z;
  K[1][0] = K[1][0] / Z;
  K[2][0] = K[2][0] / Z;

  //Correction
  X[0][0] = X[0][0] + K[0][0] * z;
  X[1][0] = X[1][0] + K[1][0] * z;
  X[2][0] = X[2][0] + K[2][0] * z;

  float aux_K[3][1];
  float K_transposed[1][3];
  float substractive_term[3][3];
  Matrix.Multiply((float*)K, &Z, 3, 1, 1, (float*)aux_K);
  Matrix.Transpose((float*)K, 3, 1, (float*)K_transposed);
  Matrix.Multiply((float*)aux_K, (float*)K_transposed, 3, 1, 3, (float*)substractive_term);

  Matrix.Subtract((float*)P, (float*)substractive_term, 3, 3, (float*)P);
}

void EKF::correctLs(float observed_theta)
{
  Serial.println("Limit switch observed!");
  //Innovation
  y = observed_theta;
  z = y - (X[0][0] + X[1][0]); // The prediction will be the initial steering plus the increment

  //Innovation covariance
  float aux[1][3];
  float H_ls_transposed[3][1];
  Matrix.Multiply((float*)H_ls, (float*)P, 1, 3, 3, (float*)aux);
  Matrix.Transpose((float*)H_ls, 1, 3, (float*)H_ls_transposed);
  Matrix.Multiply((float*)aux, (float*)H_ls_transposed, 1, 3, 1, &Z);

  Z = Z + R_ls;

  Matrix.Multiply((float*)P, (float*)H_ls_transposed, 3, 3, 1, (float*)K);
  K[0][0] = K[0][0] / Z;
  K[1][0] = K[1][0] / Z;
  K[2][0] = K[2][0] / Z;

  //Correction
  X[0][0] = X[0][0] + K[0][0] * z;
  X[1][0] = X[1][0] + K[1][0] * z;
  X[2][0] = X[2][0] + K[2][0] * z;

  float aux_K[3][1];
  float K_transposed[1][3];
  float substractive_term[3][3];
  Matrix.Multiply((float*)K, &Z, 3, 1, 1, (float*)aux_K);
  Matrix.Transpose((float*)K, 3, 1, (float*)K_transposed);
  Matrix.Multiply((float*)aux_K, (float*)K_transposed, 3, 1, 3, (float*)substractive_term);

  Matrix.Subtract((float*)P, (float*)substractive_term, 3, 3, (float*)P);

}

void EKF::getState(float& initial_steering_angle_deg, float& steering_angle_increment_deg, float& speed_ms)
{
  initial_steering_angle_deg = X[0][0];
  steering_angle_increment_deg = X[1][0];
  speed_ms = X[2][0];
}

void EKF::getVariances(float& initial_steering_angle_variance, float& steering_angle_increment_variance,
                       float& speed_variance)
{
  initial_steering_angle_variance = P[0][0];
  steering_angle_increment_variance = P[1][1];
  speed_variance = P[2][2];
}

