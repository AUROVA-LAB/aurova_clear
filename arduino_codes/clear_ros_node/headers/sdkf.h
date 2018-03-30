/*
 * sdkf.h
 *
 *  Created on: 14 Mar 2018
 *      Author: idelpino
 */

#ifndef HEADERS_SDKF_H_
#define HEADERS_SDKF_H_

class sdkf;
typedef  sdkf* sdkfPtr;

class sdkf {
private:
	float vel_;
	float covariance_;

	float A_;
	float B_;

	float volts_k_minus_one_;
	float vel_k_minus_one_;
	float dt_;
	float t_k_;
	float t_k_minus_one_;

	float process_noise_;
	float sensor_noise_;
	float kalman_gain_;
	float innovation_;
	float innovation_covariance_;

	unsigned long int last_time_prediction_;
	unsigned long int last_time_correction_;
public:
	sdkf(float A, float B, float P0, float Q, float R);
	virtual ~sdkf();

	void make_prediction(float volts, float& vel, float& covariance);
	void make_correction(float measured_speed, float& vel, float& covariance);

	void resetSDKF(void);

};

#endif /* HEADERS_SDKF_H_ */
