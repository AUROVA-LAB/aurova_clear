/*
 * encoder_hardware_interface.h
 *
 *  Created on: Jan 12, 2018
 *      Author: saul
 */

#ifndef HEADERS_ENCODER_HARDWARE_INTERFACE_H_
#define HEADERS_ENCODER_HARDWARE_INTERFACE_H_

#include <elapsedMillis.h> // For scheduling tasks


class EncoderHardwareInterface;
typedef  EncoderHardwareInterface* EncoderHardwareInterfacePtr;


class EncoderHardwareInterface
{
private:

	// Buffers for I2C communication
	// Buffers are used to send and receive long integers
	int I2C_sizeFloat_;
	byte I2C_sendBuffer_[sizeof(float)]; 			    // Sending buffer to store one long integer
	byte I2C_receiveBuffer_[1+sizeof(float)]; 	// Reception buffer to store five long integers


public:
    EncoderHardwareInterface();
    ~EncoderHardwareInterface();

    byte encoderRead(byte code, float &data);

    byte encoderReset(int encNum);

    byte encoderWrite(int encNum, long count);

};



#endif /* HEADERS_ENCODER_HARDWARE_INTERFACE_H_ */
