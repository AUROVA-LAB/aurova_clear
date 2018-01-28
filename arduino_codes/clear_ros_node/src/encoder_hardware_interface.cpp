/*
 * encoder_hardware_interface.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: saul
 */

#include <Arduino.h>
#include <Wire.h>

#include "../headers/encoder_hardware_interface.h"
#include "../headers/hardware_description_constants.h"

EncoderHardwareInterface::EncoderHardwareInterface()
{
	I2C_sizeFloat_ = sizeof(float);

}

EncoderHardwareInterface::~EncoderHardwareInterface()
{}


// Receive the pulses of the specified encoder
// encNum = number of encoder (1 or 2)
// pulses = received pulses
// return value:
//   0 = Success, the pulses has been retrieved correctly
//   2 = Received NACK on transmit of address
//   3 = Received NACK on transmit of data
//   4 = Other error duting I2C communication
//   5 = Wrong encoder number in parameter
//   6 = Error requesting the data from the slave
//   7 = Wrong received number of bytes
byte EncoderHardwareInterface::encoderRead(byte code, float &data)
{
  // Pointer to the buffer for receiving
  // The pointer to a long integer allows to read a long integer from the buffer of bytes
  float* pBuffer = (float*)I2C_receiveBuffer_;

  int index = 0; // pulses of received bytes
  byte retCode;  // Return code

  // Send a command to the slave. The command is equal to the number of encoder to be readed
  // 1 = Read encoder 1,  2 = Read encoder 2
  Wire.beginTransmission(I2C_SLAVEADDRTEENSY);
  Wire.write(code);
  retCode = Wire.endTransmission();
  if (retCode > 0)
	return retCode;

  // Request the bytes of the count of the encoder
  // Five long integers are expected
  if (Wire.requestFrom(I2C_SLAVEADDRTEENSY, I2C_sizeFloat_) <= 0)
	return 6;

  // Receive the bytes from the slave
  while (Wire.available() && (index < I2C_sizeFloat_))
	I2C_receiveBuffer_[index++] = Wire.read();

  // The number of bytes received is not correct
  if (index != I2C_sizeFloat_)
	return 7;

  data = *pBuffer;

	  return 0; // Success
}

// Reset the pulses of the specified encoder to 0
// encNum = number of code (11 or 12)
// return value:
//   0 = Success, the pulses has been retrieved correctly
//   2 = Received NACK on transmit of address
//   3 = Received NACK on transmit of data
//   4 = Other error duting I2C communication
//   5 = Wrong encoder number in parameter
byte EncoderHardwareInterface::encoderReset(int encNum)
{
  byte retCode; // Return code

  /*// Validate the number of encoder
  if ((encNum < 1) || (encNum > 2))
    return 5;

  // Send a command to the slave: 3 = Reset encoder 1,  4 = Reset encoder 2
  Wire.beginTransmission(I2C_SLAVEADDRTEENSY);
  Wire.write((encNum==1)? 3:4);
  retCode = Wire.endTransmission();*/

  // Send a command to the slave: 11 = Reset encoder 1,  12 = Reset encoder 2
  Wire.beginTransmission(I2C_SLAVEADDRTEENSY);
  Wire.write(encNum);
  retCode = Wire.endTransmission();

  return retCode;
}


// Set the pulses of the specified encoder to the given value
// encNum = number of encoder (1 or 2)
// pulses = new pulses for the encoder
// return value:
//   0 = Success, the pulses has been retrieved correctly
//   2 = Received NACK on transmit of address
//   3 = Received NACK on transmit of data
//   4 = Other error duting I2C communication
//   5 = Wrong encoder number in parameter
byte EncoderHardwareInterface::encoderWrite(byte encNum, long pulses)
{
  // Pointer to the buffer for sending
  // The pointer to a long integer allows to store a long integer into the buffer of bytes
  long* pBuffer = (long*)I2C_sendBuffer_;

  byte retCode; // Return code

  // Validate the number of encoder
  if ((encNum < 1) || (encNum > 2))
    return 5;

  // Store the given pulses in the send buffer as a long integer
  *pBuffer = pulses;

  // Send a command to the slave: 5 = Write encoder 1,  6 = Write encoder 2
  // After the command, the bytes of the pulses are sent
  Wire.beginTransmission(I2C_SLAVEADDRTEENSY);
  Wire.write((encNum==1)? 5:6);
  Wire.write(I2C_sendBuffer_, I2C_sizeFloat_);
  retCode = Wire.endTransmission();

  return retCode;
}


