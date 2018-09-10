/*! \file encoder_hardware_interface.cpp
 *
 *  Created on: Jan 12, 2018
 *      Author: saul
 */

#include <Arduino.h>
#include <Wire.h>

#include "encoder_hardware_interface.h"
#include "hardware_description_constants.h"

EncoderHardwareInterface::EncoderHardwareInterface()
{
  I2C_sizeFloat_ = sizeof(float);

}

EncoderHardwareInterface::~EncoderHardwareInterface()
{
}

/*! \brief Receive the pulses of the specified encoder
 *
 *
 *  encNum = number of encoder (1 or 2)
 *  pulses = received pulses
 *  return value:
 *  0 = Success, the pulses has been retrieved correctly
 *  2 = Received NACK on transmit of address
 *  3 = Received NACK on transmit of data
 *  4 = Other error duting I2C communication
 *  5 = Wrong encoder number in parameter
 *  6 = Error requesting the data from the slave
 *  7 = Wrong received number of bytes
 */
byte EncoderHardwareInterface::encoderRead(byte code, float &data)
{
  // Pointer to the buffer for receiving
  // The pointer to a long integer allows to read a long integer from the buffer of bytes
  float* pBuffer = (float*)I2C_receiveBuffer_;

  int index = 0; // pulses of received bytes
  byte retCode;  // Return code

  // Send a command to the slave. The command is equal to the number of encoder to be read
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

/*! \brief Reset the pulses of the specified encoder to 0
 *
 * encNum = number of code (11 or 12)
 * return value:
 *   0 = Success, the pulses has been retrieved correctly
 *   2 = Received NACK on transmit of address
 *   3 = Received NACK on transmit of data
 *   4 = Other error duting I2C communication
 *   5 = Wrong encoder number in parameter
 */
byte EncoderHardwareInterface::encoderReset(int encNum)
{
  byte retCode; // Return code

  // Send a command to the slave: 11 = Reset encoder 1,  12 = Reset encoder 2
  Wire.beginTransmission(I2C_SLAVEADDRTEENSY);
  Wire.write(encNum);
  retCode = Wire.endTransmission();

  return retCode;
}
