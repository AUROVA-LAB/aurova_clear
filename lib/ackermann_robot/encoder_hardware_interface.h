/*! \file encoder_hardware_interface.h
 *
 *  Created on: Jan 12, 2018
 *      Author: saul
 */

#ifndef HEADERS_ENCODER_HARDWARE_INTERFACE_H_
#define HEADERS_ENCODER_HARDWARE_INTERFACE_H_

#include <elapsedMillis.h>

class EncoderHardwareInterface;
typedef EncoderHardwareInterface* EncoderHardwareInterfacePtr;

/*!
 * \class EncoderHardwareInterface
 * \bried Class to manage the communications with an external
 * microcontroller thats is devoted to count the encoder pulses
 */
class EncoderHardwareInterface
{
private:

  // Buffers for I2C communication
  // Buffers are used to send and receive long integers
  int I2C_sizeFloat_;
  byte I2C_sendBuffer_[sizeof(float)]; 	        // Sending buffer to store one long integer
  byte I2C_receiveBuffer_[1 + sizeof(float)]; 	// Reception buffer to store five long integers

public:
  EncoderHardwareInterface();
  ~EncoderHardwareInterface();

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
  byte encoderRead(byte code, float &data);


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
  byte encoderReset(int encNum);
};

#endif /* HEADERS_ENCODER_HARDWARE_INTERFACE_H_ */
