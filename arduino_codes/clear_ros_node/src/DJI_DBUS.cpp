/*! \file DJI_DBUS.cpp
 DJI DBUS data decoder library
 (c) S. Driussi 20141215
 Not for commercial use

 Work started from mikeshub Futaba library
 https://github.com/mikeshub/FUTABA_SBUS

 Refer to naza_dbus_decoder_wiring.jpg diagram for proper connection

 Modified by F.Cadelas to simplify funcionality to channel decoding
 Modified by S. Cova to
 */
#include "DJI_DBUS.h"

void DJI_DBUS::begin()
{
  uint8_t loc_sbusData[25] = {0x0f, 0x01, 0x04, 0x20, 0x00, 0xff, 0x07, 0x40, 0x00, 0x02, 0x10, 0x80, 0x2c, 0x64, 0x21,
                              0x0b, 0x59, 0x08, 0x40, 0x00, 0x02, 0x10, 0x80, 0x00, 0x00};
  int16_t loc_channels[18] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
                              1023, 0, 0};
  _serial.begin(BAUDRATE);

  memcpy(sbusData, loc_sbusData, 25);
  memcpy(channels, loc_channels, 18);
  failsafe_status = DBUS_SIGNAL_OK;
  toChannels = 0;
  bufferIndex = 0;
  feedState = 0;
}

int16_t DJI_DBUS::Channel(uint8_t ch)
{
  // Read channel data
  if ((ch > 0) && (ch <= 16))
  {
    return channels[ch - 1];
  }
  else
  {
    return 1023;
  }
}
uint8_t DJI_DBUS::DigiChannel(uint8_t ch)
{
  // Read digital channel data
  if ((ch > 0) && (ch <= 2))
  {
    return channels[15 + ch];
  }
  else
  {
    return 0;
  }
}

void DJI_DBUS::UpdateChannels(void)
{
  uint8_t i;

  channels[0] = ((sbusData[1] | sbusData[2] << 8) & 0x07FF);
  channels[1] = ((sbusData[2] >> 3 | sbusData[3] << 5) & 0x07FF);
  channels[2] = ((sbusData[3] >> 6 | sbusData[4] << 2 | sbusData[5] << 10) & 0x07FF);
  channels[3] = ((sbusData[5] >> 1 | sbusData[6] << 7) & 0x07FF);
  channels[4] = ((sbusData[6] >> 4 | sbusData[7] << 4) & 0x07FF);
  channels[5] = ((sbusData[7] >> 7 | sbusData[8] << 1 | sbusData[9] << 9) & 0x07FF);
  channels[6] = ((sbusData[9] >> 2 | sbusData[10] << 6) & 0x07FF);
  channels[7] = ((sbusData[10] >> 5 | sbusData[11] << 3) & 0x07FF);

  // & the other 8 + 2 channels if you need them
#ifdef ALL_CHANNELS
  channels[8] = ((sbusData[12]|sbusData[13]<< 8) & 0x07FF);
  channels[9] = ((sbusData[13]>>3|sbusData[14]<<5) & 0x07FF);
  channels[10] = ((sbusData[14]>>6|sbusData[15]<<2|sbusData[16]<<10) & 0x07FF);
  channels[11] = ((sbusData[16]>>1|sbusData[17]<<7) & 0x07FF);
  channels[12] = ((sbusData[17]>>4|sbusData[18]<<4) & 0x07FF);
  channels[13] = ((sbusData[18]>>7|sbusData[19]<<1|sbusData[20]<<9) & 0x07FF);
  channels[14] = ((sbusData[20]>>2|sbusData[21]<<6) & 0x07FF);
  channels[15] = ((sbusData[21]>>5|sbusData[22]<<3) & 0x07FF);
  // DigiChannel 1
  if (sbusData[23] & (1<<0))
  {
    channels[16] = 1;
  }
  else
  {
    channels[16] = 0;
  }
  // DigiChannel 2
  if (sbusData[23] & (1<<1))
  {
    channels[17] = 1;
  }
  else
  {
    channels[17] = 0;
  }
#endif

  /*
   for (i=0; i<8; i++) {
   channels[i]  = map(channels[i],364,1684,1000,2000);
   }
   */

  // Failsafe
  UpdateSignalState();

}
void DJI_DBUS::FeedLine(void)
{
  if (_serial.available() > 24)
  {
    while (_serial.available() > 0)
    {
      inData = _serial.read();
      switch (feedState)
      {
        case 0:
          if (inData != 0x0f)
          {
            while (_serial.available() > 0)
            { //read the contents of in buffer this should resync the transmission
              inData = _serial.read();
            }
            return;
          }
          else
          {
            bufferIndex = 0;
            inBuffer[bufferIndex] = inData;
            inBuffer[24] = 0xff;
            feedState = 1;
          }
          break;
        case 1:
          bufferIndex++;
          inBuffer[bufferIndex] = inData;
          if (bufferIndex < 24 && _serial.available() == 0)
          {
            feedState = 0;
          }
          if (bufferIndex == 24)
          {
            feedState = 0;
            if (inBuffer[0] == 0x0f && inBuffer[24] == 0x00)
            {
              memcpy(sbusData, inBuffer, 25);
              toChannels = 1;
            }
          }
          break;
      }
    }
  }
}

void DJI_DBUS::UpdateSignalState(void)
{
  // Failsafe
  failsafe_status = DBUS_SIGNAL_OK;
  if (sbusData[23] & (1 << 2))
  {
    failsafe_status = DBUS_SIGNAL_FAILSAFE;
  }
  if (sbusData[23] & (1 << 3))
  {
    failsafe_status = DBUS_SIGNAL_LOST;
  }
}
