#ifndef _ROS_ublox_msgs_CfgDGNSS_h
#define _ROS_ublox_msgs_CfgDGNSS_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ublox_msgs
{

  class CfgDGNSS : public ros::Msg
  {
    public:
      typedef uint8_t _dgnssMode_type;
      _dgnssMode_type dgnssMode;
      uint8_t reserved0[3];
      enum { CLASS_ID =  6 };
      enum { MESSAGE_ID =  112 };
      enum { DGNSS_MODE_RTK_FLOAT =  2     };
      enum { DGNSS_MODE_RTK_FIXED =  3     };

    CfgDGNSS():
      dgnssMode(0),
      reserved0()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dgnssMode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dgnssMode);
      for( uint32_t i = 0; i < 3; i++){
      *(outbuffer + offset + 0) = (this->reserved0[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved0[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->dgnssMode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dgnssMode);
      for( uint32_t i = 0; i < 3; i++){
      this->reserved0[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->reserved0[i]);
      }
     return offset;
    }

    const char * getType(){ return "ublox_msgs/CfgDGNSS"; };
    const char * getMD5(){ return "2ec4391f93581f9db720bbc0af2b613e"; };

  };

}
#endif