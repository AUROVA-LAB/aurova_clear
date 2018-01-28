#ifndef _ROS_dbw_mkz_msgs_Gear_h
#define _ROS_dbw_mkz_msgs_Gear_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class Gear : public ros::Msg
  {
    public:
      typedef uint8_t _gear_type;
      _gear_type gear;
      enum { NONE = 0 };
      enum { PARK = 1 };
      enum { REVERSE = 2 };
      enum { NEUTRAL = 3 };
      enum { DRIVE = 4 };
      enum { LOW = 5 };

    Gear():
      gear(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->gear >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gear);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->gear =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gear);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/Gear"; };
    const char * getMD5(){ return "79b3cd667a7556f4bc4a66af7d189c96"; };

  };

}
#endif