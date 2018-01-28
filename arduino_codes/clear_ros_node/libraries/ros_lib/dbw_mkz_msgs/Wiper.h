#ifndef _ROS_dbw_mkz_msgs_Wiper_h
#define _ROS_dbw_mkz_msgs_Wiper_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class Wiper : public ros::Msg
  {
    public:
      typedef uint8_t _status_type;
      _status_type status;
      enum { OFF = 0 };
      enum { AUTO_OFF = 1 };
      enum { OFF_MOVING = 2 };
      enum { MANUAL_OFF = 3 };
      enum { MANUAL_ON = 4 };
      enum { MANUAL_LOW = 5 };
      enum { MANUAL_HIGH = 6 };
      enum { MIST_FLICK = 7 };
      enum { WASH = 8 };
      enum { AUTO_LOW = 9 };
      enum { AUTO_HIGH = 10 };
      enum { COURTESYWIPE = 11 };
      enum { AUTO_ADJUST = 12 };
      enum { RESERVED = 13 };
      enum { STALLED = 14 };
      enum { NO_DATA = 15 };

    Wiper():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/Wiper"; };
    const char * getMD5(){ return "7fccb48d5d1df108afaa89f8fc14ce1c"; };

  };

}
#endif