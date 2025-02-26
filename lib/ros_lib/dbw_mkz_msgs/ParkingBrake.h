#ifndef _ROS_dbw_mkz_msgs_ParkingBrake_h
#define _ROS_dbw_mkz_msgs_ParkingBrake_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class ParkingBrake : public ros::Msg
  {
    public:
      typedef uint8_t _status_type;
      _status_type status;
      enum { OFF = 0 };
      enum { TRANS = 1 };
      enum { ON = 2 };
      enum { FAULT = 3 };

    ParkingBrake():
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

    const char * getType(){ return "dbw_mkz_msgs/ParkingBrake"; };
    const char * getMD5(){ return "2280b2c9c46fd98be0f067aa92f74fc4"; };

  };

}
#endif