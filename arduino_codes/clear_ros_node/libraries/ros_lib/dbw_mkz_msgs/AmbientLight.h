#ifndef _ROS_dbw_mkz_msgs_AmbientLight_h
#define _ROS_dbw_mkz_msgs_AmbientLight_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class AmbientLight : public ros::Msg
  {
    public:
      typedef uint8_t _status_type;
      _status_type status;
      enum { DARK = 0 };
      enum { LIGHT = 1 };
      enum { TWILIGHT = 2 };
      enum { TUNNEL_ON = 3 };
      enum { TUNNEL_OFF = 4 };
      enum { NO_DATA = 7 };

    AmbientLight():
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

    const char * getType(){ return "dbw_mkz_msgs/AmbientLight"; };
    const char * getMD5(){ return "1ddf5fa71f1f6a75449370898bc9b6ad"; };

  };

}
#endif