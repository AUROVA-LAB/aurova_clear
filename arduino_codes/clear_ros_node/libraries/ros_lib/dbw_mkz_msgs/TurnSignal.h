#ifndef _ROS_dbw_mkz_msgs_TurnSignal_h
#define _ROS_dbw_mkz_msgs_TurnSignal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class TurnSignal : public ros::Msg
  {
    public:
      typedef uint8_t _value_type;
      _value_type value;
      enum { NONE = 0 };
      enum { LEFT = 1 };
      enum { RIGHT = 2 };

    TurnSignal():
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->value >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->value =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->value);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/TurnSignal"; };
    const char * getMD5(){ return "52e47837caa6386d671c442331ecc1cd"; };

  };

}
#endif