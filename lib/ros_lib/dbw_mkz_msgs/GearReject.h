#ifndef _ROS_dbw_mkz_msgs_GearReject_h
#define _ROS_dbw_mkz_msgs_GearReject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class GearReject : public ros::Msg
  {
    public:
      typedef uint8_t _value_type;
      _value_type value;
      enum { NONE = 0               };
      enum { SHIFT_IN_PROGRESS = 1  };
      enum { OVERRIDE = 2           };
      enum { ROTARY_LOW = 3         };
      enum { ROTARY_PARK = 4        };
      enum { VEHICLE = 5            };

    GearReject():
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

    const char * getType(){ return "dbw_mkz_msgs/GearReject"; };
    const char * getMD5(){ return "fbe6a60b9b1c14ea65edee7fef99aaaa"; };

  };

}
#endif