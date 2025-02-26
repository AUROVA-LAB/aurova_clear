#ifndef _ROS_dbw_mkz_msgs_GearCmd_h
#define _ROS_dbw_mkz_msgs_GearCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dbw_mkz_msgs/Gear.h"

namespace dbw_mkz_msgs
{

  class GearCmd : public ros::Msg
  {
    public:
      typedef dbw_mkz_msgs::Gear _cmd_type;
      _cmd_type cmd;
      typedef bool _clear_type;
      _clear_type clear;

    GearCmd():
      cmd(),
      clear(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->cmd.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_clear;
      u_clear.real = this->clear;
      *(outbuffer + offset + 0) = (u_clear.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clear);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->cmd.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_clear;
      u_clear.base = 0;
      u_clear.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->clear = u_clear.real;
      offset += sizeof(this->clear);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/GearCmd"; };
    const char * getMD5(){ return "14694bb9a291d2a80b8033843d95776e"; };

  };

}
#endif