#ifndef _ROS_dbw_mkz_msgs_BrakeCmd_h
#define _ROS_dbw_mkz_msgs_BrakeCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class BrakeCmd : public ros::Msg
  {
    public:
      typedef float _pedal_cmd_type;
      _pedal_cmd_type pedal_cmd;
      typedef uint8_t _pedal_cmd_type_type;
      _pedal_cmd_type_type pedal_cmd_type;
      typedef bool _boo_cmd_type;
      _boo_cmd_type boo_cmd;
      typedef bool _enable_type;
      _enable_type enable;
      typedef bool _clear_type;
      _clear_type clear;
      typedef bool _ignore_type;
      _ignore_type ignore;
      typedef uint8_t _count_type;
      _count_type count;
      enum { CMD_NONE = 0 };
      enum { CMD_PEDAL = 1    };
      enum { CMD_PERCENT = 2  };
      enum { CMD_TORQUE = 3   };
      enum { TORQUE_BOO = 520   };
      enum { TORQUE_MAX = 3412  };

    BrakeCmd():
      pedal_cmd(0),
      pedal_cmd_type(0),
      boo_cmd(0),
      enable(0),
      clear(0),
      ignore(0),
      count(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pedal_cmd;
      u_pedal_cmd.real = this->pedal_cmd;
      *(outbuffer + offset + 0) = (u_pedal_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pedal_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pedal_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pedal_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pedal_cmd);
      *(outbuffer + offset + 0) = (this->pedal_cmd_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pedal_cmd_type);
      union {
        bool real;
        uint8_t base;
      } u_boo_cmd;
      u_boo_cmd.real = this->boo_cmd;
      *(outbuffer + offset + 0) = (u_boo_cmd.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->boo_cmd);
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.real = this->enable;
      *(outbuffer + offset + 0) = (u_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enable);
      union {
        bool real;
        uint8_t base;
      } u_clear;
      u_clear.real = this->clear;
      *(outbuffer + offset + 0) = (u_clear.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->clear);
      union {
        bool real;
        uint8_t base;
      } u_ignore;
      u_ignore.real = this->ignore;
      *(outbuffer + offset + 0) = (u_ignore.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ignore);
      *(outbuffer + offset + 0) = (this->count >> (8 * 0)) & 0xFF;
      offset += sizeof(this->count);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_pedal_cmd;
      u_pedal_cmd.base = 0;
      u_pedal_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pedal_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pedal_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pedal_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pedal_cmd = u_pedal_cmd.real;
      offset += sizeof(this->pedal_cmd);
      this->pedal_cmd_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->pedal_cmd_type);
      union {
        bool real;
        uint8_t base;
      } u_boo_cmd;
      u_boo_cmd.base = 0;
      u_boo_cmd.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->boo_cmd = u_boo_cmd.real;
      offset += sizeof(this->boo_cmd);
      union {
        bool real;
        uint8_t base;
      } u_enable;
      u_enable.base = 0;
      u_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enable = u_enable.real;
      offset += sizeof(this->enable);
      union {
        bool real;
        uint8_t base;
      } u_clear;
      u_clear.base = 0;
      u_clear.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->clear = u_clear.real;
      offset += sizeof(this->clear);
      union {
        bool real;
        uint8_t base;
      } u_ignore;
      u_ignore.base = 0;
      u_ignore.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ignore = u_ignore.real;
      offset += sizeof(this->ignore);
      this->count =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->count);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/BrakeCmd"; };
    const char * getMD5(){ return "c0d20e1056976680942e85ab0959826c"; };

  };

}
#endif