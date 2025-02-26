#ifndef _ROS_dbw_mkz_msgs_TwistCmd_h
#define _ROS_dbw_mkz_msgs_TwistCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Twist.h"

namespace dbw_mkz_msgs
{

  class TwistCmd : public ros::Msg
  {
    public:
      typedef geometry_msgs::Twist _twist_type;
      _twist_type twist;
      typedef float _accel_limit_type;
      _accel_limit_type accel_limit;
      typedef float _decel_limit_type;
      _decel_limit_type decel_limit;

    TwistCmd():
      twist(),
      accel_limit(0),
      decel_limit(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->twist.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_accel_limit;
      u_accel_limit.real = this->accel_limit;
      *(outbuffer + offset + 0) = (u_accel_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_limit);
      union {
        float real;
        uint32_t base;
      } u_decel_limit;
      u_decel_limit.real = this->decel_limit;
      *(outbuffer + offset + 0) = (u_decel_limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_decel_limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_decel_limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_decel_limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->decel_limit);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->twist.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_accel_limit;
      u_accel_limit.base = 0;
      u_accel_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accel_limit = u_accel_limit.real;
      offset += sizeof(this->accel_limit);
      union {
        float real;
        uint32_t base;
      } u_decel_limit;
      u_decel_limit.base = 0;
      u_decel_limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_decel_limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_decel_limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_decel_limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->decel_limit = u_decel_limit.real;
      offset += sizeof(this->decel_limit);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/TwistCmd"; };
    const char * getMD5(){ return "ef873397d04f1a8acdfa4bcab4392286"; };

  };

}
#endif