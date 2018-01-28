#ifndef _ROS_dbw_mkz_msgs_ThrottleInfoReport_h
#define _ROS_dbw_mkz_msgs_ThrottleInfoReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dbw_mkz_msgs
{

  class ThrottleInfoReport : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _throttle_pc_type;
      _throttle_pc_type throttle_pc;
      typedef float _throttle_rate_type;
      _throttle_rate_type throttle_rate;
      typedef float _engine_rpm_type;
      _engine_rpm_type engine_rpm;

    ThrottleInfoReport():
      header(),
      throttle_pc(0),
      throttle_rate(0),
      engine_rpm(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_throttle_pc;
      u_throttle_pc.real = this->throttle_pc;
      *(outbuffer + offset + 0) = (u_throttle_pc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttle_pc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttle_pc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttle_pc.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle_pc);
      union {
        float real;
        uint32_t base;
      } u_throttle_rate;
      u_throttle_rate.real = this->throttle_rate;
      *(outbuffer + offset + 0) = (u_throttle_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttle_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttle_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttle_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle_rate);
      union {
        float real;
        uint32_t base;
      } u_engine_rpm;
      u_engine_rpm.real = this->engine_rpm;
      *(outbuffer + offset + 0) = (u_engine_rpm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_engine_rpm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_engine_rpm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_engine_rpm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->engine_rpm);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_throttle_pc;
      u_throttle_pc.base = 0;
      u_throttle_pc.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_throttle_pc.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_throttle_pc.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_throttle_pc.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->throttle_pc = u_throttle_pc.real;
      offset += sizeof(this->throttle_pc);
      union {
        float real;
        uint32_t base;
      } u_throttle_rate;
      u_throttle_rate.base = 0;
      u_throttle_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_throttle_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_throttle_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_throttle_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->throttle_rate = u_throttle_rate.real;
      offset += sizeof(this->throttle_rate);
      union {
        float real;
        uint32_t base;
      } u_engine_rpm;
      u_engine_rpm.base = 0;
      u_engine_rpm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_engine_rpm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_engine_rpm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_engine_rpm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->engine_rpm = u_engine_rpm.real;
      offset += sizeof(this->engine_rpm);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/ThrottleInfoReport"; };
    const char * getMD5(){ return "8255d20d2bbc661ad39074024259c71a"; };

  };

}
#endif