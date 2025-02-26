#ifndef _ROS_dbw_mkz_msgs_SteeringCmd_h
#define _ROS_dbw_mkz_msgs_SteeringCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class SteeringCmd : public ros::Msg
  {
    public:
      typedef float _steering_wheel_angle_cmd_type;
      _steering_wheel_angle_cmd_type steering_wheel_angle_cmd;
      typedef float _steering_wheel_angle_velocity_type;
      _steering_wheel_angle_velocity_type steering_wheel_angle_velocity;
      typedef bool _enable_type;
      _enable_type enable;
      typedef bool _clear_type;
      _clear_type clear;
      typedef bool _ignore_type;
      _ignore_type ignore;
      typedef bool _quiet_type;
      _quiet_type quiet;
      typedef uint8_t _count_type;
      _count_type count;

    SteeringCmd():
      steering_wheel_angle_cmd(0),
      steering_wheel_angle_velocity(0),
      enable(0),
      clear(0),
      ignore(0),
      quiet(0),
      count(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_steering_wheel_angle_cmd;
      u_steering_wheel_angle_cmd.real = this->steering_wheel_angle_cmd;
      *(outbuffer + offset + 0) = (u_steering_wheel_angle_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_wheel_angle_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_wheel_angle_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_wheel_angle_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steering_wheel_angle_cmd);
      union {
        float real;
        uint32_t base;
      } u_steering_wheel_angle_velocity;
      u_steering_wheel_angle_velocity.real = this->steering_wheel_angle_velocity;
      *(outbuffer + offset + 0) = (u_steering_wheel_angle_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_wheel_angle_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_wheel_angle_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_wheel_angle_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steering_wheel_angle_velocity);
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
      union {
        bool real;
        uint8_t base;
      } u_quiet;
      u_quiet.real = this->quiet;
      *(outbuffer + offset + 0) = (u_quiet.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->quiet);
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
      } u_steering_wheel_angle_cmd;
      u_steering_wheel_angle_cmd.base = 0;
      u_steering_wheel_angle_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_wheel_angle_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_wheel_angle_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_wheel_angle_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steering_wheel_angle_cmd = u_steering_wheel_angle_cmd.real;
      offset += sizeof(this->steering_wheel_angle_cmd);
      union {
        float real;
        uint32_t base;
      } u_steering_wheel_angle_velocity;
      u_steering_wheel_angle_velocity.base = 0;
      u_steering_wheel_angle_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_wheel_angle_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_wheel_angle_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_wheel_angle_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steering_wheel_angle_velocity = u_steering_wheel_angle_velocity.real;
      offset += sizeof(this->steering_wheel_angle_velocity);
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
      union {
        bool real;
        uint8_t base;
      } u_quiet;
      u_quiet.base = 0;
      u_quiet.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->quiet = u_quiet.real;
      offset += sizeof(this->quiet);
      this->count =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->count);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/SteeringCmd"; };
    const char * getMD5(){ return "ff1fa11624bdc2aff2aeee5aa6014057"; };

  };

}
#endif