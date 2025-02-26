#ifndef _ROS_dbw_mkz_msgs_WheelSpeedReport_h
#define _ROS_dbw_mkz_msgs_WheelSpeedReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dbw_mkz_msgs
{

  class WheelSpeedReport : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _front_left_type;
      _front_left_type front_left;
      typedef float _front_right_type;
      _front_right_type front_right;
      typedef float _rear_left_type;
      _rear_left_type rear_left;
      typedef float _rear_right_type;
      _rear_right_type rear_right;

    WheelSpeedReport():
      header(),
      front_left(0),
      front_right(0),
      rear_left(0),
      rear_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_front_left;
      u_front_left.real = this->front_left;
      *(outbuffer + offset + 0) = (u_front_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front_left);
      union {
        float real;
        uint32_t base;
      } u_front_right;
      u_front_right.real = this->front_right;
      *(outbuffer + offset + 0) = (u_front_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front_right);
      union {
        float real;
        uint32_t base;
      } u_rear_left;
      u_rear_left.real = this->rear_left;
      *(outbuffer + offset + 0) = (u_rear_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear_left);
      union {
        float real;
        uint32_t base;
      } u_rear_right;
      u_rear_right.real = this->rear_right;
      *(outbuffer + offset + 0) = (u_rear_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rear_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rear_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rear_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rear_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_front_left;
      u_front_left.base = 0;
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->front_left = u_front_left.real;
      offset += sizeof(this->front_left);
      union {
        float real;
        uint32_t base;
      } u_front_right;
      u_front_right.base = 0;
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->front_right = u_front_right.real;
      offset += sizeof(this->front_right);
      union {
        float real;
        uint32_t base;
      } u_rear_left;
      u_rear_left.base = 0;
      u_rear_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rear_left = u_rear_left.real;
      offset += sizeof(this->rear_left);
      union {
        float real;
        uint32_t base;
      } u_rear_right;
      u_rear_right.base = 0;
      u_rear_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rear_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rear_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rear_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rear_right = u_rear_right.real;
      offset += sizeof(this->rear_right);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/WheelSpeedReport"; };
    const char * getMD5(){ return "a2c91f746e5d8bec139c834f92ac7468"; };

  };

}
#endif