#ifndef _ROS_can_msgs_Frame_h
#define _ROS_can_msgs_Frame_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace can_msgs
{

  class Frame : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _id_type;
      _id_type id;
      typedef bool _is_rtr_type;
      _is_rtr_type is_rtr;
      typedef bool _is_extended_type;
      _is_extended_type is_extended;
      typedef bool _is_error_type;
      _is_error_type is_error;
      typedef uint8_t _dlc_type;
      _dlc_type dlc;
      uint8_t data[8];

    Frame():
      header(),
      id(0),
      is_rtr(0),
      is_extended(0),
      is_error(0),
      dlc(0),
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        bool real;
        uint8_t base;
      } u_is_rtr;
      u_is_rtr.real = this->is_rtr;
      *(outbuffer + offset + 0) = (u_is_rtr.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_rtr);
      union {
        bool real;
        uint8_t base;
      } u_is_extended;
      u_is_extended.real = this->is_extended;
      *(outbuffer + offset + 0) = (u_is_extended.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_extended);
      union {
        bool real;
        uint8_t base;
      } u_is_error;
      u_is_error.real = this->is_error;
      *(outbuffer + offset + 0) = (u_is_error.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_error);
      *(outbuffer + offset + 0) = (this->dlc >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dlc);
      for( uint32_t i = 0; i < 8; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->id =  ((uint32_t) (*(inbuffer + offset)));
      this->id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->id);
      union {
        bool real;
        uint8_t base;
      } u_is_rtr;
      u_is_rtr.base = 0;
      u_is_rtr.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_rtr = u_is_rtr.real;
      offset += sizeof(this->is_rtr);
      union {
        bool real;
        uint8_t base;
      } u_is_extended;
      u_is_extended.base = 0;
      u_is_extended.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_extended = u_is_extended.real;
      offset += sizeof(this->is_extended);
      union {
        bool real;
        uint8_t base;
      } u_is_error;
      u_is_error.base = 0;
      u_is_error.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_error = u_is_error.real;
      offset += sizeof(this->is_error);
      this->dlc =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->dlc);
      for( uint32_t i = 0; i < 8; i++){
      this->data[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    const char * getType(){ return "can_msgs/Frame"; };
    const char * getMD5(){ return "64ae5cebf967dc6aae4e78f5683a5b25"; };

  };

}
#endif