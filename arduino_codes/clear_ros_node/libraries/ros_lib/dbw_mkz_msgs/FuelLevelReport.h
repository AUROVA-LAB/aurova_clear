#ifndef _ROS_dbw_mkz_msgs_FuelLevelReport_h
#define _ROS_dbw_mkz_msgs_FuelLevelReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dbw_mkz_msgs
{

  class FuelLevelReport : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _fuel_level_type;
      _fuel_level_type fuel_level;

    FuelLevelReport():
      header(),
      fuel_level(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_fuel_level;
      u_fuel_level.real = this->fuel_level;
      *(outbuffer + offset + 0) = (u_fuel_level.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fuel_level.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fuel_level.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fuel_level.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fuel_level);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_fuel_level;
      u_fuel_level.base = 0;
      u_fuel_level.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fuel_level.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fuel_level.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fuel_level.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fuel_level = u_fuel_level.real;
      offset += sizeof(this->fuel_level);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/FuelLevelReport"; };
    const char * getMD5(){ return "f5ec1964dbda02fda82785b8035744e4"; };

  };

}
#endif