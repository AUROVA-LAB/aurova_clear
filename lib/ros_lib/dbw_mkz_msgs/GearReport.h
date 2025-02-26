#ifndef _ROS_dbw_mkz_msgs_GearReport_h
#define _ROS_dbw_mkz_msgs_GearReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dbw_mkz_msgs/Gear.h"
#include "dbw_mkz_msgs/GearReject.h"

namespace dbw_mkz_msgs
{

  class GearReport : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef dbw_mkz_msgs::Gear _state_type;
      _state_type state;
      typedef dbw_mkz_msgs::Gear _cmd_type;
      _cmd_type cmd;
      typedef dbw_mkz_msgs::GearReject _reject_type;
      _reject_type reject;
      typedef bool _override_type;
      _override_type override;
      typedef bool _fault_bus_type;
      _fault_bus_type fault_bus;

    GearReport():
      header(),
      state(),
      cmd(),
      reject(),
      override(0),
      fault_bus(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->state.serialize(outbuffer + offset);
      offset += this->cmd.serialize(outbuffer + offset);
      offset += this->reject.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_override;
      u_override.real = this->override;
      *(outbuffer + offset + 0) = (u_override.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->override);
      union {
        bool real;
        uint8_t base;
      } u_fault_bus;
      u_fault_bus.real = this->fault_bus;
      *(outbuffer + offset + 0) = (u_fault_bus.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault_bus);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->state.deserialize(inbuffer + offset);
      offset += this->cmd.deserialize(inbuffer + offset);
      offset += this->reject.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_override;
      u_override.base = 0;
      u_override.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->override = u_override.real;
      offset += sizeof(this->override);
      union {
        bool real;
        uint8_t base;
      } u_fault_bus;
      u_fault_bus.base = 0;
      u_fault_bus.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fault_bus = u_fault_bus.real;
      offset += sizeof(this->fault_bus);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/GearReport"; };
    const char * getMD5(){ return "785b94d5bfee677e7f0da982153f2711"; };

  };

}
#endif