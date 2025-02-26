#ifndef _ROS_dbw_mkz_msgs_ThrottleReport_h
#define _ROS_dbw_mkz_msgs_ThrottleReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dbw_mkz_msgs/WatchdogCounter.h"

namespace dbw_mkz_msgs
{

  class ThrottleReport : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _pedal_input_type;
      _pedal_input_type pedal_input;
      typedef float _pedal_cmd_type;
      _pedal_cmd_type pedal_cmd;
      typedef float _pedal_output_type;
      _pedal_output_type pedal_output;
      typedef bool _enabled_type;
      _enabled_type enabled;
      typedef bool _override_type;
      _override_type override;
      typedef bool _driver_type;
      _driver_type driver;
      typedef bool _timeout_type;
      _timeout_type timeout;
      typedef dbw_mkz_msgs::WatchdogCounter _watchdog_counter_type;
      _watchdog_counter_type watchdog_counter;
      typedef bool _fault_wdc_type;
      _fault_wdc_type fault_wdc;
      typedef bool _fault_ch1_type;
      _fault_ch1_type fault_ch1;
      typedef bool _fault_ch2_type;
      _fault_ch2_type fault_ch2;

    ThrottleReport():
      header(),
      pedal_input(0),
      pedal_cmd(0),
      pedal_output(0),
      enabled(0),
      override(0),
      driver(0),
      timeout(0),
      watchdog_counter(),
      fault_wdc(0),
      fault_ch1(0),
      fault_ch2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_pedal_input;
      u_pedal_input.real = this->pedal_input;
      *(outbuffer + offset + 0) = (u_pedal_input.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pedal_input.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pedal_input.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pedal_input.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pedal_input);
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
      union {
        float real;
        uint32_t base;
      } u_pedal_output;
      u_pedal_output.real = this->pedal_output;
      *(outbuffer + offset + 0) = (u_pedal_output.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pedal_output.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pedal_output.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pedal_output.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pedal_output);
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.real = this->enabled;
      *(outbuffer + offset + 0) = (u_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enabled);
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
      } u_driver;
      u_driver.real = this->driver;
      *(outbuffer + offset + 0) = (u_driver.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->driver);
      union {
        bool real;
        uint8_t base;
      } u_timeout;
      u_timeout.real = this->timeout;
      *(outbuffer + offset + 0) = (u_timeout.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timeout);
      offset += this->watchdog_counter.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_fault_wdc;
      u_fault_wdc.real = this->fault_wdc;
      *(outbuffer + offset + 0) = (u_fault_wdc.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault_wdc);
      union {
        bool real;
        uint8_t base;
      } u_fault_ch1;
      u_fault_ch1.real = this->fault_ch1;
      *(outbuffer + offset + 0) = (u_fault_ch1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault_ch1);
      union {
        bool real;
        uint8_t base;
      } u_fault_ch2;
      u_fault_ch2.real = this->fault_ch2;
      *(outbuffer + offset + 0) = (u_fault_ch2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault_ch2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_pedal_input;
      u_pedal_input.base = 0;
      u_pedal_input.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pedal_input.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pedal_input.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pedal_input.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pedal_input = u_pedal_input.real;
      offset += sizeof(this->pedal_input);
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
      union {
        float real;
        uint32_t base;
      } u_pedal_output;
      u_pedal_output.base = 0;
      u_pedal_output.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pedal_output.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pedal_output.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pedal_output.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pedal_output = u_pedal_output.real;
      offset += sizeof(this->pedal_output);
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.base = 0;
      u_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enabled = u_enabled.real;
      offset += sizeof(this->enabled);
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
      } u_driver;
      u_driver.base = 0;
      u_driver.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->driver = u_driver.real;
      offset += sizeof(this->driver);
      union {
        bool real;
        uint8_t base;
      } u_timeout;
      u_timeout.base = 0;
      u_timeout.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->timeout = u_timeout.real;
      offset += sizeof(this->timeout);
      offset += this->watchdog_counter.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_fault_wdc;
      u_fault_wdc.base = 0;
      u_fault_wdc.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fault_wdc = u_fault_wdc.real;
      offset += sizeof(this->fault_wdc);
      union {
        bool real;
        uint8_t base;
      } u_fault_ch1;
      u_fault_ch1.base = 0;
      u_fault_ch1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fault_ch1 = u_fault_ch1.real;
      offset += sizeof(this->fault_ch1);
      union {
        bool real;
        uint8_t base;
      } u_fault_ch2;
      u_fault_ch2.base = 0;
      u_fault_ch2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fault_ch2 = u_fault_ch2.real;
      offset += sizeof(this->fault_ch2);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/ThrottleReport"; };
    const char * getMD5(){ return "dc371d36db36a47de2ffaa1302bf4aec"; };

  };

}
#endif