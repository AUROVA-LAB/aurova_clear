#ifndef _ROS_dbw_mkz_msgs_BrakeReport_h
#define _ROS_dbw_mkz_msgs_BrakeReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dbw_mkz_msgs/WatchdogCounter.h"

namespace dbw_mkz_msgs
{

  class BrakeReport : public ros::Msg
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
      typedef float _torque_input_type;
      _torque_input_type torque_input;
      typedef float _torque_cmd_type;
      _torque_cmd_type torque_cmd;
      typedef float _torque_output_type;
      _torque_output_type torque_output;
      typedef bool _boo_input_type;
      _boo_input_type boo_input;
      typedef bool _boo_cmd_type;
      _boo_cmd_type boo_cmd;
      typedef bool _boo_output_type;
      _boo_output_type boo_output;
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
      typedef bool _watchdog_braking_type;
      _watchdog_braking_type watchdog_braking;
      typedef bool _fault_wdc_type;
      _fault_wdc_type fault_wdc;
      typedef bool _fault_ch1_type;
      _fault_ch1_type fault_ch1;
      typedef bool _fault_ch2_type;
      _fault_ch2_type fault_ch2;
      typedef bool _fault_boo_type;
      _fault_boo_type fault_boo;

    BrakeReport():
      header(),
      pedal_input(0),
      pedal_cmd(0),
      pedal_output(0),
      torque_input(0),
      torque_cmd(0),
      torque_output(0),
      boo_input(0),
      boo_cmd(0),
      boo_output(0),
      enabled(0),
      override(0),
      driver(0),
      timeout(0),
      watchdog_counter(),
      watchdog_braking(0),
      fault_wdc(0),
      fault_ch1(0),
      fault_ch2(0),
      fault_boo(0)
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
        float real;
        uint32_t base;
      } u_torque_input;
      u_torque_input.real = this->torque_input;
      *(outbuffer + offset + 0) = (u_torque_input.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torque_input.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torque_input.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torque_input.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torque_input);
      union {
        float real;
        uint32_t base;
      } u_torque_cmd;
      u_torque_cmd.real = this->torque_cmd;
      *(outbuffer + offset + 0) = (u_torque_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torque_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torque_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torque_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torque_cmd);
      union {
        float real;
        uint32_t base;
      } u_torque_output;
      u_torque_output.real = this->torque_output;
      *(outbuffer + offset + 0) = (u_torque_output.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torque_output.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torque_output.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torque_output.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torque_output);
      union {
        bool real;
        uint8_t base;
      } u_boo_input;
      u_boo_input.real = this->boo_input;
      *(outbuffer + offset + 0) = (u_boo_input.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->boo_input);
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
      } u_boo_output;
      u_boo_output.real = this->boo_output;
      *(outbuffer + offset + 0) = (u_boo_output.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->boo_output);
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
      } u_watchdog_braking;
      u_watchdog_braking.real = this->watchdog_braking;
      *(outbuffer + offset + 0) = (u_watchdog_braking.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->watchdog_braking);
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
      union {
        bool real;
        uint8_t base;
      } u_fault_boo;
      u_fault_boo.real = this->fault_boo;
      *(outbuffer + offset + 0) = (u_fault_boo.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault_boo);
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
        float real;
        uint32_t base;
      } u_torque_input;
      u_torque_input.base = 0;
      u_torque_input.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torque_input.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torque_input.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torque_input.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torque_input = u_torque_input.real;
      offset += sizeof(this->torque_input);
      union {
        float real;
        uint32_t base;
      } u_torque_cmd;
      u_torque_cmd.base = 0;
      u_torque_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torque_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torque_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torque_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torque_cmd = u_torque_cmd.real;
      offset += sizeof(this->torque_cmd);
      union {
        float real;
        uint32_t base;
      } u_torque_output;
      u_torque_output.base = 0;
      u_torque_output.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torque_output.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torque_output.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torque_output.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torque_output = u_torque_output.real;
      offset += sizeof(this->torque_output);
      union {
        bool real;
        uint8_t base;
      } u_boo_input;
      u_boo_input.base = 0;
      u_boo_input.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->boo_input = u_boo_input.real;
      offset += sizeof(this->boo_input);
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
      } u_boo_output;
      u_boo_output.base = 0;
      u_boo_output.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->boo_output = u_boo_output.real;
      offset += sizeof(this->boo_output);
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
      } u_watchdog_braking;
      u_watchdog_braking.base = 0;
      u_watchdog_braking.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->watchdog_braking = u_watchdog_braking.real;
      offset += sizeof(this->watchdog_braking);
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
      union {
        bool real;
        uint8_t base;
      } u_fault_boo;
      u_fault_boo.base = 0;
      u_fault_boo.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fault_boo = u_fault_boo.real;
      offset += sizeof(this->fault_boo);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/BrakeReport"; };
    const char * getMD5(){ return "5716c7ce378fb5a251e0ff30ac24500e"; };

  };

}
#endif