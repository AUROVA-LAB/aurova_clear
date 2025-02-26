#ifndef _ROS_dbw_mkz_msgs_SteeringReport_h
#define _ROS_dbw_mkz_msgs_SteeringReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dbw_mkz_msgs
{

  class SteeringReport : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _steering_wheel_angle_type;
      _steering_wheel_angle_type steering_wheel_angle;
      typedef float _steering_wheel_angle_cmd_type;
      _steering_wheel_angle_cmd_type steering_wheel_angle_cmd;
      typedef float _steering_wheel_torque_type;
      _steering_wheel_torque_type steering_wheel_torque;
      typedef float _speed_type;
      _speed_type speed;
      typedef bool _enabled_type;
      _enabled_type enabled;
      typedef bool _override_type;
      _override_type override;
      typedef bool _timeout_type;
      _timeout_type timeout;
      typedef bool _fault_wdc_type;
      _fault_wdc_type fault_wdc;
      typedef bool _fault_bus1_type;
      _fault_bus1_type fault_bus1;
      typedef bool _fault_bus2_type;
      _fault_bus2_type fault_bus2;
      typedef bool _fault_calibration_type;
      _fault_calibration_type fault_calibration;

    SteeringReport():
      header(),
      steering_wheel_angle(0),
      steering_wheel_angle_cmd(0),
      steering_wheel_torque(0),
      speed(0),
      enabled(0),
      override(0),
      timeout(0),
      fault_wdc(0),
      fault_bus1(0),
      fault_bus2(0),
      fault_calibration(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_steering_wheel_angle;
      u_steering_wheel_angle.real = this->steering_wheel_angle;
      *(outbuffer + offset + 0) = (u_steering_wheel_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_wheel_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_wheel_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_wheel_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steering_wheel_angle);
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
      } u_steering_wheel_torque;
      u_steering_wheel_torque.real = this->steering_wheel_torque;
      *(outbuffer + offset + 0) = (u_steering_wheel_torque.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_wheel_torque.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_wheel_torque.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_wheel_torque.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steering_wheel_torque);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
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
      } u_timeout;
      u_timeout.real = this->timeout;
      *(outbuffer + offset + 0) = (u_timeout.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timeout);
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
      } u_fault_bus1;
      u_fault_bus1.real = this->fault_bus1;
      *(outbuffer + offset + 0) = (u_fault_bus1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault_bus1);
      union {
        bool real;
        uint8_t base;
      } u_fault_bus2;
      u_fault_bus2.real = this->fault_bus2;
      *(outbuffer + offset + 0) = (u_fault_bus2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault_bus2);
      union {
        bool real;
        uint8_t base;
      } u_fault_calibration;
      u_fault_calibration.real = this->fault_calibration;
      *(outbuffer + offset + 0) = (u_fault_calibration.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault_calibration);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_steering_wheel_angle;
      u_steering_wheel_angle.base = 0;
      u_steering_wheel_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_wheel_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_wheel_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_wheel_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steering_wheel_angle = u_steering_wheel_angle.real;
      offset += sizeof(this->steering_wheel_angle);
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
      } u_steering_wheel_torque;
      u_steering_wheel_torque.base = 0;
      u_steering_wheel_torque.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_wheel_torque.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_wheel_torque.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_wheel_torque.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steering_wheel_torque = u_steering_wheel_torque.real;
      offset += sizeof(this->steering_wheel_torque);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
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
      } u_timeout;
      u_timeout.base = 0;
      u_timeout.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->timeout = u_timeout.real;
      offset += sizeof(this->timeout);
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
      } u_fault_bus1;
      u_fault_bus1.base = 0;
      u_fault_bus1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fault_bus1 = u_fault_bus1.real;
      offset += sizeof(this->fault_bus1);
      union {
        bool real;
        uint8_t base;
      } u_fault_bus2;
      u_fault_bus2.base = 0;
      u_fault_bus2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fault_bus2 = u_fault_bus2.real;
      offset += sizeof(this->fault_bus2);
      union {
        bool real;
        uint8_t base;
      } u_fault_calibration;
      u_fault_calibration.base = 0;
      u_fault_calibration.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fault_calibration = u_fault_calibration.real;
      offset += sizeof(this->fault_calibration);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/SteeringReport"; };
    const char * getMD5(){ return "435efc512abdd87ef2f942c0e8ed296d"; };

  };

}
#endif