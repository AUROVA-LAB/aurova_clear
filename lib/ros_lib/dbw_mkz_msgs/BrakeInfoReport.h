#ifndef _ROS_dbw_mkz_msgs_BrakeInfoReport_h
#define _ROS_dbw_mkz_msgs_BrakeInfoReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dbw_mkz_msgs/HillStartAssist.h"
#include "dbw_mkz_msgs/ParkingBrake.h"

namespace dbw_mkz_msgs
{

  class BrakeInfoReport : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _brake_torque_request_type;
      _brake_torque_request_type brake_torque_request;
      typedef float _brake_torque_actual_type;
      _brake_torque_actual_type brake_torque_actual;
      typedef float _wheel_torque_actual_type;
      _wheel_torque_actual_type wheel_torque_actual;
      typedef float _accel_over_ground_type;
      _accel_over_ground_type accel_over_ground;
      typedef dbw_mkz_msgs::HillStartAssist _hsa_type;
      _hsa_type hsa;
      typedef bool _abs_active_type;
      _abs_active_type abs_active;
      typedef bool _abs_enabled_type;
      _abs_enabled_type abs_enabled;
      typedef bool _stab_active_type;
      _stab_active_type stab_active;
      typedef bool _stab_enabled_type;
      _stab_enabled_type stab_enabled;
      typedef bool _trac_active_type;
      _trac_active_type trac_active;
      typedef bool _trac_enabled_type;
      _trac_enabled_type trac_enabled;
      typedef dbw_mkz_msgs::ParkingBrake _parking_brake_type;
      _parking_brake_type parking_brake;
      typedef bool _stationary_type;
      _stationary_type stationary;

    BrakeInfoReport():
      header(),
      brake_torque_request(0),
      brake_torque_actual(0),
      wheel_torque_actual(0),
      accel_over_ground(0),
      hsa(),
      abs_active(0),
      abs_enabled(0),
      stab_active(0),
      stab_enabled(0),
      trac_active(0),
      trac_enabled(0),
      parking_brake(),
      stationary(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_brake_torque_request;
      u_brake_torque_request.real = this->brake_torque_request;
      *(outbuffer + offset + 0) = (u_brake_torque_request.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_brake_torque_request.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_brake_torque_request.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_brake_torque_request.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->brake_torque_request);
      union {
        float real;
        uint32_t base;
      } u_brake_torque_actual;
      u_brake_torque_actual.real = this->brake_torque_actual;
      *(outbuffer + offset + 0) = (u_brake_torque_actual.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_brake_torque_actual.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_brake_torque_actual.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_brake_torque_actual.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->brake_torque_actual);
      union {
        float real;
        uint32_t base;
      } u_wheel_torque_actual;
      u_wheel_torque_actual.real = this->wheel_torque_actual;
      *(outbuffer + offset + 0) = (u_wheel_torque_actual.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_torque_actual.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_torque_actual.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_torque_actual.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_torque_actual);
      union {
        float real;
        uint32_t base;
      } u_accel_over_ground;
      u_accel_over_ground.real = this->accel_over_ground;
      *(outbuffer + offset + 0) = (u_accel_over_ground.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_accel_over_ground.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_accel_over_ground.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_accel_over_ground.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->accel_over_ground);
      offset += this->hsa.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_abs_active;
      u_abs_active.real = this->abs_active;
      *(outbuffer + offset + 0) = (u_abs_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->abs_active);
      union {
        bool real;
        uint8_t base;
      } u_abs_enabled;
      u_abs_enabled.real = this->abs_enabled;
      *(outbuffer + offset + 0) = (u_abs_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->abs_enabled);
      union {
        bool real;
        uint8_t base;
      } u_stab_active;
      u_stab_active.real = this->stab_active;
      *(outbuffer + offset + 0) = (u_stab_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stab_active);
      union {
        bool real;
        uint8_t base;
      } u_stab_enabled;
      u_stab_enabled.real = this->stab_enabled;
      *(outbuffer + offset + 0) = (u_stab_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stab_enabled);
      union {
        bool real;
        uint8_t base;
      } u_trac_active;
      u_trac_active.real = this->trac_active;
      *(outbuffer + offset + 0) = (u_trac_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->trac_active);
      union {
        bool real;
        uint8_t base;
      } u_trac_enabled;
      u_trac_enabled.real = this->trac_enabled;
      *(outbuffer + offset + 0) = (u_trac_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->trac_enabled);
      offset += this->parking_brake.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_stationary;
      u_stationary.real = this->stationary;
      *(outbuffer + offset + 0) = (u_stationary.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->stationary);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_brake_torque_request;
      u_brake_torque_request.base = 0;
      u_brake_torque_request.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_brake_torque_request.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_brake_torque_request.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_brake_torque_request.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->brake_torque_request = u_brake_torque_request.real;
      offset += sizeof(this->brake_torque_request);
      union {
        float real;
        uint32_t base;
      } u_brake_torque_actual;
      u_brake_torque_actual.base = 0;
      u_brake_torque_actual.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_brake_torque_actual.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_brake_torque_actual.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_brake_torque_actual.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->brake_torque_actual = u_brake_torque_actual.real;
      offset += sizeof(this->brake_torque_actual);
      union {
        float real;
        uint32_t base;
      } u_wheel_torque_actual;
      u_wheel_torque_actual.base = 0;
      u_wheel_torque_actual.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_torque_actual.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_torque_actual.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_torque_actual.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_torque_actual = u_wheel_torque_actual.real;
      offset += sizeof(this->wheel_torque_actual);
      union {
        float real;
        uint32_t base;
      } u_accel_over_ground;
      u_accel_over_ground.base = 0;
      u_accel_over_ground.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_accel_over_ground.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_accel_over_ground.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_accel_over_ground.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->accel_over_ground = u_accel_over_ground.real;
      offset += sizeof(this->accel_over_ground);
      offset += this->hsa.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_abs_active;
      u_abs_active.base = 0;
      u_abs_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->abs_active = u_abs_active.real;
      offset += sizeof(this->abs_active);
      union {
        bool real;
        uint8_t base;
      } u_abs_enabled;
      u_abs_enabled.base = 0;
      u_abs_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->abs_enabled = u_abs_enabled.real;
      offset += sizeof(this->abs_enabled);
      union {
        bool real;
        uint8_t base;
      } u_stab_active;
      u_stab_active.base = 0;
      u_stab_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stab_active = u_stab_active.real;
      offset += sizeof(this->stab_active);
      union {
        bool real;
        uint8_t base;
      } u_stab_enabled;
      u_stab_enabled.base = 0;
      u_stab_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stab_enabled = u_stab_enabled.real;
      offset += sizeof(this->stab_enabled);
      union {
        bool real;
        uint8_t base;
      } u_trac_active;
      u_trac_active.base = 0;
      u_trac_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->trac_active = u_trac_active.real;
      offset += sizeof(this->trac_active);
      union {
        bool real;
        uint8_t base;
      } u_trac_enabled;
      u_trac_enabled.base = 0;
      u_trac_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->trac_enabled = u_trac_enabled.real;
      offset += sizeof(this->trac_enabled);
      offset += this->parking_brake.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_stationary;
      u_stationary.base = 0;
      u_stationary.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->stationary = u_stationary.real;
      offset += sizeof(this->stationary);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/BrakeInfoReport"; };
    const char * getMD5(){ return "fc88af128b5b3213ea25ab325a9b3bbb"; };

  };

}
#endif