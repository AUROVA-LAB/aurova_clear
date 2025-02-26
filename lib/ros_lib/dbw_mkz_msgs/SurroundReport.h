#ifndef _ROS_dbw_mkz_msgs_SurroundReport_h
#define _ROS_dbw_mkz_msgs_SurroundReport_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dbw_mkz_msgs
{

  class SurroundReport : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef bool _cta_left_alert_type;
      _cta_left_alert_type cta_left_alert;
      typedef bool _cta_right_alert_type;
      _cta_right_alert_type cta_right_alert;
      typedef bool _cta_left_enabled_type;
      _cta_left_enabled_type cta_left_enabled;
      typedef bool _cta_right_enabled_type;
      _cta_right_enabled_type cta_right_enabled;
      typedef bool _blis_left_alert_type;
      _blis_left_alert_type blis_left_alert;
      typedef bool _blis_right_alert_type;
      _blis_right_alert_type blis_right_alert;
      typedef bool _blis_left_enabled_type;
      _blis_left_enabled_type blis_left_enabled;
      typedef bool _blis_right_enabled_type;
      _blis_right_enabled_type blis_right_enabled;
      typedef bool _sonar_enabled_type;
      _sonar_enabled_type sonar_enabled;
      typedef bool _sonar_fault_type;
      _sonar_fault_type sonar_fault;
      float sonar[12];
      enum { FRONT_LEFT_SIDE = 0 };
      enum { FRONT_LEFT_CORNER = 1 };
      enum { FRONT_LEFT_CENTER = 2 };
      enum { FRONT_RIGHT_CENTER = 3 };
      enum { FRONT_RIGHT_CORNER = 4 };
      enum { FRONT_RIGHT_SIDE = 5 };
      enum { REAR_LEFT_SIDE = 6 };
      enum { REAR_LEFT_CORNER = 7 };
      enum { REAR_LEFT_CENTER = 8 };
      enum { REAR_RIGHT_CENTER = 9 };
      enum { REAR_RIGHT_CORNER = 10 };
      enum { REAR_RIGHT_SIDE = 11 };

    SurroundReport():
      header(),
      cta_left_alert(0),
      cta_right_alert(0),
      cta_left_enabled(0),
      cta_right_enabled(0),
      blis_left_alert(0),
      blis_right_alert(0),
      blis_left_enabled(0),
      blis_right_enabled(0),
      sonar_enabled(0),
      sonar_fault(0),
      sonar()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_cta_left_alert;
      u_cta_left_alert.real = this->cta_left_alert;
      *(outbuffer + offset + 0) = (u_cta_left_alert.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cta_left_alert);
      union {
        bool real;
        uint8_t base;
      } u_cta_right_alert;
      u_cta_right_alert.real = this->cta_right_alert;
      *(outbuffer + offset + 0) = (u_cta_right_alert.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cta_right_alert);
      union {
        bool real;
        uint8_t base;
      } u_cta_left_enabled;
      u_cta_left_enabled.real = this->cta_left_enabled;
      *(outbuffer + offset + 0) = (u_cta_left_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cta_left_enabled);
      union {
        bool real;
        uint8_t base;
      } u_cta_right_enabled;
      u_cta_right_enabled.real = this->cta_right_enabled;
      *(outbuffer + offset + 0) = (u_cta_right_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->cta_right_enabled);
      union {
        bool real;
        uint8_t base;
      } u_blis_left_alert;
      u_blis_left_alert.real = this->blis_left_alert;
      *(outbuffer + offset + 0) = (u_blis_left_alert.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blis_left_alert);
      union {
        bool real;
        uint8_t base;
      } u_blis_right_alert;
      u_blis_right_alert.real = this->blis_right_alert;
      *(outbuffer + offset + 0) = (u_blis_right_alert.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blis_right_alert);
      union {
        bool real;
        uint8_t base;
      } u_blis_left_enabled;
      u_blis_left_enabled.real = this->blis_left_enabled;
      *(outbuffer + offset + 0) = (u_blis_left_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blis_left_enabled);
      union {
        bool real;
        uint8_t base;
      } u_blis_right_enabled;
      u_blis_right_enabled.real = this->blis_right_enabled;
      *(outbuffer + offset + 0) = (u_blis_right_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blis_right_enabled);
      union {
        bool real;
        uint8_t base;
      } u_sonar_enabled;
      u_sonar_enabled.real = this->sonar_enabled;
      *(outbuffer + offset + 0) = (u_sonar_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sonar_enabled);
      union {
        bool real;
        uint8_t base;
      } u_sonar_fault;
      u_sonar_fault.real = this->sonar_fault;
      *(outbuffer + offset + 0) = (u_sonar_fault.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sonar_fault);
      for( uint32_t i = 0; i < 12; i++){
      union {
        float real;
        uint32_t base;
      } u_sonari;
      u_sonari.real = this->sonar[i];
      *(outbuffer + offset + 0) = (u_sonari.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sonari.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sonari.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sonari.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sonar[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_cta_left_alert;
      u_cta_left_alert.base = 0;
      u_cta_left_alert.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cta_left_alert = u_cta_left_alert.real;
      offset += sizeof(this->cta_left_alert);
      union {
        bool real;
        uint8_t base;
      } u_cta_right_alert;
      u_cta_right_alert.base = 0;
      u_cta_right_alert.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cta_right_alert = u_cta_right_alert.real;
      offset += sizeof(this->cta_right_alert);
      union {
        bool real;
        uint8_t base;
      } u_cta_left_enabled;
      u_cta_left_enabled.base = 0;
      u_cta_left_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cta_left_enabled = u_cta_left_enabled.real;
      offset += sizeof(this->cta_left_enabled);
      union {
        bool real;
        uint8_t base;
      } u_cta_right_enabled;
      u_cta_right_enabled.base = 0;
      u_cta_right_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->cta_right_enabled = u_cta_right_enabled.real;
      offset += sizeof(this->cta_right_enabled);
      union {
        bool real;
        uint8_t base;
      } u_blis_left_alert;
      u_blis_left_alert.base = 0;
      u_blis_left_alert.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blis_left_alert = u_blis_left_alert.real;
      offset += sizeof(this->blis_left_alert);
      union {
        bool real;
        uint8_t base;
      } u_blis_right_alert;
      u_blis_right_alert.base = 0;
      u_blis_right_alert.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blis_right_alert = u_blis_right_alert.real;
      offset += sizeof(this->blis_right_alert);
      union {
        bool real;
        uint8_t base;
      } u_blis_left_enabled;
      u_blis_left_enabled.base = 0;
      u_blis_left_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blis_left_enabled = u_blis_left_enabled.real;
      offset += sizeof(this->blis_left_enabled);
      union {
        bool real;
        uint8_t base;
      } u_blis_right_enabled;
      u_blis_right_enabled.base = 0;
      u_blis_right_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blis_right_enabled = u_blis_right_enabled.real;
      offset += sizeof(this->blis_right_enabled);
      union {
        bool real;
        uint8_t base;
      } u_sonar_enabled;
      u_sonar_enabled.base = 0;
      u_sonar_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sonar_enabled = u_sonar_enabled.real;
      offset += sizeof(this->sonar_enabled);
      union {
        bool real;
        uint8_t base;
      } u_sonar_fault;
      u_sonar_fault.base = 0;
      u_sonar_fault.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sonar_fault = u_sonar_fault.real;
      offset += sizeof(this->sonar_fault);
      for( uint32_t i = 0; i < 12; i++){
      union {
        float real;
        uint32_t base;
      } u_sonari;
      u_sonari.base = 0;
      u_sonari.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sonari.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sonari.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sonari.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sonar[i] = u_sonari.real;
      offset += sizeof(this->sonar[i]);
      }
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/SurroundReport"; };
    const char * getMD5(){ return "17a8c9ed72da4f55d44d6d71483cf0e3"; };

  };

}
#endif