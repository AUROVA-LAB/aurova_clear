#ifndef _ROS_dbw_mkz_msgs_Misc1Report_h
#define _ROS_dbw_mkz_msgs_Misc1Report_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dbw_mkz_msgs/TurnSignal.h"
#include "dbw_mkz_msgs/Wiper.h"
#include "dbw_mkz_msgs/AmbientLight.h"

namespace dbw_mkz_msgs
{

  class Misc1Report : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef dbw_mkz_msgs::TurnSignal _turn_signal_type;
      _turn_signal_type turn_signal;
      typedef bool _high_beam_headlights_type;
      _high_beam_headlights_type high_beam_headlights;
      typedef dbw_mkz_msgs::Wiper _wiper_type;
      _wiper_type wiper;
      typedef dbw_mkz_msgs::AmbientLight _ambient_light_type;
      _ambient_light_type ambient_light;
      typedef bool _btn_cc_on_type;
      _btn_cc_on_type btn_cc_on;
      typedef bool _btn_cc_off_type;
      _btn_cc_off_type btn_cc_off;
      typedef bool _btn_cc_on_off_type;
      _btn_cc_on_off_type btn_cc_on_off;
      typedef bool _btn_cc_res_type;
      _btn_cc_res_type btn_cc_res;
      typedef bool _btn_cc_cncl_type;
      _btn_cc_cncl_type btn_cc_cncl;
      typedef bool _btn_cc_res_cncl_type;
      _btn_cc_res_cncl_type btn_cc_res_cncl;
      typedef bool _btn_cc_set_inc_type;
      _btn_cc_set_inc_type btn_cc_set_inc;
      typedef bool _btn_cc_set_dec_type;
      _btn_cc_set_dec_type btn_cc_set_dec;
      typedef bool _btn_cc_gap_inc_type;
      _btn_cc_gap_inc_type btn_cc_gap_inc;
      typedef bool _btn_cc_gap_dec_type;
      _btn_cc_gap_dec_type btn_cc_gap_dec;
      typedef bool _btn_la_on_off_type;
      _btn_la_on_off_type btn_la_on_off;
      typedef bool _btn_ld_ok_type;
      _btn_ld_ok_type btn_ld_ok;
      typedef bool _btn_ld_up_type;
      _btn_ld_up_type btn_ld_up;
      typedef bool _btn_ld_down_type;
      _btn_ld_down_type btn_ld_down;
      typedef bool _btn_ld_left_type;
      _btn_ld_left_type btn_ld_left;
      typedef bool _btn_ld_right_type;
      _btn_ld_right_type btn_ld_right;
      typedef bool _fault_bus_type;
      _fault_bus_type fault_bus;
      typedef bool _door_driver_type;
      _door_driver_type door_driver;
      typedef bool _door_passenger_type;
      _door_passenger_type door_passenger;
      typedef bool _door_rear_left_type;
      _door_rear_left_type door_rear_left;
      typedef bool _door_rear_right_type;
      _door_rear_right_type door_rear_right;
      typedef bool _door_hood_type;
      _door_hood_type door_hood;
      typedef bool _door_trunk_type;
      _door_trunk_type door_trunk;
      typedef bool _passenger_detect_type;
      _passenger_detect_type passenger_detect;
      typedef bool _passenger_airbag_type;
      _passenger_airbag_type passenger_airbag;
      typedef bool _buckle_driver_type;
      _buckle_driver_type buckle_driver;
      typedef bool _buckle_passenger_type;
      _buckle_passenger_type buckle_passenger;

    Misc1Report():
      header(),
      turn_signal(),
      high_beam_headlights(0),
      wiper(),
      ambient_light(),
      btn_cc_on(0),
      btn_cc_off(0),
      btn_cc_on_off(0),
      btn_cc_res(0),
      btn_cc_cncl(0),
      btn_cc_res_cncl(0),
      btn_cc_set_inc(0),
      btn_cc_set_dec(0),
      btn_cc_gap_inc(0),
      btn_cc_gap_dec(0),
      btn_la_on_off(0),
      btn_ld_ok(0),
      btn_ld_up(0),
      btn_ld_down(0),
      btn_ld_left(0),
      btn_ld_right(0),
      fault_bus(0),
      door_driver(0),
      door_passenger(0),
      door_rear_left(0),
      door_rear_right(0),
      door_hood(0),
      door_trunk(0),
      passenger_detect(0),
      passenger_airbag(0),
      buckle_driver(0),
      buckle_passenger(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->turn_signal.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_high_beam_headlights;
      u_high_beam_headlights.real = this->high_beam_headlights;
      *(outbuffer + offset + 0) = (u_high_beam_headlights.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->high_beam_headlights);
      offset += this->wiper.serialize(outbuffer + offset);
      offset += this->ambient_light.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_on;
      u_btn_cc_on.real = this->btn_cc_on;
      *(outbuffer + offset + 0) = (u_btn_cc_on.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_on);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_off;
      u_btn_cc_off.real = this->btn_cc_off;
      *(outbuffer + offset + 0) = (u_btn_cc_off.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_off);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_on_off;
      u_btn_cc_on_off.real = this->btn_cc_on_off;
      *(outbuffer + offset + 0) = (u_btn_cc_on_off.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_on_off);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_res;
      u_btn_cc_res.real = this->btn_cc_res;
      *(outbuffer + offset + 0) = (u_btn_cc_res.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_res);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_cncl;
      u_btn_cc_cncl.real = this->btn_cc_cncl;
      *(outbuffer + offset + 0) = (u_btn_cc_cncl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_cncl);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_res_cncl;
      u_btn_cc_res_cncl.real = this->btn_cc_res_cncl;
      *(outbuffer + offset + 0) = (u_btn_cc_res_cncl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_res_cncl);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_set_inc;
      u_btn_cc_set_inc.real = this->btn_cc_set_inc;
      *(outbuffer + offset + 0) = (u_btn_cc_set_inc.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_set_inc);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_set_dec;
      u_btn_cc_set_dec.real = this->btn_cc_set_dec;
      *(outbuffer + offset + 0) = (u_btn_cc_set_dec.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_set_dec);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_gap_inc;
      u_btn_cc_gap_inc.real = this->btn_cc_gap_inc;
      *(outbuffer + offset + 0) = (u_btn_cc_gap_inc.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_gap_inc);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_gap_dec;
      u_btn_cc_gap_dec.real = this->btn_cc_gap_dec;
      *(outbuffer + offset + 0) = (u_btn_cc_gap_dec.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_cc_gap_dec);
      union {
        bool real;
        uint8_t base;
      } u_btn_la_on_off;
      u_btn_la_on_off.real = this->btn_la_on_off;
      *(outbuffer + offset + 0) = (u_btn_la_on_off.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_la_on_off);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_ok;
      u_btn_ld_ok.real = this->btn_ld_ok;
      *(outbuffer + offset + 0) = (u_btn_ld_ok.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_ld_ok);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_up;
      u_btn_ld_up.real = this->btn_ld_up;
      *(outbuffer + offset + 0) = (u_btn_ld_up.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_ld_up);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_down;
      u_btn_ld_down.real = this->btn_ld_down;
      *(outbuffer + offset + 0) = (u_btn_ld_down.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_ld_down);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_left;
      u_btn_ld_left.real = this->btn_ld_left;
      *(outbuffer + offset + 0) = (u_btn_ld_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_ld_left);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_right;
      u_btn_ld_right.real = this->btn_ld_right;
      *(outbuffer + offset + 0) = (u_btn_ld_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->btn_ld_right);
      union {
        bool real;
        uint8_t base;
      } u_fault_bus;
      u_fault_bus.real = this->fault_bus;
      *(outbuffer + offset + 0) = (u_fault_bus.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fault_bus);
      union {
        bool real;
        uint8_t base;
      } u_door_driver;
      u_door_driver.real = this->door_driver;
      *(outbuffer + offset + 0) = (u_door_driver.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->door_driver);
      union {
        bool real;
        uint8_t base;
      } u_door_passenger;
      u_door_passenger.real = this->door_passenger;
      *(outbuffer + offset + 0) = (u_door_passenger.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->door_passenger);
      union {
        bool real;
        uint8_t base;
      } u_door_rear_left;
      u_door_rear_left.real = this->door_rear_left;
      *(outbuffer + offset + 0) = (u_door_rear_left.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->door_rear_left);
      union {
        bool real;
        uint8_t base;
      } u_door_rear_right;
      u_door_rear_right.real = this->door_rear_right;
      *(outbuffer + offset + 0) = (u_door_rear_right.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->door_rear_right);
      union {
        bool real;
        uint8_t base;
      } u_door_hood;
      u_door_hood.real = this->door_hood;
      *(outbuffer + offset + 0) = (u_door_hood.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->door_hood);
      union {
        bool real;
        uint8_t base;
      } u_door_trunk;
      u_door_trunk.real = this->door_trunk;
      *(outbuffer + offset + 0) = (u_door_trunk.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->door_trunk);
      union {
        bool real;
        uint8_t base;
      } u_passenger_detect;
      u_passenger_detect.real = this->passenger_detect;
      *(outbuffer + offset + 0) = (u_passenger_detect.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->passenger_detect);
      union {
        bool real;
        uint8_t base;
      } u_passenger_airbag;
      u_passenger_airbag.real = this->passenger_airbag;
      *(outbuffer + offset + 0) = (u_passenger_airbag.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->passenger_airbag);
      union {
        bool real;
        uint8_t base;
      } u_buckle_driver;
      u_buckle_driver.real = this->buckle_driver;
      *(outbuffer + offset + 0) = (u_buckle_driver.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->buckle_driver);
      union {
        bool real;
        uint8_t base;
      } u_buckle_passenger;
      u_buckle_passenger.real = this->buckle_passenger;
      *(outbuffer + offset + 0) = (u_buckle_passenger.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->buckle_passenger);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->turn_signal.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_high_beam_headlights;
      u_high_beam_headlights.base = 0;
      u_high_beam_headlights.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->high_beam_headlights = u_high_beam_headlights.real;
      offset += sizeof(this->high_beam_headlights);
      offset += this->wiper.deserialize(inbuffer + offset);
      offset += this->ambient_light.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_on;
      u_btn_cc_on.base = 0;
      u_btn_cc_on.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_on = u_btn_cc_on.real;
      offset += sizeof(this->btn_cc_on);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_off;
      u_btn_cc_off.base = 0;
      u_btn_cc_off.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_off = u_btn_cc_off.real;
      offset += sizeof(this->btn_cc_off);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_on_off;
      u_btn_cc_on_off.base = 0;
      u_btn_cc_on_off.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_on_off = u_btn_cc_on_off.real;
      offset += sizeof(this->btn_cc_on_off);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_res;
      u_btn_cc_res.base = 0;
      u_btn_cc_res.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_res = u_btn_cc_res.real;
      offset += sizeof(this->btn_cc_res);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_cncl;
      u_btn_cc_cncl.base = 0;
      u_btn_cc_cncl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_cncl = u_btn_cc_cncl.real;
      offset += sizeof(this->btn_cc_cncl);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_res_cncl;
      u_btn_cc_res_cncl.base = 0;
      u_btn_cc_res_cncl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_res_cncl = u_btn_cc_res_cncl.real;
      offset += sizeof(this->btn_cc_res_cncl);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_set_inc;
      u_btn_cc_set_inc.base = 0;
      u_btn_cc_set_inc.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_set_inc = u_btn_cc_set_inc.real;
      offset += sizeof(this->btn_cc_set_inc);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_set_dec;
      u_btn_cc_set_dec.base = 0;
      u_btn_cc_set_dec.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_set_dec = u_btn_cc_set_dec.real;
      offset += sizeof(this->btn_cc_set_dec);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_gap_inc;
      u_btn_cc_gap_inc.base = 0;
      u_btn_cc_gap_inc.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_gap_inc = u_btn_cc_gap_inc.real;
      offset += sizeof(this->btn_cc_gap_inc);
      union {
        bool real;
        uint8_t base;
      } u_btn_cc_gap_dec;
      u_btn_cc_gap_dec.base = 0;
      u_btn_cc_gap_dec.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_cc_gap_dec = u_btn_cc_gap_dec.real;
      offset += sizeof(this->btn_cc_gap_dec);
      union {
        bool real;
        uint8_t base;
      } u_btn_la_on_off;
      u_btn_la_on_off.base = 0;
      u_btn_la_on_off.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_la_on_off = u_btn_la_on_off.real;
      offset += sizeof(this->btn_la_on_off);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_ok;
      u_btn_ld_ok.base = 0;
      u_btn_ld_ok.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_ld_ok = u_btn_ld_ok.real;
      offset += sizeof(this->btn_ld_ok);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_up;
      u_btn_ld_up.base = 0;
      u_btn_ld_up.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_ld_up = u_btn_ld_up.real;
      offset += sizeof(this->btn_ld_up);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_down;
      u_btn_ld_down.base = 0;
      u_btn_ld_down.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_ld_down = u_btn_ld_down.real;
      offset += sizeof(this->btn_ld_down);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_left;
      u_btn_ld_left.base = 0;
      u_btn_ld_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_ld_left = u_btn_ld_left.real;
      offset += sizeof(this->btn_ld_left);
      union {
        bool real;
        uint8_t base;
      } u_btn_ld_right;
      u_btn_ld_right.base = 0;
      u_btn_ld_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->btn_ld_right = u_btn_ld_right.real;
      offset += sizeof(this->btn_ld_right);
      union {
        bool real;
        uint8_t base;
      } u_fault_bus;
      u_fault_bus.base = 0;
      u_fault_bus.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->fault_bus = u_fault_bus.real;
      offset += sizeof(this->fault_bus);
      union {
        bool real;
        uint8_t base;
      } u_door_driver;
      u_door_driver.base = 0;
      u_door_driver.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->door_driver = u_door_driver.real;
      offset += sizeof(this->door_driver);
      union {
        bool real;
        uint8_t base;
      } u_door_passenger;
      u_door_passenger.base = 0;
      u_door_passenger.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->door_passenger = u_door_passenger.real;
      offset += sizeof(this->door_passenger);
      union {
        bool real;
        uint8_t base;
      } u_door_rear_left;
      u_door_rear_left.base = 0;
      u_door_rear_left.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->door_rear_left = u_door_rear_left.real;
      offset += sizeof(this->door_rear_left);
      union {
        bool real;
        uint8_t base;
      } u_door_rear_right;
      u_door_rear_right.base = 0;
      u_door_rear_right.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->door_rear_right = u_door_rear_right.real;
      offset += sizeof(this->door_rear_right);
      union {
        bool real;
        uint8_t base;
      } u_door_hood;
      u_door_hood.base = 0;
      u_door_hood.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->door_hood = u_door_hood.real;
      offset += sizeof(this->door_hood);
      union {
        bool real;
        uint8_t base;
      } u_door_trunk;
      u_door_trunk.base = 0;
      u_door_trunk.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->door_trunk = u_door_trunk.real;
      offset += sizeof(this->door_trunk);
      union {
        bool real;
        uint8_t base;
      } u_passenger_detect;
      u_passenger_detect.base = 0;
      u_passenger_detect.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->passenger_detect = u_passenger_detect.real;
      offset += sizeof(this->passenger_detect);
      union {
        bool real;
        uint8_t base;
      } u_passenger_airbag;
      u_passenger_airbag.base = 0;
      u_passenger_airbag.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->passenger_airbag = u_passenger_airbag.real;
      offset += sizeof(this->passenger_airbag);
      union {
        bool real;
        uint8_t base;
      } u_buckle_driver;
      u_buckle_driver.base = 0;
      u_buckle_driver.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->buckle_driver = u_buckle_driver.real;
      offset += sizeof(this->buckle_driver);
      union {
        bool real;
        uint8_t base;
      } u_buckle_passenger;
      u_buckle_passenger.base = 0;
      u_buckle_passenger.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->buckle_passenger = u_buckle_passenger.real;
      offset += sizeof(this->buckle_passenger);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/Misc1Report"; };
    const char * getMD5(){ return "c5c1e0d6ba52586919873bf9b0355143"; };

  };

}
#endif