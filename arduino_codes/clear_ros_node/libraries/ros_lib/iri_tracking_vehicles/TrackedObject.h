#ifndef _ROS_iri_tracking_vehicles_TrackedObject_h
#define _ROS_iri_tracking_vehicles_TrackedObject_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "iri_tracking_vehicles/BoundingBox.h"

namespace iri_tracking_vehicles
{

  class TrackedObject : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _id_type;
      _id_type id;
      typedef float _confidence_type;
      _confidence_type confidence;
      typedef geometry_msgs::PoseWithCovariance _pose_with_covariance_type;
      _pose_with_covariance_type pose_with_covariance;
      typedef geometry_msgs::TwistWithCovariance _twist_with_covariance_type;
      _twist_with_covariance_type twist_with_covariance;
      typedef iri_tracking_vehicles::BoundingBox _box_type;
      _box_type box;

    TrackedObject():
      header(),
      id(0),
      confidence(0),
      pose_with_covariance(),
      twist_with_covariance(),
      box()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.real = this->id;
      *(outbuffer + offset + 0) = (u_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.real = this->confidence;
      *(outbuffer + offset + 0) = (u_confidence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_confidence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_confidence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_confidence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->confidence);
      offset += this->pose_with_covariance.serialize(outbuffer + offset);
      offset += this->twist_with_covariance.serialize(outbuffer + offset);
      offset += this->box.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_id;
      u_id.base = 0;
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->id = u_id.real;
      offset += sizeof(this->id);
      union {
        float real;
        uint32_t base;
      } u_confidence;
      u_confidence.base = 0;
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_confidence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->confidence = u_confidence.real;
      offset += sizeof(this->confidence);
      offset += this->pose_with_covariance.deserialize(inbuffer + offset);
      offset += this->twist_with_covariance.deserialize(inbuffer + offset);
      offset += this->box.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "iri_tracking_vehicles/TrackedObject"; };
    const char * getMD5(){ return "09e3780457ae7e7e093d3d31f03d0ed2"; };

  };

}
#endif