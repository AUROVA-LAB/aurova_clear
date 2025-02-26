#ifndef _ROS_dbw_mkz_msgs_HillStartAssist_h
#define _ROS_dbw_mkz_msgs_HillStartAssist_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class HillStartAssist : public ros::Msg
  {
    public:
      typedef uint8_t _status_type;
      _status_type status;
      typedef uint8_t _mode_type;
      _mode_type mode;
      enum { STAT_INACTIVE = 0 };
      enum { STAT_FINDING_GRADIENT = 1 };
      enum { STAT_ACTIVE_PRESSED = 2 };
      enum { STAT_ACTIVE_RELEASED = 3 };
      enum { STAT_FAST_RELEASE = 4 };
      enum { STAT_SLOW_RELEASE = 5 };
      enum { STAT_FAILED = 6 };
      enum { STAT_UNDEFINED = 7 };
      enum { MODE_OFF = 0 };
      enum { MODE_AUTO = 1 };
      enum { MODE_MANUAL = 2 };
      enum { MODE_UNDEFINED = 3 };

    HillStartAssist():
      status(0),
      mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      *(outbuffer + offset + 0) = (this->mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->status);
      this->mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->mode);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/HillStartAssist"; };
    const char * getMD5(){ return "387491ab93866eae85dd46ccb3f787fc"; };

  };

}
#endif