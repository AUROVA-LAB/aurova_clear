#ifndef _ROS_dbw_mkz_msgs_WatchdogCounter_h
#define _ROS_dbw_mkz_msgs_WatchdogCounter_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dbw_mkz_msgs
{

  class WatchdogCounter : public ros::Msg
  {
    public:
      typedef uint8_t _source_type;
      _source_type source;
      enum { NONE = 0                };
      enum { OTHER_BRAKE = 1         };
      enum { OTHER_THROTTLE = 2      };
      enum { OTHER_STEERING = 3      };
      enum { BRAKE_COUNTER = 4       };
      enum { BRAKE_DISABLED = 5      };
      enum { BRAKE_COMMAND = 6       };
      enum { BRAKE_REPORT = 7        };
      enum { THROTTLE_COUNTER = 8    };
      enum { THROTTLE_DISABLED = 9   };
      enum { THROTTLE_COMMAND = 10   };
      enum { THROTTLE_REPORT = 11    };
      enum { STEERING_COUNTER = 12   };
      enum { STEERING_DISABLED = 13  };
      enum { STEERING_COMMAND = 14   };
      enum { STEERING_REPORT = 15    };

    WatchdogCounter():
      source(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->source >> (8 * 0)) & 0xFF;
      offset += sizeof(this->source);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->source =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->source);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/WatchdogCounter"; };
    const char * getMD5(){ return "49446aafd000e473e340dfb1d38eeb50"; };

  };

}
#endif