#ifndef _ROS_dbw_mkz_msgs_TurnSignalCmd_h
#define _ROS_dbw_mkz_msgs_TurnSignalCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dbw_mkz_msgs/TurnSignal.h"

namespace dbw_mkz_msgs
{

  class TurnSignalCmd : public ros::Msg
  {
    public:
      typedef dbw_mkz_msgs::TurnSignal _cmd_type;
      _cmd_type cmd;

    TurnSignalCmd():
      cmd()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->cmd.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->cmd.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "dbw_mkz_msgs/TurnSignalCmd"; };
    const char * getMD5(){ return "f1310dcd252c98fc408c6df907b9495a"; };

  };

}
#endif