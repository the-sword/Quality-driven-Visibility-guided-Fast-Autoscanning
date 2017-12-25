#ifndef _ROS_moma_msgs_HandCommand_h
#define _ROS_moma_msgs_HandCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moma_msgs/FingerCommand.h"

namespace moma_msgs
{

  class HandCommand : public ros::Msg
  {
    public:
      int32_t control_mode;
      uint8_t finger_cmd_length;
      moma_msgs::FingerCommand st_finger_cmd;
      moma_msgs::FingerCommand * finger_cmd;
      enum { VELOCITY_CONTROL = 0 };
      enum { POSITION_CONTROL = 1 };

    HandCommand():
      control_mode(0),
      finger_cmd_length(0), finger_cmd(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_control_mode;
      u_control_mode.real = this->control_mode;
      *(outbuffer + offset + 0) = (u_control_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_control_mode.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_control_mode.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_control_mode.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->control_mode);
      *(outbuffer + offset++) = finger_cmd_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < finger_cmd_length; i++){
      offset += this->finger_cmd[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_control_mode;
      u_control_mode.base = 0;
      u_control_mode.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_control_mode.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_control_mode.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_control_mode.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->control_mode = u_control_mode.real;
      offset += sizeof(this->control_mode);
      uint8_t finger_cmd_lengthT = *(inbuffer + offset++);
      if(finger_cmd_lengthT > finger_cmd_length)
        this->finger_cmd = (moma_msgs::FingerCommand*)realloc(this->finger_cmd, finger_cmd_lengthT * sizeof(moma_msgs::FingerCommand));
      offset += 3;
      finger_cmd_length = finger_cmd_lengthT;
      for( uint8_t i = 0; i < finger_cmd_length; i++){
      offset += this->st_finger_cmd.deserialize(inbuffer + offset);
        memcpy( &(this->finger_cmd[i]), &(this->st_finger_cmd), sizeof(moma_msgs::FingerCommand));
      }
     return offset;
    }

    const char * getType(){ return "moma_msgs/HandCommand"; };
    const char * getMD5(){ return "a65f44527acc51bc83ef03fb68d9b46d"; };

  };

}
#endif