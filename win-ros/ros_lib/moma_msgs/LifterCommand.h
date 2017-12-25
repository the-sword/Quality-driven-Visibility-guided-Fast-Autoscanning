#ifndef _ROS_moma_msgs_LifterCommand_h
#define _ROS_moma_msgs_LifterCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace moma_msgs
{

  class LifterCommand : public ros::Msg
  {
    public:
      int32_t control_mode;
      float position;
      int32_t move;
      enum { POSITION_CONTROL = 0  };
      enum { ACTION_CONTROL = 1  };
      enum { MOVE_UP = 1  };
      enum { STOP = 0  };
      enum { MOVE_DOWN = -1  };

    LifterCommand():
      control_mode(0),
      position(0),
      move(0)
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
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.real = this->position;
      *(outbuffer + offset + 0) = (u_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position);
      union {
        int32_t real;
        uint32_t base;
      } u_move;
      u_move.real = this->move;
      *(outbuffer + offset + 0) = (u_move.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_move.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_move.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_move.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->move);
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
      union {
        float real;
        uint32_t base;
      } u_position;
      u_position.base = 0;
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position = u_position.real;
      offset += sizeof(this->position);
      union {
        int32_t real;
        uint32_t base;
      } u_move;
      u_move.base = 0;
      u_move.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_move.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_move.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_move.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->move = u_move.real;
      offset += sizeof(this->move);
     return offset;
    }

    const char * getType(){ return "moma_msgs/LifterCommand"; };
    const char * getMD5(){ return "d1945bb57ae7e112e278bf606ba6911c"; };

  };

}
#endif