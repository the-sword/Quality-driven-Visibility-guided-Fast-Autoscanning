#ifndef _ROS_moma_msgs_MobileBaseState_h
#define _ROS_moma_msgs_MobileBaseState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

namespace moma_msgs
{

  class MobileBaseState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Pose2D pose;
      geometry_msgs::Twist vel;
      int32_t emergent_code;
      int32_t error_code;

    MobileBaseState():
      header(),
      pose(),
      vel(),
      emergent_code(0),
      error_code(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->vel.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_emergent_code;
      u_emergent_code.real = this->emergent_code;
      *(outbuffer + offset + 0) = (u_emergent_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_emergent_code.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_emergent_code.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_emergent_code.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->emergent_code);
      union {
        int32_t real;
        uint32_t base;
      } u_error_code;
      u_error_code.real = this->error_code;
      *(outbuffer + offset + 0) = (u_error_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error_code.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error_code.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error_code.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error_code);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->vel.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_emergent_code;
      u_emergent_code.base = 0;
      u_emergent_code.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_emergent_code.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_emergent_code.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_emergent_code.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->emergent_code = u_emergent_code.real;
      offset += sizeof(this->emergent_code);
      union {
        int32_t real;
        uint32_t base;
      } u_error_code;
      u_error_code.base = 0;
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error_code.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->error_code = u_error_code.real;
      offset += sizeof(this->error_code);
     return offset;
    }

    const char * getType(){ return "moma_msgs/MobileBaseState"; };
    const char * getMD5(){ return "566f6c27453a881a6f0ea588040a6822"; };

  };

}
#endif