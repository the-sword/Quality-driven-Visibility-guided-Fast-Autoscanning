#ifndef _ROS_moma_msgs_TOFState_h
#define _ROS_moma_msgs_TOFState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace moma_msgs
{

  class TOFState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float foreward;
      float backward;
      float leftside;
      float rightside;

    TOFState():
      header(),
      foreward(0),
      backward(0),
      leftside(0),
      rightside(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_foreward;
      u_foreward.real = this->foreward;
      *(outbuffer + offset + 0) = (u_foreward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_foreward.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_foreward.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_foreward.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->foreward);
      union {
        float real;
        uint32_t base;
      } u_backward;
      u_backward.real = this->backward;
      *(outbuffer + offset + 0) = (u_backward.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_backward.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_backward.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_backward.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->backward);
      union {
        float real;
        uint32_t base;
      } u_leftside;
      u_leftside.real = this->leftside;
      *(outbuffer + offset + 0) = (u_leftside.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftside.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftside.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftside.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leftside);
      union {
        float real;
        uint32_t base;
      } u_rightside;
      u_rightside.real = this->rightside;
      *(outbuffer + offset + 0) = (u_rightside.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightside.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightside.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightside.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rightside);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_foreward;
      u_foreward.base = 0;
      u_foreward.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_foreward.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_foreward.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_foreward.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->foreward = u_foreward.real;
      offset += sizeof(this->foreward);
      union {
        float real;
        uint32_t base;
      } u_backward;
      u_backward.base = 0;
      u_backward.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_backward.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_backward.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_backward.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->backward = u_backward.real;
      offset += sizeof(this->backward);
      union {
        float real;
        uint32_t base;
      } u_leftside;
      u_leftside.base = 0;
      u_leftside.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftside.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftside.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftside.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->leftside = u_leftside.real;
      offset += sizeof(this->leftside);
      union {
        float real;
        uint32_t base;
      } u_rightside;
      u_rightside.base = 0;
      u_rightside.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightside.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightside.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightside.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rightside = u_rightside.real;
      offset += sizeof(this->rightside);
     return offset;
    }

    const char * getType(){ return "moma_msgs/TOFState"; };
    const char * getMD5(){ return "8d68091cf6b0e5f45c2c9f5158ea1b65"; };

  };

}
#endif