#ifndef _ROS_moma_msgs_HandState_h
#define _ROS_moma_msgs_HandState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "moma_msgs/FingerState.h"

namespace moma_msgs
{

  class HandState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t fingers_length;
      moma_msgs::FingerState st_fingers;
      moma_msgs::FingerState * fingers;

    HandState():
      header(),
      fingers_length(0), fingers(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = fingers_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < fingers_length; i++){
      offset += this->fingers[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t fingers_lengthT = *(inbuffer + offset++);
      if(fingers_lengthT > fingers_length)
        this->fingers = (moma_msgs::FingerState*)realloc(this->fingers, fingers_lengthT * sizeof(moma_msgs::FingerState));
      offset += 3;
      fingers_length = fingers_lengthT;
      for( uint8_t i = 0; i < fingers_length; i++){
      offset += this->st_fingers.deserialize(inbuffer + offset);
        memcpy( &(this->fingers[i]), &(this->st_fingers), sizeof(moma_msgs::FingerState));
      }
     return offset;
    }

    const char * getType(){ return "moma_msgs/HandState"; };
    const char * getMD5(){ return "cbfcc92fb62c3ac496321131c24f3dab"; };

  };

}
#endif