#ifndef _ROS_moma_msgs_FingerState_h
#define _ROS_moma_msgs_FingerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "moma_msgs/PhalanxState.h"

namespace moma_msgs
{

  class FingerState : public ros::Msg
  {
    public:
      uint8_t phalanxes_length;
      moma_msgs::PhalanxState st_phalanxes;
      moma_msgs::PhalanxState * phalanxes;

    FingerState():
      phalanxes_length(0), phalanxes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = phalanxes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < phalanxes_length; i++){
      offset += this->phalanxes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t phalanxes_lengthT = *(inbuffer + offset++);
      if(phalanxes_lengthT > phalanxes_length)
        this->phalanxes = (moma_msgs::PhalanxState*)realloc(this->phalanxes, phalanxes_lengthT * sizeof(moma_msgs::PhalanxState));
      offset += 3;
      phalanxes_length = phalanxes_lengthT;
      for( uint8_t i = 0; i < phalanxes_length; i++){
      offset += this->st_phalanxes.deserialize(inbuffer + offset);
        memcpy( &(this->phalanxes[i]), &(this->st_phalanxes), sizeof(moma_msgs::PhalanxState));
      }
     return offset;
    }

    const char * getType(){ return "moma_msgs/FingerState"; };
    const char * getMD5(){ return "0379a63b34b8d69f073923a5ae1c0686"; };

  };

}
#endif