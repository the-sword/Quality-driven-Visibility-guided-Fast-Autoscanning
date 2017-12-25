#ifndef _ROS_moma_msgs_ProtobufData_h
#define _ROS_moma_msgs_ProtobufData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace moma_msgs
{

  class ProtobufData : public ros::Msg
  {
    public:
      int16_t data_type;
      uint8_t pb_data_length;
      uint8_t st_pb_data;
      uint8_t * pb_data;

    ProtobufData():
      data_type(0),
      pb_data_length(0), pb_data(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_data_type;
      u_data_type.real = this->data_type;
      *(outbuffer + offset + 0) = (u_data_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data_type.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data_type);
      *(outbuffer + offset++) = pb_data_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < pb_data_length; i++){
      *(outbuffer + offset + 0) = (this->pb_data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pb_data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_data_type;
      u_data_type.base = 0;
      u_data_type.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data_type.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data_type = u_data_type.real;
      offset += sizeof(this->data_type);
      uint8_t pb_data_lengthT = *(inbuffer + offset++);
      if(pb_data_lengthT > pb_data_length)
        this->pb_data = (uint8_t*)realloc(this->pb_data, pb_data_lengthT * sizeof(uint8_t));
      offset += 3;
      pb_data_length = pb_data_lengthT;
      for( uint8_t i = 0; i < pb_data_length; i++){
      this->st_pb_data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->st_pb_data);
        memcpy( &(this->pb_data[i]), &(this->st_pb_data), sizeof(uint8_t));
      }
     return offset;
    }

    const char * getType(){ return "moma_msgs/ProtobufData"; };
    const char * getMD5(){ return "e668193d0608eb162a3d2e248cd2fc6a"; };

  };

}
#endif