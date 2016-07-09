#ifndef _ROS_SERVICE_orientation_h
#define _ROS_SERVICE_orientation_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace headers
{

static const char ORIENTATION[] = "headers/orientation";

  class orientationRequest : public ros::Msg
  {
    public:
      double vecx;
      double vecy;

    orientationRequest():
      vecx(0),
      vecy(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_vecx;
      u_vecx.real = this->vecx;
      *(outbuffer + offset + 0) = (u_vecx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vecx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vecx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vecx.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vecx.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vecx.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vecx.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vecx.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vecx);
      union {
        double real;
        uint64_t base;
      } u_vecy;
      u_vecy.real = this->vecy;
      *(outbuffer + offset + 0) = (u_vecy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vecy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vecy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vecy.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vecy.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vecy.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vecy.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vecy.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vecy);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_vecx;
      u_vecx.base = 0;
      u_vecx.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vecx.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vecx.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vecx.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vecx.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vecx.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vecx.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vecx.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vecx = u_vecx.real;
      offset += sizeof(this->vecx);
      union {
        double real;
        uint64_t base;
      } u_vecy;
      u_vecy.base = 0;
      u_vecy.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vecy.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vecy.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vecy.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vecy.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vecy.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vecy.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vecy.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vecy = u_vecy.real;
      offset += sizeof(this->vecy);
     return offset;
    }

    const char * getType(){ return ORIENTATION; };
    const char * getMD5(){ return "4d1a6778ffbf99c9927795c92ed98765"; };

  };

  class orientationResponse : public ros::Msg
  {
    public:
      double x;
      double y;
      double tf;

    orientationResponse():
      x(0),
      y(0),
      tf(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_x.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_x.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_x.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_x.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_y.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_y.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_y.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_y.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_tf;
      u_tf.real = this->tf;
      *(outbuffer + offset + 0) = (u_tf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tf.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tf.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tf.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tf.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tf.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tf);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_x.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        double real;
        uint64_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_y.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        double real;
        uint64_t base;
      } u_tf;
      u_tf.base = 0;
      u_tf.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tf.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tf.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tf.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_tf.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_tf.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_tf.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_tf.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->tf = u_tf.real;
      offset += sizeof(this->tf);
     return offset;
    }

    const char * getType(){ return ORIENTATION; };
    const char * getMD5(){ return "f98cfad3deaa076127287637d3d88d34"; };

  };

  class orientation {
    public:
    typedef orientationRequest Request;
    typedef orientationResponse Response;
  };

}
#endif
