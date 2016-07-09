#ifndef _ROS_SERVICE_gtg_h
#define _ROS_SERVICE_gtg_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace headers
{

static const char GTG[] = "headers/gtg";

  class gtgRequest : public ros::Msg
  {
    public:
      double xg;
      double yg;

    gtgRequest():
      xg(0),
      yg(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_xg;
      u_xg.real = this->xg;
      *(outbuffer + offset + 0) = (u_xg.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xg.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xg.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xg.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_xg.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_xg.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_xg.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_xg.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->xg);
      union {
        double real;
        uint64_t base;
      } u_yg;
      u_yg.real = this->yg;
      *(outbuffer + offset + 0) = (u_yg.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yg.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yg.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yg.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_yg.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_yg.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_yg.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_yg.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->yg);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_xg;
      u_xg.base = 0;
      u_xg.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xg.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xg.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xg.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_xg.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_xg.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_xg.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_xg.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->xg = u_xg.real;
      offset += sizeof(this->xg);
      union {
        double real;
        uint64_t base;
      } u_yg;
      u_yg.base = 0;
      u_yg.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yg.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yg.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yg.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_yg.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_yg.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_yg.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_yg.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->yg = u_yg.real;
      offset += sizeof(this->yg);
     return offset;
    }

    const char * getType(){ return GTG; };
    const char * getMD5(){ return "edcb9ea6e72fee5ba33c9704396294ec"; };

  };

  class gtgResponse : public ros::Msg
  {
    public:
      double x;
      double y;
      double t;

    gtgResponse():
      x(0),
      y(0),
      t(0)
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
      } u_t;
      u_t.real = this->t;
      *(outbuffer + offset + 0) = (u_t.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_t.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_t.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_t.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_t.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_t.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_t.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_t.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->t);
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
      } u_t;
      u_t.base = 0;
      u_t.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_t.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_t.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_t.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_t.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_t.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_t.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_t.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->t = u_t.real;
      offset += sizeof(this->t);
     return offset;
    }

    const char * getType(){ return GTG; };
    const char * getMD5(){ return "4a28bd10bf2ad79a2c715f25c5a7ebd3"; };

  };

  class gtg {
    public:
    typedef gtgRequest Request;
    typedef gtgResponse Response;
  };

}
#endif
