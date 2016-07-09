#ifndef _ROS_SERVICE_arc_h
#define _ROS_SERVICE_arc_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace headers
{

static const char ARC[] = "headers/arc";

  class arcRequest : public ros::Msg
  {
    public:
      double rg;
      double arcl;

    arcRequest():
      rg(0),
      arcl(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_rg;
      u_rg.real = this->rg;
      *(outbuffer + offset + 0) = (u_rg.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rg.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rg.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rg.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_rg.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_rg.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_rg.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_rg.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->rg);
      union {
        double real;
        uint64_t base;
      } u_arcl;
      u_arcl.real = this->arcl;
      *(outbuffer + offset + 0) = (u_arcl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arcl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_arcl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_arcl.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_arcl.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_arcl.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_arcl.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_arcl.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->arcl);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_rg;
      u_rg.base = 0;
      u_rg.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rg.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rg.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rg.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_rg.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_rg.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_rg.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_rg.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->rg = u_rg.real;
      offset += sizeof(this->rg);
      union {
        double real;
        uint64_t base;
      } u_arcl;
      u_arcl.base = 0;
      u_arcl.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arcl.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_arcl.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_arcl.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_arcl.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_arcl.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_arcl.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_arcl.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->arcl = u_arcl.real;
      offset += sizeof(this->arcl);
     return offset;
    }

    const char * getType(){ return ARC; };
    const char * getMD5(){ return "5e33bda32ab3d20bdfe1aadf5e3578e1"; };

  };

  class arcResponse : public ros::Msg
  {
    public:
      double x;
      double y;
      double tc;

    arcResponse():
      x(0),
      y(0),
      tc(0)
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
      } u_tc;
      u_tc.real = this->tc;
      *(outbuffer + offset + 0) = (u_tc.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tc.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tc.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tc.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tc.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tc.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tc.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tc.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tc);
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
      } u_tc;
      u_tc.base = 0;
      u_tc.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tc.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tc.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tc.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_tc.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_tc.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_tc.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_tc.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->tc = u_tc.real;
      offset += sizeof(this->tc);
     return offset;
    }

    const char * getType(){ return ARC; };
    const char * getMD5(){ return "84e4f6eadb78eb0d756b2a3e47fcc7ac"; };

  };

  class arc {
    public:
    typedef arcRequest Request;
    typedef arcResponse Response;
  };

}
#endif
