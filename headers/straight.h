#ifndef _ROS_SERVICE_straight_h
#define _ROS_SERVICE_straight_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace headers
{

static const char STRAIGHT[] = "headers/straight";

  class straightRequest : public ros::Msg
  {
    public:
      double vel;
      int64_t time;

    straightRequest():
      vel(0),
      time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_vel;
      u_vel.real = this->vel;
      *(outbuffer + offset + 0) = (u_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vel.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vel.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vel.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vel.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vel.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vel);
      union {
        int64_t real;
        uint64_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_vel;
      u_vel.base = 0;
      u_vel.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vel.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vel.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vel.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vel.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vel.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vel.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vel.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vel = u_vel.real;
      offset += sizeof(this->vel);
      union {
        int64_t real;
        uint64_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time = u_time.real;
      offset += sizeof(this->time);
     return offset;
    }

    const char * getType(){ return STRAIGHT; };
    const char * getMD5(){ return "3953e27aa5d5f17546b25df963292f19"; };

  };

  class straightResponse : public ros::Msg
  {
    public:
      double vl;
      double vr;

    straightResponse():
      vl(0),
      vr(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_vl;
      u_vl.real = this->vl;
      *(outbuffer + offset + 0) = (u_vl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vl.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vl.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vl.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vl.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vl.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vl);
      union {
        double real;
        uint64_t base;
      } u_vr;
      u_vr.real = this->vr;
      *(outbuffer + offset + 0) = (u_vr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vr.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_vr.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_vr.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_vr.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_vr.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->vr);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_vl;
      u_vl.base = 0;
      u_vl.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vl.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vl.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vl.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vl.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vl.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vl.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vl.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vl = u_vl.real;
      offset += sizeof(this->vl);
      union {
        double real;
        uint64_t base;
      } u_vr;
      u_vr.base = 0;
      u_vr.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vr.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vr.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vr.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_vr.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_vr.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_vr.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_vr.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->vr = u_vr.real;
      offset += sizeof(this->vr);
     return offset;
    }

    const char * getType(){ return STRAIGHT; };
    const char * getMD5(){ return "6f0db14fcc6988863b4cb4bcc65efc6f"; };

  };

  class straight {
    public:
    typedef straightRequest Request;
    typedef straightResponse Response;
  };

}
#endif
