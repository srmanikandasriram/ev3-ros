ev3-ros
=======

Interfacing LEGO Mindstorms EV3 running [ev3dev](https://github.com/ev3dev/ev3dev) linux OS with ROS

Setup
=====
- Follow instructions from [here](http://wiki.ros.org/rosserial_embeddedlinux/GenericInstall) to setup the development environment on a host computer. This will create two folders - `ros_lib` and `examples` - in your current directory.
- Get CPP language bindings for ev3dev from [here](https://github.com/ev3dev/ev3dev-lang) and copy the files `ev3dev.h` and `ev3dev.cpp` from `cpp` folder into this directory
- Install `arm-linux-gnueabi-gcc` toolchain and use the the provided `setenv.sh` script during make
-`source ../setenv.sh; make all`
- Check implementation of `time.h` and `time.cpp`
- Correlate effort value to duty_cycle_set_point