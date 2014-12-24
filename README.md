ev3-ros
=======

Interfacing LEGO Mindstorms EV3 running [ev3dev](https://github.com/ev3dev/ev3dev) linux OS with ROS

Pre-requisites:
===============
+ ev3dev (jessie release)
+ Host computer with ROS installed
+ ROSSerial

Setup
=====
- Follow instructions from [here](http://wiki.ros.org/rosserial_embeddedlinux/GenericInstall) to setup the development environment on a host computer. This will create two folders - `ros_lib` and `examples` - in your current directory.
- Get CPP language bindings for ev3dev from [here](https://github.com/ev3dev/ev3dev-lang) and copy the files `ev3dev.h` and `ev3dev.cpp` from `cpp` folder into the `src` directory.
- Install `arm-linux-gnueabi-gcc` toolchain and use the the provided `setenv.sh` script during make
-`source ../setenv.sh; make all`
- replace `time.h` and `time.cpp` files from `ros_lib` directory with the ones provided here. 
- Correlate effort value to `duty_cycle_set_point`.
- Increase `OUTPUT_SIZE` and `INPUT_SIZE` in `ros/node_handle.h` incase you face buffer overflow issues.
- The current `ROSSerial_EmbeddedLinux` available as binary in repositories has a bug in serialization which causes all negative numbers to be converted to 0. This has been fixed in the trunk version. Hence, build `rosserial` from source code until an updated release is available.
- When constructing messages in EV3, ensure all variables are initialised. If a message contains an array, initilise it with zero if you don't want to use data in that.
- Server forking does not work for ROSSerial Python client. Therefore only one node can run on EV3