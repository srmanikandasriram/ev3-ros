ev3-ros
=======

Interfacing LEGO Mindstorms EV3 running [ev3dev](https://github.com/ev3dev/ev3dev) linux OS with ROS  

```
Note : This repositry is compatible with the ev3image file ev3dev-jessie-2015-12-30

```


Pre-requisites:
===============
+ Host computer with ROS installed
+ A lego mindstorm ev3 unit with a Wi-Fi dongle

Setup
=====
+ Download the image file from this repository or from [here](https://github.com/ev3dev/ev3dev/releases).

+ Follow the instructions [here](http://www.ev3dev.org/docs/getting-started/) to copy the image file onto a SD card .

+ Follow the instructions to setup the ssh connection between your workstation and the ev3 brick :

	(i) In order to make the connection with a Wi-Fi dongle one has to first follow the instructions to get connected through USB [here]( 
		http://www.ev3dev.org/docs/tutorials/connecting-to-the-internet-via-usb/).

	(ii) Now once the connection is made through the USB, go [here](http://www.ev3dev.org/docs/tutorials/setting-up-wifi-using-the-command-line/) to learn how to get connected through a Wi-Fi dongle.

```
Note: Connecting the first time through dongle will only be cumbersome, this step is completed,connection will be made automatically upon booting the ev3 and our job is to only type the ssh command. 
```

+ Now, follow instructions from [here](http://wiki.ros.org/rosserial_embeddedlinux/GenericInstall) 
   to setup the developement environment on the host computer.This will download two folders- `examples` and `ros_lib` in your working directory.`roslib` contains all the libraries required by the pre- existing ROS packages
   to run. 

+ Get CPP language bindings for ev3dev from [here](https://github.com/ev3dev/ev3dev-lang)
   and copy the files `ev3dev.h` and `ev3dev.cpp` from the repository into the `src` directory.

+ Store this repository along with the two directories downloaded earlier in a one new folder and name it `ev3-ros` in your catkin workspace's src directory.

+ Replace `time.h` and `time.cpp` files from `ros_lib` directory with the ones provided here.

+ Install gcc-arm-linux-gnueabi, g++-arm-linux-gnueabi, toolchains(these are the compilers that we use in 
   cross compilation to obtain an executable that will run on the ev3).

+ Now, run

  `$ cd ~/<your catkin workspace>/src/<ev3-ros>/src`

  `$ source ../setenv.sh ; make all`

```
Note: " make all " compiles all the programs in the src file.
If suppose you want to compile only a specific program, 
run $ make <name of that program> 

These commands will compile the programs written in the src folder and would create an executable for 
the program that can
run in ev3 in  ~/<your catkin workspace>/src/ev3-ros>/bin 
folder   
```

+ Now run the ssh command to connect the host computer and the ev3 brick.

+ Now, to send the executable from the computer to the ev3, in your laptop's terminal run,

   ` $ cd ~/<your catkin workspace>/src/ev3-ros/bin`

   ` $ scp <executable name> robot@ev3dev.local:/<address of the directory in ev3 to store>`

+ Once the executable is copied onto the ev3 go [here](http://wiki.ros.org/rosserial_embeddedlinux/Tutorials) to   
    understand rosserial in embeddedlinux.

+ Now in the ROS workstation run, 
	`$ roscore `

	In another terminal, 
	`$ rosrun rosserial_python serial_node.py tcp`

+ In the ev3 ssh terminal, 

	`$ cd <wherever you stored the executable file in step 10>`

	`$ ./<name of executable>`

+ Increase `OUTPUT_SIZE` and `INPUT_SIZE` in ros/node_handle.h incase you face buffer overflow issue. The current  
    ROSSerial_EmbeddedLinux available as binary in repositories has a bug in serialization which causes all negative numbers to be converted to 0. This has been fixed in the trunk version. Hence, build rosserial from source code until an updated release is available.
 
+ When constructing messages in EV3, ensure all variables are initialised. If a message contains an array,  
    initialise it with zero if you don't want to use data in that.  

