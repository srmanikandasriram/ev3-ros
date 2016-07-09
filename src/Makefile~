CFLAGS=-O2 -I../ros_lib/ -march=armv5
CCFLAGS=-std=c++11 -D_GLIBCXX_USE_NANOSLEEP -lm -pthread
DEPS=ev3dev.h
OBJ=ev3dev.o
LIBS=-lstdc++

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)


teleop: $(OBJ) teleop.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

teleop_odom: $(OBJ) teleop_odom.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

ultrasonic_sensor: $(OBJ) ultrasonic_sensor.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)


ultrasonic_laserscanner: $(OBJ) ultrasonic_laserscanner.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

diffdrive_orientation: $(OBJ) diffdrive_orientation.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

go_to_goal: $(OBJ) go_to_goal.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

ev3_demo: $(OBJ) ev3_demo.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

go_straight_srv: $(OBJ) go_straight_srv.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

arc_service: $(OBJ) arc_service.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

master: $(OBJ) master.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)







.PHONY: all clean

clean:
	rm -f *.o *~  ../bin/ev3_demo ../bin/ultrasonic_sensor ../bin/ultrasonic_laserscanner  ../bin/diffdrive_orientation  ../bin/teleop_odom ../bin/teleop  ../bin/go_straight_srv ../bin/go_to_goal ../bin/arc_service ../bin/master

all: diffdrive_orientation ev3_demo teleop  go_straight_srv go_to_goal arc_service master ultrasonic_laserscanner ultrasonic_sensor teleop_odom
