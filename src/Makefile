CFLAGS=-O2 -I../ros_lib/ -march=armv5
CCFLAGS=-std=c++11 -D_GLIBCXX_USE_NANOSLEEP -lm -pthread
DEPS=ev3dev.h
OBJ=ev3dev.o
LIBS=-lstdc++

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(CCFLAGS)

motor: $(OBJ) motor.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

ultrasonic_sensor: $(OBJ) ultrasonic_sensor.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

color_sensor: $(OBJ) color_sensor.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

diffdrive: $(OBJ) diffdrive.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

diffdrive_1: $(OBJ) diffdrive_1.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

ultrasonic_laserscanner: $(OBJ) ultrasonic_laserscanner.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

diffdrive_ultrasonic: $(OBJ) diffdrive_ultrasonic.o
	$(CC) -o ../bin/$@ $^ $(CFLAGS) $(CCFLAGS) $(LIBS)

.PHONY: all clean

clean:
	rm -f *.o *~ ../bin/diffdrive ../bin/motor ../bin/ultrasonic_sensor ../bin/color_sensor ../bin/ultrasonic_laserscanner ../bin/diffdrive_ultrasonic

all:  diffdrive motor ultrasonic_laserscanner diffdrive_ultrasonic
