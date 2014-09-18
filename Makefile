OBJS = serialport.o vel_optimal.o main.o process.o global.o
SRCS = main.cpp serialport.cpp vel_optimal.cpp process.cpp global.cpp
CFLAGS = -Wall -O -g -lstdc++
CC = g++

main: $(OBJS) $(SRCS)
	$(CC) $(OBJS) -o main -lpthread

serialport.o: serialport.h serialport.cpp
	$(CC) $(CFLAGS) -c serialport.cpp -o serialport.o

vel_optimal.o: vel_optimal.h vel_optimal.cpp
	$(CC) $(CFLAGS) -c vel_optimal.cpp -o vel_optimal.o

process.o:	global.o process.h  process.cpp
	$(CC) $(CFLAGS) -c process.cpp -o process.o

global.o:	global.h  global.cpp
	$(CC) $(CFLAGS) -c global.cpp -o global.o

clean:
	rm -rf *.o main
