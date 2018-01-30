OBJS = commands.pb-c.o kinematics.o leg.o logger.o servo.o tools.o server.o states.o updater.o sequences.o
TARGET = robot
CC = gcc
DEBUG = -g
CFLAGS = $(DEBUG) -c -Wall
LFLAGS = $(DEBUG) -Wall
LIBS = -lm -lgsl -lgslcblas -lprotobuf-c -pthread -lrt
IDIR = -I$$HOME/gsl/include
LDIR = -L$$HOME/gsl/lib
PROTO = $$HOME/protobuf/bin/protoc
PROTO_C = /usr/bin/protoc-c


all:
	make clean
	make protobuf
	make testing

default: all

testing: $(OBJS)
	$(CC) $(LFLAGS) $(IDIR) $(LDIR) $(OBJS) testing.c -o testing $(LIBS)

protobuf: commands.proto
	@echo "======================================"
	@echo "Compiling new protobuf files..."
	$(PROTO) --python_out=./controller $^
	$(PROTO_C) --c_out=./ $^
	@echo "======================================"

robot: $(OBJS)
	$(CC) $(LFLAGS) $(IDIR) $(LDIR) $(OBJS) main.c -o $(TARGET) $(LIBS)

commands.pb-c.o: protobuf commands.pb-c.c commands.pb-c.h
	$(CC) $(CFLAGS) -o $@ commands.pb-c.c -lprotobuf-c

kinematics.o: kinematics.c kinematics.h logger.h tools.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ kinematics.c $(LIBS)

leg.o: leg.c leg.h leg_constants.h servo.h tools.h logger.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ leg.c $(LIBS)

logger.o: logger.c logger.h
	$(CC) $(CFLAGS) -DLOG_USE_COLOR -o $@ logger.c

servo.o: servo.c servo.h logger.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ servo.c $(LIBS)

tools.o: tools.c tools.h logger.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ tools.c $(LIBS)

server.o: server.c server.h commands.pb-c.h logger.h
	$(CC) $(CFLAGS) -o $@ server.c $(LIBS)

states.o: states.c states.h leg.h logger.h server.h updater.h sequences.h commands.pb-c.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ states.c $(LIBS)

updater.o: updater.c updater.h logger.h sequences.h leg.h kinematics.h commands.pb-c.h tools.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ updater.c $(LIBS)

sequences.o: sequences.c sequences.h leg.h logger.h commands.pb-c.h server.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ sequences.c $(LIBS)

.PHONY: clean

clean:
	@echo "======================================"
	@echo "Cleaning up object files..."
	-rm -f *.o
	@echo "Cleaning up executables..."
	-rm $(TARGET)
	rm -f testing
	@echo "Cleaning up protobuf files..."
	-rm -f commands.pb-c.c
	-rm -f commands.pb-c.h
	-rm -f ./controller/*.pyc
	-rm -f ./controller/commands_pb2.py
	@echo "======================================"
