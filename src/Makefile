OBJS = commands.pb-c.o genesis.o kinematics.o leg.o logger.o servo.o tools.o
TARGET = testing
CC = gcc
DEBUG = -g
CFLAGS = $(DEBUG) -c -Wall -DLOG_USE_COLOR
LFLAGS = $(DEBUG) -Wall
LIBS = -lm -lgsl -lgslcblas -lprotobuf-c
IDIR = -I$$HOME/gsl/include
LDIR = -L$$HOME/gsl/lib
PROTO = $$HOME/protobuf/bin/protoc
PROTO_C = /usr/bin/protoc-c

PROJECT_ROOT = $$HOME/Documents/coding_projects/Boots_2.1

default: all

protobuf: commands.proto
	@echo "======================================"
	@echo "Compiling new protobuf files..."
	$(PROTO) --python_out=$(PROJECT_ROOT)/tools/controller $^
	$(PROTO_C) --c_out=./ $^
	@echo "======================================"

tester: $(OBJS)
	$(CC) $(LFLAGS) $(IDIR) $(LDIR) $(OBJS) main.c -o $(TARGET) $(LIBS)

commands.pb-c.o: commands.pb-c.c commands.pb-c.h
	$(CC) $(CFLAGS) -o $@ commands.pb-c.c -lprotobuf-c

genesis.o: genesis.c genesis.h leg.h leg_constants.h logger.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ genesis.c $(LIBS)

kinematics.o: kinematics.c kinematics.h logger.h tools.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ kinematics.c $(LIBS)

leg.o: leg.c leg.h servo.h tools.h logger.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ leg.c $(LIBS)

logger.o: logger.c logger.h
	$(CC) $(CFLAGS) -DLOG_USE_COLOR -o $@ logger.c

servo.o: servo.c servo.h logger.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ servo.c $(LIBS)

tools.o: tools.c tools.h logger.h
	$(CC) $(CFLAGS) $(IDIR) $(LDIR) -o $@ tools.c $(LIBS)

.PHONY: clean

clean:
	@echo "======================================"
	@echo "Cleaning up object files..."
	rm *.o
	@echo "Cleaning up executables..."
	rm $(TARGET)
	@echo "Cleaning up protobuf files..."
	rm commands.pb-c.c
	rm commands.pb-c.h
	rm $(PROJECT_ROOT)/tools/controller/*.pyc
	rm $(PROJECT_ROOT)/tools/controller/commands_pb2.py
	@echo "======================================"
