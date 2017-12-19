TARGET = testing
TESTER_TARGET = matrix_tester
LIBS = -lm -lgsl -lgslcblas
CC = gcc
CFLAGS = -g -Wall -DLOG_USE_COLOR
INC = -I/home/sauron/gsl/include
LIB = -L/home/sauron/gsl/lib


default: all

tester:
	$(CC) $(CFLAGS) $(INC) $(LIB) tools.c leg.c servo.c logger.c genesis.c kinematics.c main.c $(LIBS) -o $(TARGET)

matrix_tester:
	$(CC) $(CFLAGS) $(INC) $(LIB) matrix_tester.c -o $(TESTER_TARGET) $(LIBS)

all: tester

clean:
	rm $(TARGET)
	rm $(TESTER_TARGET)
