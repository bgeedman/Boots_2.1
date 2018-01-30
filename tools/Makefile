TARGET = angle_finder
CC = gcc
DEBUG = -g
LIBS = -lm -lgsl -lgslcblas
CLFAGS = -Wall $(DEBUG)
IFLAGS = -I$$HOME/gsl/include
LFLAGS = -L$$HOME/gsl/lib

all:
	make clean
	make angle_finder

default: all

angle_finder: angle_finder.c
	$(CC) $(CFLAGS) $(IFLAGS) $(LFLAGS) -o $(TARGET) angle_finder.c $(LIBS)

make clean:
	@echo "Cleaning up..."
	rm -f angle_finder
