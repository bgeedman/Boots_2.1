/**
 * Copyright (c) 2017
 */

#ifndef TOOLS_H
#define TOOLS_H

#define TOOLS_VERSION "0.0.1"

#include <math.h>
#include <gsl/gsl_matrix.h>
#include <pthread.h>

#include "leg.h"

extern pthread_mutex_t log_mutex;

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

void tools_matrix_print(char *label, gsl_matrix *matrix);

void lock_logger(void *, int);

int open_serial_port(char *tty);

int write_command(char *buf);

#endif
