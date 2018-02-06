#ifndef __TOOLS_H
#define __TOOLS_H

#define __TOOLS_VERSION "0.0.1"

#include <gsl/gsl_matrix.h>
#include <math.h>
#include <pthread.h>
#include "leg.h"
#include "sequences.h"

extern pthread_mutex_t log_mutex;

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

void lock_logger(void *, int);
int open_serial_port(char *tty);
int write_command(char *buf);

#endif
