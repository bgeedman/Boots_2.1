#ifndef UPDATER_H
#define UPDATER_H

#define UPDATER_H_VERSION "0.0.1"

#include <signal.h>
#include "leg.h"
#include "sequences.h"

void callback(int signo, siginfo_t *info, void *context);

int create_timer_callback(double sec, Leg **legs);
void *updater_thread(void *);
void set_sequence(point_t (*foo)(int, int));

int set_timer_interval(double sec);

#endif
