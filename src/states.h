#ifndef __STATES_H
#define __STATES_H

#define __STATES_H_VERSION "0.0.1"

#include <pthread.h>
#include "leg.h"

typedef struct state_args {
    Leg **legs;
    pthread_t tid;
    const char *address;
    short port;
}state_args;

extern int (*state_table[])(state_args *);

enum {
    SETUP,
    PARK,
    STAND,
    STRETCH,
    WALK,
    TURN,
    CLEANUP,
    END,
    EXIT
};

int setup_state(state_args *);
int park_state(state_args *);
int stand_state(state_args *);
int stretch_state(state_args *);
int walk_state(state_args *);
int turn_state(state_args *);
int cleanup_state(state_args *);
int end_state(state_args *);

#define ENTRY_STATE SETUP
#define EXIT_STATE EXIT


#endif
