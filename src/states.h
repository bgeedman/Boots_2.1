/*
 * Defining a simple state machine for the differnt states robot can be in.
 * Each state function returns the next state that should be run and can return
 * itself to indicate a repeat.
 *
 * TODO: Need to figure out a way to do the state transitions. Maybe have
 * intermediate states that handle that and can't be interrupted until they
 * have finished running. For Example: When going from the STAND to the PARK
 * state, perhaps have a STAND_TO_PARK STATE
 *
 * STATE_TRANSITIONS:
 * ------------------
 * SETUP    -> CLEANUP, END
 * PARK     -> STAND, CLEANUP
 * STAND    -> PARK, STRETCH, TURN, WALK, CLEANUP
 * STRETCH  -> PARK, CLEANUP
 * WALK     -> PARK, CLEANUP
 * TURN     -> PARK, CLEANUP
 * CLEANUP  -> END
 * END      -> EXIT
 */


#ifndef STATES_H
#define STATES_H

#include <pthread.h>
#include "leg.h"
#include "commands.pb-c.h"

#define STATES_H_VERSION "0.0.1"


typedef struct state_args {
    Leg **legs;
    pthread_t tid;
    const char *address;
    short port;
//    Command *cmd;
}state_args;



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

extern int (*state_table[])(state_args *);

#endif
