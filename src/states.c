#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <pthread.h>


#include "leg.h"
#include "logger.h"
#include "server.h"
#include "updater.h"
#include "states.h"
#include "sequences.h"
#include "commands.pb-c.h"


static const char *commands[] = {
    "STOP",
    "STAND",
    "WALK",
    "TURN",
    "STRETCH",
    "PARK",
    "QUIT"
};

Command *command;

int (*state_table[])(state_args *) = {
    setup_state,
    park_state,
    stand_state,
    stretch_state,
    walk_state,
    turn_state,
    cleanup_state,
    end_state
};



int setup_state(state_args *arg) {
    log_trace("Entering setup state");
    if ((arg->legs = malloc(4 * sizeof(Leg *))) == NULL) {
        return END;
    }

    log_info("Initializing legs");
    if (leg_init(arg->legs)) {
        log_fatal("Failed to initialize legs");
        return END;
    }

    log_info("Setting up the server thread...");
    if (create_server_thread(arg->address, arg->port)) {
        log_fatal("Failed to setup server thread");
        return END;
    }

    log_info("Setting up timer callback...");
    if (create_timer_callback(0.1, arg->legs)) {
        log_fatal("Failed to create timer callback");
        return END;
    }
    log_info("Setting sequence to unknown_to_park");
    set_sequence(seq_unknown_to_park);
    return PARK;
}




int park_state(state_args *arg) {
    log_trace("Entering park state");
    int cmd;
    if (command == NULL) {
        return PARK;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    log_info("Current cmd: %s", commands[cmd]);
    switch (cmd) {
        case COMMAND__TYPE__STAND:
            log_info("Setting sequence park_to_stand");
            set_sequence(seq_park_to_stand);
            return STAND;
        case COMMAND__TYPE__STOP:
            return PARK;
        case COMMAND__TYPE__QUIT:
            return CLEANUP;
        default:
            log_warn("Invalid command for current state: PARK");
            return PARK;
    }
}



int stand_state(state_args *arg) {
    log_trace("Entering stand state");
    int cmd;
    int dir;
    if (command == NULL) {
        log_warn("Something strange has happened");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    dir = command->dir;
    pthread_mutex_unlock(&cmd_mutex);

    log_info("Current cmd: %s", commands[cmd]);

    switch (cmd) {
        case COMMAND__TYPE__PARK:
            log_info("Setting sequence stand_to_park");
            set_sequence(seq_stand_to_park);
            return PARK;
        case COMMAND__TYPE__STRETCH:
            log_debug("Set sequence STAND_TO_STRETCH");
            set_sequence(seq_stand_to_stretch);
            return STRETCH;
        case COMMAND__TYPE__TURN:
            log_debug("Set sequence STAND_TO_TURN");
            if (dir == COMMAND__DIRECTION__LEFT) {
                set_sequence(seq_stand_to_turn_left);
            } else if (dir == COMMAND__DIRECTION__RIGHT) {
                set_sequence(seq_stand_to_turn_right);
            } else {
                log_warn("Invalid direction detected: %d", dir);
                return STAND;
            }
            // check the direction and set the correct turn sequence
            return TURN;
        case COMMAND__TYPE__WALK:
            log_debug("Set sequence STAND_TO_WALK");
            set_sequence(seq_stand_to_walk);
            return WALK;
        case COMMAND__TYPE__STOP:
            return STAND;
        case COMMAND__TYPE__QUIT:
            log_debug("Quitting in current state");
            return CLEANUP;
        default:
            log_warn("Invalid for current state: STAND");
            return STAND;
    }
}



int stretch_state(state_args *arg) {
    log_trace("Entering stretch state");
    int cmd;
    if (command == NULL) {
        log_warn("Something strange has happened");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    log_info("Current cmd; %s", commands[cmd]);
    switch (cmd) {
        case COMMAND__TYPE__STOP:
            log_debug("Set sequence to STOP_AND_CENTER");
            return STAND;
        case COMMAND__TYPE__QUIT:
            log_debug("Quitting in current state");
            return CLEANUP;
        default:
            log_warn("Invalid command for current state: STRETCH");
            return STRETCH;
    }
}



/*
 * Might add in different gates.  Likely start with creep gait (my favorit)
 * but could add in trot and crawl
 */
int walk_state(state_args *arg) {
    log_trace("Entering walk state");
    int cmd;
    if (command == NULL) {
        log_warn("Something strange has happened");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    log_info("Current cmd: %s", commands[cmd]);
    switch (cmd) {
        case COMMAND__TYPE__STOP:
            log_debug("Set sequence to STOP_AND_CENTER");
            set_sequence(seq_stop_and_center);
            return STAND;
        case COMMAND__TYPE__QUIT:
            log_debug("Quitting in current state");
            return CLEANUP;
        default:
            log_warn("Invalid command for current state: WALK");
            return WALK;
    }
}


/*
 * TODO: Add in different turn gates.  Trot would be the fastest, but could
 * have the more stable crawl
 */
int turn_state(state_args *arg) {
    log_trace("Entering turn state");
    int cmd;
    if (command == NULL) {
        log_warn("Something strange has happened");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    log_info("Current cmd: %s", commands[cmd]);
    switch (cmd) {
        case COMMAND__TYPE__STOP:
            log_debug("Set sequence to STOP_AND_CENTER");
            return STAND;
        case COMMAND__TYPE__QUIT:
            log_debug("Quitting in current state");
            return CLEANUP;
        default:
            log_warn("Invalid command for current state: TURN");
            return TURN;
    }
}



int cleanup_state(state_args *arg) {
    log_trace("Entering cleanup state");
    log_info("Cleaning up legs...");
    leg_destroy((arg->legs)[FRONT_LEFT]);
    leg_destroy((arg->legs)[FRONT_RIGHT]);
    leg_destroy((arg->legs)[BACK_LEFT]);
    leg_destroy((arg->legs)[BACK_RIGHT]);
    free(arg->legs);
    return END;
}


int end_state(state_args *arg) {
    log_trace("Entering end state");
    log_debug("Do we need to clean anything up?");
    return EXIT;
}
