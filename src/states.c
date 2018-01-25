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
    if (create_server_thread(arg->address, arg->port, &(arg->cmd))) {
        log_fatal("Failed to setup server thread");
        return END;
    }

    log_info("Setting up timer callback...");
    //if (create_timer_callback(0.5, arg->legs)) {
    if (create_timer_callback(1.0, arg->legs)) {
        log_fatal("Failed to create timer callback");
        return END;
    }

    set_sequence(seq_unknown_to_park);
    return PARK;
}




int park_state(state_args *arg) {
    log_trace("Entering park state");

    int command;
    if (arg->cmd == NULL) {
        return PARK;
    }
    pthread_mutex_lock(&cmd_mutex);
    command = arg->cmd->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    log_info("Current State: %d", command);
    switch (command) {
        case COMMAND__TYPE__STAND:
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
    int command;
    if (arg->cmd == NULL) {
        log_warn("Something strange has happened....cmd is null");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    command = arg->cmd->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    switch (command) {
        case COMMAND__TYPE__PARK:
            log_debug("Set sequence STAND_TO_PARK");
            return PARK;
        case COMMAND__TYPE__STRETCH:
            log_debug("Set sequence STAND_TO_STRETCH");
            return STRETCH;
        case COMMAND__TYPE__TURN:
            log_debug("Set sequence STAND_TO_TURN");
            return TURN;
        case COMMAND__TYPE__WALK:
            log_debug("Set sequence STAND_TO_WALK");
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
    int command;
    if (arg->cmd == NULL) {
        log_warn("Something strange has happened....cmd is null");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    command = arg->cmd->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    switch (command){
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



int walk_state(state_args *arg) {
    log_trace("Entering walk state");
    int command;
    if (arg->cmd == NULL) {
        log_warn("Something strange has happened....cmd is null");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    command = arg->cmd->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    switch (command) {
        case COMMAND__TYPE__STOP:
            log_debug("Set sequence to STOP_AND_CENTER");
            return STAND;
        case COMMAND__TYPE__QUIT:
            log_debug("Quitting in current state");
            return CLEANUP;
        default:
            log_warn("Invalid command for current state: WALK");
            return WALK;
    }
}



int turn_state(state_args *arg) {
    log_trace("Entering turn state");
    int command;
    if (arg->cmd == NULL) {
        log_warn("Something strange has happened....cmd is null");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    command = arg->cmd->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    switch (command) {
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
