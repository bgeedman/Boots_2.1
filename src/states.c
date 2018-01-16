#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <pthread.h>


#include "leg.h"
#include "logger.h"
#include "server.h"
#include "setup.h"
#include "states.h"
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
    serv_args_t *serv_a; // this is a memory leak
    if ((arg->legs = malloc(4 * sizeof(Leg *))) == NULL) {
        return END;
    }

    if ((serv_a = malloc(sizeof(serv_args_t))) == NULL) {
        log_fatal("failed to create space for the server args");
        return END;
    }

    int str_len = strlen(arg->address) + 1;
    serv_a->address = malloc(str_len);
    if (serv_a->address == NULL) {
        log_fatal("Failed to malloc space in server args");
        free(serv_a);
        return END;
    }

    strncpy(serv_a->address, arg->address, str_len);
    serv_a->port = arg->port;
    serv_a->cmd = &(arg->cmd);



    log_info("Initializing legs");
    if (setup_leg_init(arg->legs)) {
        log_fatal("Failed to initialize legs");
        free(serv_a->address);
        free(serv_a);
        return END;
    }


    log_debug("Initializing server");
    if (pthread_create(&(arg->tid), NULL, server_thread, serv_a)) {
        log_fatal("Failed to initialize server");
        free(serv_a->address);
        free(serv_a);
        return END;
    }


    // setup the timer callback
    log_debug("Set sequence to UNKNOWN_TO_PARK");
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
            log_debug("Set sequence PARK_TO_STAND");
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
    if (pthread_join(arg->tid, NULL)) {
        log_error("Failed to join thread");
    }
    return EXIT;
}
