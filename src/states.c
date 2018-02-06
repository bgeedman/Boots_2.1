#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include "commands.pb-c.h"
#include "leg.h"
#include "logger.h"
#include "sequences.h"
#include "server.h"
#include "states.h"
#include "updater.h"


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



static point_t (*turn_sequences[])(int, Leg *) = {
    seq_stand_to_turn_left_trot,
    seq_stand_to_turn_right_trot,
    seq_stand_to_turn_left_crawl,
    seq_stand_to_turn_right_crawl,
    seq_stand_to_turn_left_creep,
    seq_stand_to_turn_right_creep,
};

static point_t (*walk_sequences[])(int, Leg *) = {
    seq_stand_to_walk_trot,
    seq_stand_to_walk_crawl,
    seq_stand_to_walk_creep,
};



int setup_state(state_args *arg) {
    log_info("Entering setup state");
    int error;
    if ((arg->legs = malloc(4 * sizeof(Leg *))) == NULL) {
        error = errno;
        log_fatal("Failed to malloc legs: %s", strerror(errno));
        errno = error;
        return END;
    }

    log_info("Initializing legs...");
    if (leg_init(arg->legs)) {
        log_fatal("Failed to initialize legs");
        free(arg->legs);
        return END;
    }

    log_info("Setting up the server thread...");
    if (create_server_thread(arg->address, arg->port)) {
        log_fatal("Failed to setup server thread");
        return CLEANUP;
    }

    log_info("Setting up timer callback...");
    if (create_timer_callback(LEG_SPEED_SEC, arg->legs)) {
        log_fatal("Failed to create timer callback");
        return CLEANUP;
    }

    log_info("Initialization complete");
    log_info("Setting sequence: unknown_to_park");
    set_sequence(seq_unknown_to_park);
    return PARK;
}




int park_state(state_args *arg) {
    log_trace("Entering park state");
    int cmd;
    if (command == NULL) {
        log_warn("NULL command detected in Park state");
        return PARK;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    log_debug("Current cmd: %s", commands[cmd]);
    switch (cmd) {
        case COMMAND__TYPE__STAND:
            log_info("Received Stand command. Setting sequence: park_to_stand");
            set_sequence(seq_park_to_stand);
            return STAND;

        case COMMAND__TYPE__STOP:
            return PARK;

        case COMMAND__TYPE__QUIT:
            log_info("Received Stop command. Going to cleanup");
            return CLEANUP;

        default:
            return PARK;
    }
}



int stand_state(state_args *arg) {
    log_trace("Entering stand state");
    int cmd;
    int dir;
    int gait;
    if (command == NULL) {
        log_warn("NULL command detected in Stand state");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    dir = command->dir;
    gait = command->gait;
    pthread_mutex_unlock(&cmd_mutex);

    log_debug("Current cmd: %s", commands[cmd]);

    switch (cmd) {
        case COMMAND__TYPE__PARK:
            log_info("Received Park command. Setting sequence: stand_to_park");
            set_sequence(seq_stand_to_park);
            return PARK;

        case COMMAND__TYPE__STRETCH:
            log_info("Received Stretch command. Setting sequence: stand_to_stretch");
            set_sequence(seq_stand_to_stretch);
            return STRETCH;

        case COMMAND__TYPE__TURN:
            log_info("Received Turn command. Setting appropriate turn sequence");
            if (dir == COMMAND__DIRECTION__LEFT) {
                set_sequence(turn_sequences[2 * gait + 0]);
            } else if (dir == COMMAND__DIRECTION__RIGHT) {
                set_sequence(turn_sequences[2 * gait + 1]);
            } else {
                log_warn("Invalid direction detected: %d", dir);
                return STAND;
            }
            return TURN;

        case COMMAND__TYPE__WALK:
            log_info("Received Walk command. Setting appropriate walk sequence");
            if (dir == COMMAND__DIRECTION__FORWARD) {
                set_sequence(walk_sequences[gait]);
                return WALK;
            } else {
                log_warn("Invalid direction detected: %d", dir);
            }
            return STAND;

        case COMMAND__TYPE__STOP:
            return STAND;

        case COMMAND__TYPE__QUIT:
            log_info("Received Quit command. Going to cleanup");
            return CLEANUP;

        default:
            return STAND;
    }
}



int stretch_state(state_args *arg) {
    log_trace("Entering stretch state");
    int cmd;
    if (command == NULL) {
        log_warn("NULL command detected in Stretch state");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    log_debug("Current cmd; %s", commands[cmd]);

    switch (cmd) {
        case COMMAND__TYPE__STOP:
            log_info("Received Stop command. Setting sequence: stop_and_center");
            return STAND;

        case COMMAND__TYPE__QUIT:
            log_info("Received Quit command. Going to cleanup");
            return CLEANUP;

        default:
            return STRETCH;
    }
}



int walk_state(state_args *arg) {
    log_trace("Entering walk state");
    int cmd;
    if (command == NULL) {
        log_warn("NULL command detected in Walk state");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    log_debug("Current cmd: %s", commands[cmd]);

    switch (cmd) {
        case COMMAND__TYPE__STOP:
            log_info("Received Stop command. Setting sequence: stop_and_center");
            set_sequence(seq_stop_and_center);
            return STAND;

        case COMMAND__TYPE__QUIT:
            log_info("Received Quit command. Going to cleanup");
            return CLEANUP;

        default:
            return WALK;
    }
}



int turn_state(state_args *arg) {
    log_trace("Entering turn state");
    int cmd;
    if (command == NULL) {
        log_warn("NULL command detected in Turn state");
        return STAND;
    }
    pthread_mutex_lock(&cmd_mutex);
    cmd = command->cmd;
    pthread_mutex_unlock(&cmd_mutex);

    log_debug("Current cmd: %s", commands[cmd]);

    switch (cmd) {
        case COMMAND__TYPE__STOP:
            log_info("Received Stop command. Setting sequence: stop_and_center");
            set_sequence(seq_stop_and_center);
            return STAND;

        case COMMAND__TYPE__QUIT:
            log_info("Received Quit command. Going to cleanup");
            return CLEANUP;

        default:
            return TURN;
    }
}



int cleanup_state(state_args *arg) {
    log_info("Entering cleanup state");
    log_info("Cleaning up legs...");
    leg_destroy((arg->legs)[FRONT_LEFT]);
    leg_destroy((arg->legs)[FRONT_RIGHT]);
    leg_destroy((arg->legs)[BACK_LEFT]);
    leg_destroy((arg->legs)[BACK_RIGHT]);
    free(arg->legs);
    arg->legs = NULL;
    log_info("Cleanup complete!");
    return END;
}



int end_state(state_args *arg) {
    log_info("Entering end state");
    log_debug("Do we need to clean anything up?");
    return EXIT;
}
