#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <semaphore.h>

#include "updater.h"
#include "leg.h"
#include "kinematics.h"
#include "sequences.h"
#include "logger.h"
#include "commands.pb-c.h"




#define BILLION 1000000000L

static point_t (*sequence_function)(int, int, Command *);
static int frame = 0;
static sem_t updatesem;
static timer_t timerid;


static int setinterrupt() {
    struct sigaction act;
    act.sa_flags = SA_SIGINFO;
    act.sa_sigaction = callback;
    if ((sigemptyset(&act.sa_mask) == -1) ||
        (sigaction(SIGALRM, &act, NULL) == -1)) {
        return -1;
    }
    return 0;
}



int set_timer_interval(double sec) {
    struct itimerspec value;

    value.it_interval.tv_sec = (long)sec;
    value.it_interval.tv_nsec = (sec - value.it_interval.tv_sec) * BILLION;
    if (value.it_interval.tv_nsec > BILLION) {
        value.it_interval.tv_sec++;
        value.it_interval.tv_nsec -= BILLION;
    }
    value.it_value = value.it_interval;
    return timer_settime(timerid, 0, &value, NULL);
}

// timer callback function
void callback(int signo, siginfo_t *info, void *context) {
    if (sem_post(&updatesem)) {
        log_warn("failed to post to semaphore");
    }
}


void *updater_thread(void *data) {
    struct update_thread_args *args = (struct update_thread_args *)data;
    Leg **legs = args->legs;
    Command *cmd = args->cmd;

    log_trace("Update thread");
    char buf[1024];

    while (sem_wait(&updatesem) == 0) {
        log_debug("get next leg postiion");
        // call sequence function with the frame number

        log_debug("updating leg positions for frame: %d", frame);
        point_t epoint = sequence_function(frame, FRONT_LEFT, cmd);

        log_debug("FRONT_LEFT: (%d, %d, %d)", epoint.x, epoint.y, epoint.z);
        leg_set_end_point(legs[FRONT_LEFT], epoint.x, epoint.y, epoint.z);

        epoint = sequence_function(frame, FRONT_RIGHT, cmd);
        log_debug("FRONT_RIGHT: (%d, %d, %d)", epoint.x, epoint.y, epoint.z);
        leg_set_end_point(legs[FRONT_RIGHT], epoint.x, epoint.y, epoint.z);

        epoint = sequence_function(frame, BACK_LEFT, cmd);
        log_debug("BACK_LEFT: (%d, %d, %d)", epoint.x, epoint.y, epoint.z);
        leg_set_end_point(legs[BACK_LEFT], epoint.x, epoint.y, epoint.z);

        epoint = sequence_function(frame, BACK_RIGHT, cmd);
        log_debug("BACK_RIGHT: (%d, %d, %d)", epoint.x, epoint.y, epoint.z);
        leg_set_end_point(legs[BACK_RIGHT], epoint.x, epoint.y, epoint.z);

        log_debug("solve kinematics");
        if ((kinematics_geometric(legs[FRONT_LEFT])) ||
            (kinematics_geometric(legs[FRONT_RIGHT])) ||
            (kinematics_geometric(legs[BACK_LEFT])) ||
            (kinematics_geometric(legs[BACK_RIGHT]))) {
            log_fatal("Failed to solve one of the kinematics");
            log_fatal("Frame: %d", frame);
        } else {
            log_debug("write command to SSC-32");
            leg_generate_cmd(legs, buf, NUM_LEGS);
            printf("cmd: %s\n", buf);
        }
        frame++;
    }
    return NULL;
}


int create_timer_callback(double sec, Leg **legs, Command *cmd) {
    pthread_t tid;
    struct update_thread_args *targs;
    if ((targs = malloc(sizeof(struct update_thread_args))) == NULL) {
        log_error("Failed to malloc space for thread args");
        return -1;
    }
    targs->legs = legs;
    targs->cmd = cmd;


    if (sem_init(&updatesem, 0, 0)) {
        log_error("Failed to init semaphore");
        return -1;
    }

    if (pthread_create(&tid, NULL, updater_thread, targs)) {
        log_error("Failed to create updater thread");
        return -1;
    }
    if (pthread_detach(tid)) {
        log_error("Failed to detach updater thread");
        return -1;
    }
    if (setinterrupt()) {
        log_error("Failed to setup interrupt handler");
        return -1;
    }

    if (timer_create(CLOCK_REALTIME, NULL, &timerid)) {
        log_error("Failed to create timer");
        return -1;
    }

    if (set_timer_interval(sec)) {
        log_error("Failed to set timer interval");
        return -1;
    }
    return 0;
}


void set_sequence(point_t (*func)(int, int, Command *)) {
    sequence_function = func;
    frame = 0;
}
