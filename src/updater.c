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

#include "logger.h"




#define BILLION 1000000000L

static void (*sequence_function)(void);
static int frame = 0;
static sem_t updatesem;


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


static int setperiodic(double sec) {
    timer_t timerid;
    struct itimerspec value;

    if (timer_create(CLOCK_REALTIME, NULL, &timerid)) {
        return -1;
    }
    value.it_interval.tv_sec = (long)sec;
    value.it_interval.tv_nsec = (sec - value.it_interval.tv_sec) * BILLION;
    if (value.it_interval.tv_nsec > BILLION) {
        value.it_interval.tv_sec++;
        value.it_interval.tv_nsec -= BILLION;
    }
    value.it_value = value.it_interval;
    return timer_settime(timerid, 0, &value, NULL);
}


void callback(int signo, siginfo_t *info, void *context) {
    if (sem_post(&updatesem)) {
        log_warn("failed to post to semaphore");
    }
}


void *updater_thread(void *data) {
    Leg **legs = (Leg **)data;
    log_trace("Update thread");
    while (sem_wait(&updatesem) == 0) {
        log_debug("get next leg postiion");
        // call sequence function with the frame number


        log_debug("update leg positions");
        leg_set_end_point(legs[FRONT_LEFT], 0, 0, 0);
        leg_set_end_point(legs[FRONT_RIGHT], 0, 0, 0);
        leg_set_end_point(legs[BACK_LEFT], 0, 0, 0);
        leg_set_end_point(legs[BACK_RIGHT], 0, 0, 0);


        log_debug("solve kinematics");
        kinematics_geometric(legs[FRONT_LEFT]);
        kinematics_geometric(legs[FRONT_RIGHT]);
        kinematics_geometric(legs[BACK_LEFT]);
        kinematics_geometric(legs[BACK_RIGHT]);

        log_debug("write command to SSC-32");

    }
    return NULL;
}


int create_timer_callback(double sec, Leg **legs) {
    pthread_t tid;
    if (sem_init(&updatesem, 0, 0)) {
        log_error("Failed to init semaphore");
        return -1;
    }
    if (pthread_create(&tid, NULL, updater_thread, legs)) {
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

    if (setperiodic(sec)) {
        log_error("Failed to make timer periodic");
        return -1;
    }
    return 0;
}


void set_sequence(void (*foo)(void)) {
    // might need a lock on this
    sequence_function = foo;
    frame = 0;
}
