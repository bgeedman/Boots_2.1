#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "commands.pb-c.h"
#include "kinematics.h"
#include "leg.h"
#include "logger.h"
#include "sequences.h"
#include "tools.h"
#include "updater.h"

#define BILLION 1000000000L

static point_t (*sequence_function)(int, Leg *);
static sem_t updatesem;
static timer_t timerid;

Command *command;
unsigned long frame;


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


void callback(int signo, siginfo_t *info, void *context) {
    if (sem_post(&updatesem)) {
        log_warn("Failed to post to semaphore");
    }
}



void *updater_thread(void *data) {
    struct update_thread_args *args = (struct update_thread_args *)data;
    Leg **legs = args->legs;
    char buf[1024];
    int i;
    point_t epoint;

    while (sem_wait(&updatesem) == 0) {
        log_debug("updating leg positions for frame: %d", frame);
        for (i = 0; i < NUM_LEGS; i++) {
            epoint = sequence_function(i, legs[i]);
            if (epoint.x == 0 && epoint.y == 0 && epoint.z == 0) {
                log_warn("Not updating %s", legs[i]->label);
            } else {
                log_debug("%s: (%d, %d, %d)", legs[i]->label,
                                              epoint.x,
                                              epoint.y,
                                              epoint.z);
                leg_set_end_point(legs[i], epoint.x, epoint.y, epoint.z);
            }
        }

        if ((kinematics_geometric(legs[FRONT_LEFT])) ||
            (kinematics_geometric(legs[FRONT_RIGHT])) ||
            (kinematics_geometric(legs[BACK_LEFT])) ||
            (kinematics_geometric(legs[BACK_RIGHT]))) {
            log_error("Failed to solve one of the kinematics");
            log_error("Frame: %d", frame);
        } else {
            leg_generate_cmd(legs, buf, NUM_LEGS);
            log_debug("Sending command: %s", buf);
            write_command(buf);
        }
        frame++;
    }

    return NULL;
}



int create_timer_callback(double sec, Leg **legs) {
    pthread_t tid;
    struct update_thread_args *targs;
    int error;

    if ((targs = malloc(sizeof(struct update_thread_args))) == NULL) {
        error = errno;
        log_fatal("Failed to malloc space for thread args: %s",
                    strerror(errno));
        errno = error;
        return -1;
    }
    targs->legs = legs;


    if (sem_init(&updatesem, 0, 0)) {
        error = errno;
        log_fatal("Failed to init semaphore: %s", strerror(errno));
        free(targs);
        errno = error;
        return -1;
    }

    if (open_serial_port("/dev/ttyUSB0")) {
        log_fatal("Failed to open serial port");
        free(targs);
        return -1;
    }

    if (pthread_create(&tid, NULL, updater_thread, targs)) {
        error = errno;
        log_fatal("Failed to create updater thread: %s", strerror(errno));
        free(targs);
        errno = error;
        return -1;
    }

    if (pthread_detach(tid)) {
        error = errno;
        log_fatal("Failed to detach updater thread: %s", strerror(errno));
        free(targs);
        errno = error;
        return -1;
    }

    if (setinterrupt()) {
        log_fatal("Failed to setup interrupt handler");
        free(targs);
        return -1;
    }

    if (timer_create(CLOCK_REALTIME, NULL, &timerid)) {
        error = errno;
        log_fatal("Failed to create timer: %s", strerror(errno));
        free(targs);
        errno = error;
        return -1;
    }

    if (set_timer_interval(sec)) {
        log_fatal("Failed to set timer interval");
        free(targs);
        return -1;
    }

    return 0;
}


void set_sequence(point_t (*func)(int, Leg*)) {
    sequence_function = func;
    frame = 0;
}
