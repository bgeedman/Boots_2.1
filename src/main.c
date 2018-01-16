#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <sys/stat.h>

#include "logger.h"
#include "kinematics.h"
#include "states.h"




int main(int argc, char *argv[]) {
    int (*current_state)(state_args *);
    int ret_state;
    state_args sa;

    sa.address = "127.0.0.1";
    sa.port = 15000;
    sa.cmd = NULL;



    current_state = state_table[ENTRY_STATE];
    for (;;) {
        ret_state = current_state(&sa);
        if (ret_state == EXIT_STATE) {
            break;
        }
        current_state = state_table[ret_state];
        sleep(1);
    }
    exit(EXIT_SUCCESS);
}
