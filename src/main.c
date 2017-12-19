#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "servo.h"
#include "leg.h"
#include "logger.h"
#include "genesis.h"
#include "kinematics.h"



int main(int argc, char *argv[]) {
    Leg *front_left, *front_right, *back_left, *back_right;
    if (argc < 4) {
        log_fatal("Failed to pass required arguments");
        log_fatal("Usage: <x> <y> <z>");
        exit(1);
    }
    int status = genesis_leg_creation(&front_left,
                                      &front_right,
                                      &back_left,
                                      &back_right);
    if (status) {
        log_fatal("Failed to initialize legs");
        exit(1);
    }

    int x, y, z;
    x = atoi(argv[1]);
    y = atoi(argv[2]);
    z = atoi(argv[3]);
    /* genesis_leg_destruction(front_left, front_right, back_left, back_right); */
    leg_set_end_point(front_left, x, y, z);
    /* leg_set_end_point(front_right, 100, 100, -50); */
    /* leg_set_end_point(back_left, -1000, -100, -50); */
    /* leg_set_end_point(back_right, 100, -100, -50); */

    solve_kinematics_geometric(front_left);
    /* solve_kinematics(front_right); */
    /* solve_kinematics(back_left); */
    /* solve_kinematics(back_right); */

    leg_print_details(front_left);
    /* leg_print_details(front_right); */
    /* leg_print_details(back_left); */
    /* leg_print_details(back_right); */
    __dev_leg_print(front_left);


    genesis_leg_destruction(front_left, front_right, back_left, back_right);
/*
    leg_set_end_point(front_left, x, y, z); // set the end point (world coord)
    solve_kinematic(front_left);
    // timer triggers....
    update_leg_positions(front_left, front_right, back_left, back_right);
    */
    exit(0);
}

/* void update_leg_positions(Leg *front_left, Leg *front_right, back_left, back_right) { */
    // build the command string by checking if the current servo position is
    // the same as the desired position.  Difference indicates that we wish
    // to move this servo. Same indicates we don't
    // Once the cmd string it built, we can just shoot it out to the SSC-32
/* } */
