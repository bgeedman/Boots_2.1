#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "kinematics.h"
#include "leg.h"
#include "logger.h"
#include "servo.h"


int main(int argc, char *argv[]) {
    int x, y, z;
    char buf[CMD_SIZE];

    if (argc < 4) {
        log_fatal("Usage: crapola <x> <y> <z>");
        exit(EXIT_FAILURE);
    }
    x = atoi(argv[1]);
    y = atoi(argv[2]);
    z = atoi(argv[3]);
    log_info("(%d, %d, %d)", x, y, z);

    Leg *leg = leg_create("Testing");
    if (leg == NULL) {
        log_fatal("failed to create leg");
    }
    leg_set_coxa(leg, 53);
    leg_set_femur(leg, 78);
    leg_set_tibia(leg, 122);
    leg_set_rotation(leg, 270.0);
    leg_set_translation(leg, 40, -40, 0);

    leg_add_servo(leg, SHOULDER, 27);
    leg_add_servo(leg, FEMUR, 28);
    leg_add_servo(leg, TIBIA, 29);

    leg_set_servo_range(leg, SHOULDER, 1115, 2015);
    leg_set_servo_range(leg, FEMUR, 650, 2500);
    leg_set_servo_range(leg, TIBIA, 735, 2275);

    leg_set_servo_zero_position(leg, SHOULDER, 2015);
    leg_set_servo_zero_position(leg, FEMUR, 1600);
    leg_set_servo_zero_position(leg, TIBIA, 735);

    leg_set_servo_inverted(leg, SHOULDER, -1);
    leg_set_servo_inverted(leg, FEMUR, -1);
    leg_set_servo_inverted(leg, TIBIA, 1);

    log_info("Leg should be created");
    leg_set_end_point(leg, x, y, z);
    kinematics_geometric(leg);
    leg_generate_cmd(&leg, buf, 1);
    log_info("cmd: %s", buf);
    leg_destroy(leg);

}
