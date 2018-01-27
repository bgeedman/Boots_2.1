#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "kinematics.h"
#include "leg.h"
#include "servo.h"
#include "leg_constants.h"

int create_front_left_leg(Leg **leg);
int create_front_right_leg(Leg **leg);
int create_back_left_leg(Leg **leg);
int create_back_right_leg(Leg **leg);

static const char *labels[] = { "Front Left", "Front Right", "Back Left", "Back Right" };

static int(*create_function[])(Leg **leg) = {
    create_front_left_leg,
    create_front_right_leg,
    create_back_left_leg,
    create_back_right_leg
};

int main(int argc, char *argv[]) {
    int x, y, z;
    char buf[CMD_SIZE];
    Leg *leg;
    int cmd;

    if (argc < 5) {
        fprintf(stderr, "Usage: testing <leg> <x> <y> <z>\n");
        fprintf(stderr, "leg values:\n");
        fprintf(stderr, "\t0 - front left\n");
        fprintf(stderr, "\t1 - front right\n");
        fprintf(stderr, "\t2 - back left\n");
        fprintf(stderr, "\t3 - back right\n");
        exit(EXIT_FAILURE);
    }
    cmd = atoi(argv[1]);
    x = atoi(argv[2]);
    y = atoi(argv[3]);
    z = atoi(argv[4]);

    if (cmd > 3) {
        fprintf(stderr, "Invalid leg number %d\nPlease use value between 0-3\n", cmd);
        exit(EXIT_FAILURE);
    }


    printf("Using leg: %s\n", labels[cmd]);
    printf("using coordinate: (%d, %d, %d)\n", x, y, z);

    if (create_function[cmd](&leg)) {
        fprintf(stderr, "Failed to create leg\n");
        exit(EXIT_FAILURE);
    }

    printf("Everything should be succesfully created\n");
    leg_set_end_point(leg, x, y, z);
    kinematics_geometric(leg);
    leg_generate_cmd(&leg, buf, 1);
    printf("Command: %s\n", buf);
    leg_destroy(leg);

}


int create_front_left_leg(Leg **leg) {
    *leg = leg_create("Front Left");
    if (leg == NULL) {
        return -1;
    }
    leg_set_coxa(*leg, FRONT_LEFT_COXA);
    leg_set_femur(*leg, FRONT_LEFT_FEMUR);
    leg_set_tibia(*leg, FRONT_LEFT_TIBIA);
    leg_set_rotation(*leg, FRONT_LEFT_ROTATION);
    leg_set_translation(*leg, FRONT_LEFT_DELTA_X, FRONT_LEFT_DELTA_Y, FRONT_LEFT_DELTA_Z);

    leg_add_servo(*leg, SHOULDER, FRONT_LEFT_SHOULDER_PIN);
    leg_add_servo(*leg, FEMUR, FRONT_LEFT_FEMUR_PIN);
    leg_add_servo(*leg, TIBIA, FRONT_LEFT_TIBIA_PIN);

    leg_set_servo_range(*leg, SHOULDER, FRONT_LEFT_MIN_SHOULDER_POSITION, FRONT_LEFT_MAX_SHOULDER_POSITION);
    leg_set_servo_range(*leg, FEMUR, FRONT_LEFT_MIN_FEMUR_POSITION, FRONT_LEFT_MAX_FEMUR_POSITION);
    leg_set_servo_range(*leg, TIBIA, FRONT_LEFT_MIN_TIBIA_POSITION, FRONT_LEFT_MAX_TIBIA_POSITION);

    leg_set_servo_zero_position(*leg, SHOULDER, FRONT_LEFT_SHOULDER_ZERO_POSITION);
    leg_set_servo_zero_position(*leg, FEMUR, FRONT_LEFT_FEMUR_ZERO_POSITION);
    leg_set_servo_zero_position(*leg, TIBIA, FRONT_LEFT_TIBIA_ZERO_POSITION);

    leg_set_servo_inverted(*leg, SHOULDER, FRONT_LEFT_SHOULDER_INVERTED);
    leg_set_servo_inverted(*leg, FEMUR, FRONT_LEFT_FEMUR_INVERTED);
    leg_set_servo_inverted(*leg, TIBIA, FRONT_LEFT_TIBIA_INVERTED);
    return 0;
}

int create_front_right_leg(Leg **leg) {
    *leg = leg_create("Front Right");
    if (leg == NULL) {
        return -1;
    }
    leg_set_coxa(*leg, FRONT_RIGHT_COXA);
    leg_set_femur(*leg, FRONT_RIGHT_FEMUR);
    leg_set_tibia(*leg, FRONT_RIGHT_TIBIA);
    leg_set_rotation(*leg, FRONT_RIGHT_ROTATION);
    leg_set_translation(*leg, FRONT_RIGHT_DELTA_X, FRONT_RIGHT_DELTA_Y, FRONT_RIGHT_DELTA_Z);

    leg_add_servo(*leg, SHOULDER, FRONT_RIGHT_SHOULDER_PIN);
    leg_add_servo(*leg, FEMUR, FRONT_RIGHT_FEMUR_PIN);
    leg_add_servo(*leg, TIBIA, FRONT_RIGHT_TIBIA_PIN);

    leg_set_servo_range(*leg, SHOULDER, FRONT_RIGHT_MIN_SHOULDER_POSITION, FRONT_RIGHT_MAX_SHOULDER_POSITION);
    leg_set_servo_range(*leg, FEMUR, FRONT_RIGHT_MIN_FEMUR_POSITION, FRONT_RIGHT_MAX_FEMUR_POSITION);
    leg_set_servo_range(*leg, TIBIA, FRONT_RIGHT_MIN_TIBIA_POSITION, FRONT_RIGHT_MAX_TIBIA_POSITION);

    leg_set_servo_zero_position(*leg, SHOULDER, FRONT_RIGHT_SHOULDER_ZERO_POSITION);
    leg_set_servo_zero_position(*leg, FEMUR, FRONT_RIGHT_FEMUR_ZERO_POSITION);
    leg_set_servo_zero_position(*leg, TIBIA, FRONT_RIGHT_TIBIA_ZERO_POSITION);

    leg_set_servo_inverted(*leg, SHOULDER, FRONT_RIGHT_SHOULDER_INVERTED);
    leg_set_servo_inverted(*leg, FEMUR, FRONT_RIGHT_FEMUR_INVERTED);
    leg_set_servo_inverted(*leg, TIBIA, FRONT_RIGHT_TIBIA_INVERTED);
    return 0;
}

int create_back_left_leg(Leg **leg) {
    *leg = leg_create("Back Left");
    if (leg == NULL) {
        return -1;
    }
    leg_set_coxa(*leg, BACK_LEFT_COXA);
    leg_set_femur(*leg, BACK_LEFT_FEMUR);
    leg_set_tibia(*leg, BACK_LEFT_TIBIA);
    leg_set_rotation(*leg, BACK_LEFT_ROTATION);
    leg_set_translation(*leg, BACK_LEFT_DELTA_X, BACK_LEFT_DELTA_Y, BACK_LEFT_DELTA_Z);

    leg_add_servo(*leg, SHOULDER, BACK_LEFT_SHOULDER_PIN);
    leg_add_servo(*leg, FEMUR, BACK_LEFT_FEMUR_PIN);
    leg_add_servo(*leg, TIBIA, BACK_LEFT_TIBIA_PIN);

    leg_set_servo_range(*leg, SHOULDER, BACK_LEFT_MIN_SHOULDER_POSITION, BACK_LEFT_MAX_SHOULDER_POSITION);
    leg_set_servo_range(*leg, FEMUR, BACK_LEFT_MIN_FEMUR_POSITION, BACK_LEFT_MAX_FEMUR_POSITION);
    leg_set_servo_range(*leg, TIBIA, BACK_LEFT_MIN_TIBIA_POSITION, BACK_LEFT_MAX_TIBIA_POSITION);

    leg_set_servo_zero_position(*leg, SHOULDER, BACK_LEFT_SHOULDER_ZERO_POSITION);
    leg_set_servo_zero_position(*leg, FEMUR, BACK_LEFT_FEMUR_ZERO_POSITION);
    leg_set_servo_zero_position(*leg, TIBIA, BACK_LEFT_TIBIA_ZERO_POSITION);

    leg_set_servo_inverted(*leg, SHOULDER, BACK_LEFT_SHOULDER_INVERTED);
    leg_set_servo_inverted(*leg, FEMUR, BACK_LEFT_FEMUR_INVERTED);
    leg_set_servo_inverted(*leg, TIBIA, BACK_LEFT_TIBIA_INVERTED);
    return 0;
}

int create_back_right_leg(Leg **leg) {
    *leg = leg_create("Back Right");
    if (leg == NULL) {
        return -1;
    }
    leg_set_coxa(*leg, BACK_RIGHT_COXA);
    leg_set_femur(*leg, BACK_RIGHT_FEMUR);
    leg_set_tibia(*leg, BACK_RIGHT_TIBIA);
    leg_set_rotation(*leg, BACK_RIGHT_ROTATION);
    leg_set_translation(*leg, BACK_RIGHT_DELTA_X, BACK_RIGHT_DELTA_Y, BACK_RIGHT_DELTA_Z);

    leg_add_servo(*leg, SHOULDER, BACK_RIGHT_SHOULDER_PIN);
    leg_add_servo(*leg, FEMUR, BACK_RIGHT_FEMUR_PIN);
    leg_add_servo(*leg, TIBIA, BACK_RIGHT_TIBIA_PIN);

    leg_set_servo_range(*leg, SHOULDER, BACK_RIGHT_MIN_SHOULDER_POSITION, BACK_RIGHT_MAX_SHOULDER_POSITION);
    leg_set_servo_range(*leg, FEMUR, BACK_RIGHT_MIN_FEMUR_POSITION, BACK_RIGHT_MAX_FEMUR_POSITION);
    leg_set_servo_range(*leg, TIBIA, BACK_RIGHT_MIN_TIBIA_POSITION, BACK_RIGHT_MAX_TIBIA_POSITION);

    leg_set_servo_zero_position(*leg, SHOULDER, BACK_RIGHT_SHOULDER_ZERO_POSITION);
    leg_set_servo_zero_position(*leg, FEMUR, BACK_RIGHT_FEMUR_ZERO_POSITION);
    leg_set_servo_zero_position(*leg, TIBIA, BACK_RIGHT_TIBIA_ZERO_POSITION);

    leg_set_servo_inverted(*leg, SHOULDER, BACK_RIGHT_SHOULDER_INVERTED);
    leg_set_servo_inverted(*leg, FEMUR, BACK_RIGHT_FEMUR_INVERTED);
    leg_set_servo_inverted(*leg, TIBIA, BACK_RIGHT_TIBIA_INVERTED);
    return 0;
}


