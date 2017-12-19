#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "leg.h"
#include "leg_constants.h"
#include "logger.h"

#include "genesis.h"

static int genesis_front_left(Leg **leg);
static int genesis_front_right(Leg **leg);
static int genesis_back_left(Leg **leg);
static int genesis_back_right(Leg **leg);


int genesis_leg_creation(Leg **front_left, Leg **front_right,
                        Leg **back_left, Leg **back_right) {
    int status = genesis_front_left(front_left);
    if (status) {
        log_error("Failed to create Front Left leg");
        genesis_leg_destruction(*front_left,
                                *front_right,
                                *back_left,
                                *back_right);
        return 1;
    }
    status = genesis_front_right(front_right);
    if (status) {
        log_error("Failed to create Front Right leg");
        genesis_leg_destruction(*front_left,
                                *front_right,
                                *back_left,
                                *back_right);
        return 1;
    }
    status = genesis_back_left(back_left);
    if (status) {
        log_error("Failed to create Back Left leg");
        genesis_leg_destruction(*front_left,
                                *front_right,
                                *back_left,
                                *back_right);
        return 1;
    }
    status = genesis_back_right(back_right);
    if (status) {
        log_error("Failed to create Back Right leg");
        genesis_leg_destruction(*front_left,
                                *front_right,
                                *back_left,
                                *back_right);
        return 1;
    }
    return 0;
}

void genesis_leg_destruction(Leg *front_left, Leg *front_right,
                            Leg *back_left, Leg *back_right) {
    leg_destroy(front_left);
    leg_destroy(front_right);
    leg_destroy(back_left);
    leg_destroy(back_right);
}

static int genesis_front_left(Leg **leg) {
    *leg = leg_create("Front Left");
    if (*leg == NULL) {
        return 1;
    }
    leg_set_rotation(*leg, FRONT_LEFT_LEG_ROTATION);
    leg_set_translation(*leg,
                        FRONT_LEFT_LEG_DELTA_X,
                        FRONT_LEFT_LEG_DELTA_Y,
                        FRONT_LEFT_LEG_DELTA_Z);
    leg_set_coxa(*leg, FRONT_LEFT_LEG_COXA);
    leg_set_femur(*leg, FRONT_LEFT_LEG_FEMUR);
    leg_set_tibia(*leg, FRONT_LEFT_LEG_TIBIA);
    leg_add_servo(*leg, SHOULDER, FRONT_LEFT_LEG_SHOULDER_PIN);
    leg_add_servo(*leg, FEMUR, FRONT_LEFT_LEG_FEMUR_PIN);
    leg_add_servo(*leg, TIBIA, FRONT_LEFT_LEG_TIBIA_PIN);
    leg_set_servo_range(*leg, SHOULDER,
                        FRONT_LEFT_LEG_MIN_SHOULDER_POSITION,
                        FRONT_LEFT_LEG_MAX_SHOULDER_POSITION);
    leg_set_servo_range(*leg, FEMUR,
                        FRONT_LEFT_LEG_MIN_FEMUR_POSITION,
                        FRONT_LEFT_LEG_MAX_FEMUR_POSITION);
    leg_set_servo_range(*leg, TIBIA,
                        FRONT_LEFT_LEG_MIN_TIBIA_POSITION,
                        FRONT_LEFT_LEG_MAX_TIBIA_POSITION);
    return 0;
}

static int genesis_front_right(Leg **leg) {
    *leg = leg_create("Front Right");
    if (*leg == NULL) {
        return 1;
    }
    leg_set_rotation(*leg, FRONT_RIGHT_LEG_ROTATION);
    leg_set_translation(*leg,
                        FRONT_RIGHT_LEG_DELTA_X,
                        FRONT_RIGHT_LEG_DELTA_Y,
                        FRONT_RIGHT_LEG_DELTA_Z);
    leg_set_coxa(*leg, FRONT_RIGHT_LEG_COXA);
    leg_set_femur(*leg, FRONT_RIGHT_LEG_FEMUR);
    leg_set_tibia(*leg, FRONT_RIGHT_LEG_TIBIA);
    leg_add_servo(*leg, SHOULDER, FRONT_RIGHT_LEG_SHOULDER_PIN);
    leg_add_servo(*leg, FEMUR, FRONT_RIGHT_LEG_FEMUR_PIN);
    leg_add_servo(*leg, TIBIA, FRONT_RIGHT_LEG_TIBIA_PIN);
    leg_set_servo_range(*leg, SHOULDER,
                        FRONT_RIGHT_LEG_MIN_SHOULDER_POSITION,
                        FRONT_RIGHT_LEG_MAX_SHOULDER_POSITION);
    leg_set_servo_range(*leg, FEMUR,
                        FRONT_RIGHT_LEG_MIN_FEMUR_POSITION,
                        FRONT_RIGHT_LEG_MAX_FEMUR_POSITION);
    leg_set_servo_range(*leg, TIBIA,
                        FRONT_RIGHT_LEG_MIN_TIBIA_POSITION,
                        FRONT_RIGHT_LEG_MAX_TIBIA_POSITION);
    return 0;
}

static int genesis_back_left(Leg **leg) {
    *leg = leg_create("Back Left");
    if (*leg == NULL) {
        return 1;
    }
    leg_set_rotation(*leg, BACK_LEFT_LEG_ROTATION);
    leg_set_translation(*leg,
                        BACK_LEFT_LEG_DELTA_X,
                        BACK_LEFT_LEG_DELTA_Y,
                        BACK_LEFT_LEG_DELTA_Z);
    leg_set_coxa(*leg, BACK_LEFT_LEG_COXA);
    leg_set_femur(*leg, BACK_LEFT_LEG_FEMUR);
    leg_set_tibia(*leg, BACK_LEFT_LEG_TIBIA);
    leg_add_servo(*leg, SHOULDER, BACK_LEFT_LEG_SHOULDER_PIN);
    leg_add_servo(*leg, FEMUR, BACK_LEFT_LEG_FEMUR_PIN);
    leg_add_servo(*leg, TIBIA, BACK_LEFT_LEG_TIBIA_PIN);
    leg_set_servo_range(*leg, SHOULDER,
                        BACK_LEFT_LEG_MIN_SHOULDER_POSITION,
                        BACK_LEFT_LEG_MAX_SHOULDER_POSITION);
    leg_set_servo_range(*leg, FEMUR,
                        BACK_LEFT_LEG_MIN_FEMUR_POSITION,
                        BACK_LEFT_LEG_MAX_FEMUR_POSITION);
    leg_set_servo_range(*leg, TIBIA,
                        BACK_LEFT_LEG_MIN_TIBIA_POSITION,
                        BACK_LEFT_LEG_MAX_TIBIA_POSITION);
    return 0;
}

static int genesis_back_right(Leg **leg) {
    *leg = leg_create("Back Right");
    if (*leg == NULL) {
        return 1;
    }
    leg_set_rotation(*leg, BACK_RIGHT_LEG_ROTATION);
    leg_set_translation(*leg,
                        BACK_RIGHT_LEG_DELTA_X,
                        BACK_RIGHT_LEG_DELTA_Y,
                        BACK_RIGHT_LEG_DELTA_Z);
    leg_set_coxa(*leg, BACK_RIGHT_LEG_COXA);
    leg_set_femur(*leg, BACK_RIGHT_LEG_FEMUR);
    leg_set_tibia(*leg, BACK_RIGHT_LEG_TIBIA);
    leg_add_servo(*leg, SHOULDER, BACK_RIGHT_LEG_SHOULDER_PIN);
    leg_add_servo(*leg, FEMUR, BACK_RIGHT_LEG_FEMUR_PIN);
    leg_add_servo(*leg, TIBIA, BACK_RIGHT_LEG_TIBIA_PIN);
    leg_set_servo_range(*leg, SHOULDER,
                        BACK_RIGHT_LEG_MIN_SHOULDER_POSITION,
                        BACK_RIGHT_LEG_MAX_SHOULDER_POSITION);
    leg_set_servo_range(*leg, FEMUR,
                        BACK_RIGHT_LEG_MIN_FEMUR_POSITION,
                        BACK_RIGHT_LEG_MAX_FEMUR_POSITION);
    leg_set_servo_range(*leg, TIBIA,
                        BACK_RIGHT_LEG_MIN_TIBIA_POSITION,
                        BACK_RIGHT_LEG_MAX_TIBIA_POSITION);
    return 0;
}
