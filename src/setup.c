#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "setup.h"
#include "leg_constants.h"
#include "leg.h"


int setup_leg_init(Leg **legs) {
    legs[FRONT_LEFT] = leg_create("Front Left");
    legs[FRONT_RIGHT] = leg_create("Front Right");
    legs[BACK_LEFT] = leg_create("Back Left");
    legs[BACK_RIGHT] = leg_create("Back Right");

    if (legs[FRONT_LEFT] == NULL || legs[FRONT_RIGHT] == NULL ||
        legs[BACK_LEFT] == NULL || legs[BACK_RIGHT] == NULL)
    {
        goto ABORT;
    }


    leg_set_rotation(legs[FRONT_LEFT], FRONT_LEFT_ROTATION);
    leg_set_translation(legs[FRONT_LEFT], FRONT_LEFT_DELTA_X,
                        FRONT_LEFT_DELTA_Y, FRONT_LEFT_DELTA_Z);
    leg_set_coxa(legs[FRONT_LEFT], FRONT_LEFT_COXA);
    leg_set_femur(legs[FRONT_LEFT], FRONT_LEFT_FEMUR);
    leg_set_tibia(legs[FRONT_LEFT], FRONT_LEFT_TIBIA);
    if (leg_add_servo(legs[FRONT_LEFT], SHOULDER, FRONT_LEFT_SHOULDER_PIN))
        goto ABORT;
    if (leg_add_servo(legs[FRONT_LEFT], FEMUR, FRONT_LEFT_FEMUR_PIN))
        goto ABORT;
    if (leg_add_servo(legs[FRONT_LEFT], TIBIA, FRONT_LEFT_TIBIA_PIN))
        goto ABORT;
    leg_set_servo_range(legs[FRONT_LEFT], SHOULDER,
                        FRONT_LEFT_MIN_SHOULDER_POSITION,
                        FRONT_LEFT_MAX_SHOULDER_POSITION);
    leg_set_servo_range(legs[FRONT_LEFT], FEMUR,
                        FRONT_LEFT_MIN_FEMUR_POSITION,
                        FRONT_LEFT_MAX_FEMUR_POSITION);
    leg_set_servo_range(legs[FRONT_LEFT], TIBIA,
                        FRONT_LEFT_MIN_TIBIA_POSITION,
                        FRONT_LEFT_MAX_TIBIA_POSITION);
    leg_set_servo_zero_position(legs[FRONT_LEFT], SHOULDER,
                        FRONT_LEFT_SHOULDER_ZERO_POSITION);
    leg_set_servo_zero_position(legs[FRONT_LEFT], FEMUR,
                        FRONT_LEFT_FEMUR_ZERO_POSITION);
    leg_set_servo_zero_position(legs[FRONT_LEFT], TIBIA,
                        FRONT_LEFT_TIBIA_ZERO_POSITION);


    leg_set_rotation(legs[FRONT_RIGHT], FRONT_RIGHT_ROTATION);
    leg_set_translation(legs[FRONT_RIGHT], FRONT_RIGHT_DELTA_X,
                        FRONT_RIGHT_DELTA_Y, FRONT_RIGHT_DELTA_Z);
    leg_set_coxa(legs[FRONT_RIGHT], FRONT_RIGHT_COXA);
    leg_set_femur(legs[FRONT_RIGHT], FRONT_RIGHT_FEMUR);
    leg_set_tibia(legs[FRONT_RIGHT], FRONT_RIGHT_TIBIA);
    if (leg_add_servo(legs[FRONT_RIGHT], SHOULDER, FRONT_RIGHT_SHOULDER_PIN))
        goto ABORT;
    if (leg_add_servo(legs[FRONT_RIGHT], FEMUR, FRONT_RIGHT_FEMUR_PIN))
        goto ABORT;
    if (leg_add_servo(legs[FRONT_RIGHT], TIBIA, FRONT_RIGHT_TIBIA_PIN))
        goto ABORT;
    leg_set_servo_range(legs[FRONT_RIGHT], SHOULDER,
                        FRONT_RIGHT_MIN_SHOULDER_POSITION,
                        FRONT_RIGHT_MAX_SHOULDER_POSITION);
    leg_set_servo_range(legs[FRONT_RIGHT], FEMUR,
                        FRONT_RIGHT_MIN_FEMUR_POSITION,
                        FRONT_RIGHT_MAX_FEMUR_POSITION);
    leg_set_servo_range(legs[FRONT_RIGHT], TIBIA,
                        FRONT_RIGHT_MIN_TIBIA_POSITION,
                        FRONT_RIGHT_MAX_TIBIA_POSITION);
    leg_set_servo_zero_position(legs[FRONT_RIGHT], SHOULDER,
                        FRONT_RIGHT_SHOULDER_ZERO_POSITION);
    leg_set_servo_zero_position(legs[FRONT_RIGHT], FEMUR,
                        FRONT_RIGHT_FEMUR_ZERO_POSITION);
    leg_set_servo_zero_position(legs[FRONT_RIGHT], TIBIA,
                        FRONT_RIGHT_TIBIA_ZERO_POSITION);



    leg_set_rotation(legs[BACK_LEFT], BACK_LEFT_ROTATION);
    leg_set_translation(legs[BACK_LEFT], BACK_LEFT_DELTA_X,
                        BACK_LEFT_DELTA_Y, BACK_LEFT_DELTA_Z);
    leg_set_coxa(legs[BACK_LEFT], BACK_LEFT_COXA);
    leg_set_femur(legs[BACK_LEFT], BACK_LEFT_FEMUR);
    leg_set_tibia(legs[BACK_LEFT], BACK_LEFT_TIBIA);
    if (leg_add_servo(legs[BACK_LEFT], SHOULDER, BACK_LEFT_SHOULDER_PIN))
        goto ABORT;
    if (leg_add_servo(legs[BACK_LEFT], FEMUR, BACK_LEFT_FEMUR_PIN))
        goto ABORT;
    if (leg_add_servo(legs[BACK_LEFT], TIBIA, BACK_LEFT_TIBIA_PIN))
        goto ABORT;
    leg_set_servo_range(legs[BACK_LEFT], SHOULDER,
                        BACK_LEFT_MIN_SHOULDER_POSITION,
                        BACK_LEFT_MAX_SHOULDER_POSITION);
    leg_set_servo_range(legs[BACK_LEFT], FEMUR,
                        BACK_LEFT_MIN_FEMUR_POSITION,
                        BACK_LEFT_MAX_FEMUR_POSITION);
    leg_set_servo_range(legs[BACK_LEFT], TIBIA,
                        BACK_LEFT_MIN_TIBIA_POSITION,
                        BACK_LEFT_MAX_TIBIA_POSITION);
    leg_set_servo_zero_position(legs[BACK_LEFT], SHOULDER,
                        BACK_LEFT_SHOULDER_ZERO_POSITION);
    leg_set_servo_zero_position(legs[BACK_LEFT], FEMUR,
                        BACK_LEFT_FEMUR_ZERO_POSITION);
    leg_set_servo_zero_position(legs[BACK_LEFT], TIBIA,
                        BACK_LEFT_TIBIA_ZERO_POSITION);


    leg_set_rotation(legs[BACK_RIGHT], BACK_RIGHT_ROTATION);
    leg_set_translation(legs[BACK_RIGHT], BACK_RIGHT_DELTA_X,
                        BACK_RIGHT_DELTA_Y, BACK_RIGHT_DELTA_Z);
    leg_set_coxa(legs[BACK_RIGHT], BACK_RIGHT_COXA);
    leg_set_femur(legs[BACK_RIGHT], BACK_RIGHT_FEMUR);
    leg_set_tibia(legs[BACK_RIGHT], BACK_RIGHT_TIBIA);
    if (leg_add_servo(legs[BACK_RIGHT], SHOULDER, BACK_RIGHT_SHOULDER_PIN))
        goto ABORT;
    if (leg_add_servo(legs[BACK_RIGHT], FEMUR, BACK_RIGHT_FEMUR_PIN))
        goto ABORT;
    if (leg_add_servo(legs[BACK_RIGHT], TIBIA, BACK_RIGHT_TIBIA_PIN))
        goto ABORT;
    leg_set_servo_range(legs[BACK_RIGHT], SHOULDER,
                        BACK_RIGHT_MIN_SHOULDER_POSITION,
                        BACK_RIGHT_MAX_SHOULDER_POSITION);
    leg_set_servo_range(legs[BACK_RIGHT], FEMUR,
                        BACK_RIGHT_MIN_FEMUR_POSITION,
                        BACK_RIGHT_MAX_FEMUR_POSITION);
    leg_set_servo_range(legs[BACK_RIGHT], TIBIA,
                        BACK_RIGHT_MIN_TIBIA_POSITION,
                        BACK_RIGHT_MAX_TIBIA_POSITION);
    leg_set_servo_zero_position(legs[BACK_RIGHT], SHOULDER,
                        BACK_RIGHT_SHOULDER_ZERO_POSITION);
    leg_set_servo_zero_position(legs[BACK_RIGHT], FEMUR,
                        BACK_RIGHT_FEMUR_ZERO_POSITION);
    leg_set_servo_zero_position(legs[BACK_RIGHT], TIBIA,
                        BACK_RIGHT_TIBIA_ZERO_POSITION);

    return 0;

ABORT:
    leg_destroy(legs[FRONT_LEFT]);
    leg_destroy(legs[FRONT_RIGHT]);
    leg_destroy(legs[BACK_LEFT]);
    leg_destroy(legs[BACK_RIGHT]);
    return 1;

}
