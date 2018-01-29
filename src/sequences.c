#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_blas.h>
#include <math.h>


#include "leg.h"
#include "tools.h"
#include "logger.h"
#include "sequences.h"
#include "commands.pb-c.h"
#include "server.h"

Command *command;
unsigned long frame = 0;

//point_t seq_unknown_to_park(int frame, int leg) {
point_t seq_unknown_to_park(int leg_num, Leg *leg) {
    log_trace("unknown_to_park: %d", frame);
    static int NUMBER_OF_FRAMES = 0;
    static point_t seq[] = {
        {-150, 40, 30}, {150, 40, 30}, {-150, -50, 30}, {150, -40, 30}
    };

    if (frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * frame + leg_num];
}




point_t seq_park_to_stand(int leg_num, Leg *leg) {
    log_trace("park_to_stand: %d", frame);
    static int NUMBER_OF_FRAMES = 8;

    static point_t seq[] ={
        {-150, 40, 30}, {150, 40, 30}, {-150, -40, 30}, {150, -40, 30},
        {-175, 40, 15}, {175, 40, 15}, {-175, -40, 15}, {175, -40, 15},
        {-200, 40, 0}, {200, 40, 0}, {-200, -40, 0}, {200, -40, 0},
        {-200, 40, 0}, {200, 40, 0}, {-200, -40, 0}, {200, -40, 0},
        {-150, 150, 0}, {150, 150, 0}, {-150, -150, 0}, {150, -150, 0},
        {-150, 150, 25}, {150, 150, 25}, {-150, -150, 25}, {150, -150, 25},
        {-150, 150, 50}, {150, 150, 50}, {-150, -150, 50}, {150, -150, 50},
        {-150, 150, 75}, {150, 150, 75}, {-150, -150, 75}, {150, -150, 75},
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100}
    };


    if (frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * frame + leg_num];
}


/*
 * TODO: MIGHT CHANGE THIS SEQUENCE SO IT DOES ALL FOUR LEGS AT ONCE TO KEEP
 * BALANCE
 */
point_t seq_stand_to_park(int leg_num, Leg *leg) {
    log_trace("stand_to_park: %d", frame);
    static int NUMBER_OF_FRAMES = 8;
    static point_t seq[] = {
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100},
        {-150, 150, 75}, {150, 150, 75}, {-150, -150, 75}, {150, -150, 75},
        {-150, 150, 50}, {150, 150, 50}, {-150, -150, 50}, {150, -150, 50},
        {-150, 150, 25}, {150, 150, 25}, {-150, -150, 25}, {150, -150, 25},
        {-150, 150, 0}, {150, 150, 0}, {-150, -150, 0}, {150, -150, 0},
        {-200, 40, 0}, {200, 40, 0}, {-200, -40, 0}, {200, -40, 0},
        {-200, 40, 0}, {200, 40, 0}, {-200, -40, 0}, {200, -40, 0},
        {-175, 40, 15}, {175, 40, 15}, {-175, -40, 15}, {175, -40, 15},
        {-150, 40, 30}, {150, 40, 30}, {-150, -40, 30}, {150, -40, 30},
    };

    if (frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * frame + leg_num];
}





point_t seq_stand_to_stretch(int leg_num, Leg *leg) {
    float roll, pitch, yaw;
    int delta_x, delta_y;
    gsl_matrix *original_point = NULL;


    gsl_matrix *pitch_rotation = NULL;
    gsl_matrix *yaw_rotation = NULL;
    gsl_matrix *roll_rotation = NULL;


    gsl_matrix *trans = NULL;
    point_t ret = {0, 0, 0};

    pthread_mutex_lock(&cmd_mutex);
        roll = command->roll;
        pitch = command->pitch;
        yaw = command->yaw;
        delta_x = command->delta_x;
        delta_y = command->delta_y;
    pthread_mutex_unlock(&cmd_mutex);
    log_info("Roll: %.2f, Pitch: %.2f, Yaw: %.2f, Delta_x: %d, Delta_y: %d",
            roll, pitch, yaw, delta_x, delta_y);

    static point_t seq[] = {
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100}
    };


    // I should really only do this once
    if ((original_point = gsl_matrix_alloc(4, 1)) == NULL) {
        log_warn("Failed to allocate matrix fro stretching");
        goto FAIL;
    }

    if ((trans = gsl_matrix_alloc(4, 4)) == NULL) {
        log_warn("Failed to alloc trans matrix");
        goto FAIL;
    }
    gsl_matrix_set(original_point, 0, 0, seq[leg_num].x);
    gsl_matrix_set(original_point, 1, 0, seq[leg_num].y);
    gsl_matrix_set(original_point, 2, 0, seq[leg_num].z);
    gsl_matrix_set(original_point, 3, 0, 1);

    /* gsl_matrix_set_identity(rot); */
    gsl_matrix_set_identity(trans);

    if ((yaw_rotation = gsl_matrix_alloc(4, 4)) == NULL) {
        log_warn("Failed to allocate yaw rotation matrix");
        goto FAIL;
    }
    gsl_matrix_set_identity(yaw_rotation);
    gsl_matrix_set(yaw_rotation, 0, 0, cos(degreesToRadians(yaw)));
    gsl_matrix_set(yaw_rotation, 0, 1, sin(degreesToRadians(yaw)));
    gsl_matrix_set(yaw_rotation, 1, 0, -sin(degreesToRadians(yaw)));
    gsl_matrix_set(yaw_rotation, 1, 1, cos(degreesToRadians(yaw)));

    if ((roll_rotation = gsl_matrix_alloc(4, 4)) == NULL) {
        log_warn("Failed to allocate roll rotation matrix");
        goto FAIL;
    }
    gsl_matrix_set_identity(roll_rotation);
    gsl_matrix_set(roll_rotation, 0, 0, cos(degreesToRadians(roll)));
    gsl_matrix_set(roll_rotation, 0, 2, sin(degreesToRadians(roll)));
    gsl_matrix_set(roll_rotation, 2, 0, -sin(degreesToRadians(roll)));
    gsl_matrix_set(roll_rotation, 2, 2, cos(degreesToRadians(roll)));

    if ((pitch_rotation = gsl_matrix_alloc(4, 4)) == NULL) {
        log_warn("Failed to allocate pitch rotation matrix");
        goto FAIL;
    }
    gsl_matrix_set_identity(pitch_rotation);
    gsl_matrix_set(pitch_rotation, 1, 1, cos(degreesToRadians(pitch)));
    gsl_matrix_set(pitch_rotation, 1, 2, sin(degreesToRadians(pitch)));
    gsl_matrix_set(pitch_rotation, 2, 1, -sin(degreesToRadians(pitch)));
    gsl_matrix_set(pitch_rotation, 2, 2, cos(degreesToRadians(pitch)));


    // load the translation matrix
    gsl_matrix_set(trans, 0, 3, 50 * delta_x);
    gsl_matrix_set(trans, 1, 3, 50 * delta_y);

    gsl_matrix *tmp = NULL;
    gsl_matrix *tmp2 = NULL;

    if ((tmp = gsl_matrix_alloc(4, 1)) == NULL) {
        log_error("Failed to allocate tmp matrix\n");
        goto FAIL;
    }

    if ((tmp2 = gsl_matrix_alloc(4, 1)) == NULL) {
        log_error("Failed to allocate tmp matrix\n");
        goto FAIL;
    }

    // mutliply the translate by the end point
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,
                    trans, original_point, 0.0, tmp); // store in tmp

    // multiply the rotate by the end point
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,
                    yaw_rotation, tmp, 0.0, tmp2); // store in tmp2

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,
                    pitch_rotation, tmp2, 0.0, tmp); // store in tmp

    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,
                    roll_rotation, tmp, 0.0, tmp2); // store in tmp2


    ret.x = gsl_matrix_get(tmp2, 0, 0);
    ret.y = gsl_matrix_get(tmp2, 1, 0);
    ret.z = gsl_matrix_get(tmp2, 2, 0);

FAIL:
    gsl_matrix_free(original_point);
    gsl_matrix_free(yaw_rotation);
    gsl_matrix_free(trans);
    gsl_matrix_free(tmp);
    gsl_matrix_free(tmp2);
    return ret;
}


/*
 * TODO:
 * Might need to make this less than 45 degrees, make things a bit unstable.
 * Also need to add in the creep turn
 */
point_t seq_stand_to_turn_left(int leg_num, Leg *leg) {
    log_trace("stand_to_turn_left: %d", frame);
    static int NUMBER_OF_FRAMES = 8;
    static point_t seq[] = {
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100},
        /*---------------- MOVE FRONT RIGHT AND BACK LEFT ------------------*/
        {-150, 150, 100}, {150, 150, 30}, {-150, -150, 30}, {150, -150, 100},
        {-150, 150, 100}, {118, 176, 30}, {-118, -176, 30}, {150, -150, 100},
        {-150, 150, 100}, {81, 196, 30}, {-81, -196, 30}, {150, -150, 100},
        {-150, 150, 100}, {81, 196, 100}, {-81, -196, 100}, {150, -150, 100},
        /*-----------------------------------------------------------------*/
        /*----- NOW LIFT FRONT LEFT AND BACK RIGHT AND ROTATE BACK --------*/
        {-150, 150, 30}, {81, 196, 100}, {-81, -196, 100}, {150, -150, 30},
        {-150, 150, 30}, {118, 176, 100}, {-118, -176, 100}, {150, -150, 30},
        {-150, 150, 30}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 30},
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100},
        /*-----------------------------------------------------------------*/
    };
    if (frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * frame + leg_num];
}




point_t seq_stand_to_turn_right(int leg_num, Leg *leg) {
    log_trace("stand_to_turn_right: %d", frame);
    static int NUMBER_OF_FRAMES = 8;
    static point_t seq[] = {
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100},
        /*---------------- MOVE FRONT LEFT AND BACK RIGHT ------------------*/
        {-150, 150, 30}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 30},
        {-118, 176, 30}, {150, 150, 100}, {-150, -150, 100}, {118, -176, 30},
        {-81, 196, 30}, {150, 150, 100}, {-150, -150, 100}, {81, -196, 30},
        {-81, 196, 100}, {150, 150, 100}, {-150, -150, 100}, {81, -196, 100},
        {-81, 196, 100}, {150, 150, 30}, {-150, -150, 30}, {81, -196, 100},
        {-118, 176, 100}, {150, 150, 30}, {-150, -150, 30}, {118, -176, 100},
        {-150, 150, 100}, {150, 150, 30}, {-150, -150, 30}, {150, -150, 100},
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100},
    };
    if (frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * frame + leg_num];
}




/*
 * TODO:
 *   Need to add in the body shift. It is currently too unstable
 *   Do i need to shift forward and back as well...?
 *
 *   Body shift right => move the FRONT_LEFT and BACK_LEFT legs further out and FRONT_RIGHT and BACK_RIGHT closer
 *   Body shift left => move the FRONT_RIGHT and BACK_RIGHT legs further out and FRONT_LEFT and BACK_LEFT closer
 */
point_t seq_stand_to_walk(int leg_num, Leg *leg) {
    log_trace("stand_to_walk: %d", frame);
    static int NUMBER_OF_FRAMES = 45;
    static point_t seq[] = {
        /* FRONT_LEFT      FRONT_RIGHT      BACK_LEFT           BACK_RIGHT */
        /*------------------- INITIAL LEG SETUP ----------------------------*/
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100},
        {-150, 166, 86}, {150, 145, 100}, {-150, -145, 100}, {150, -166, 100},
        {-150, 182, 30}, {150, 140, 100}, {-150, -140, 100}, {150, -182, 100},
        {-150, 198, 30}, {150, 134, 100}, {-150, -134, 100}, {150, -198, 100}, /* Start shifting left */
        {-150, 214, 65}, {150, 129, 100}, {-150, -129, 100}, {150, -214, 100},
        {-150, 230, 100}, {150, 123, 100}, {-150, -123, 100}, {150, -230, 100},
        /*------------------------------------------------------------------*/
        /*------------------- BACK RIGHT STEP ------------------------------*/
        {-150, 224, 100}, {150, 118, 100}, {-150, -128, 100}, {150, -214, 65}, /* End body shift left */
        {-150, 219, 100}, {150, 112, 100}, {-150, -134, 100}, {150, -198, 30},
        {-150, 214, 100}, {150, 107, 100}, {-150, -139, 100}, {150, -182, 30},
        {-150, 208, 100}, {150, 102, 100}, {-150, -144, 100}, {150, -166, 30},
        {-150, 203, 100}, {150, 96, 100}, {-150, -150, 100}, {150, -150, 30},
        {-150, 198, 100}, {150, 91, 100}, {-150, -155, 100}, {150, -134, 30},
        {-150, 192, 100}, {150, 86, 100}, {-150, -160, 100}, {150, -118, 30},
        {-150, 187, 100}, {150, 80, 100}, {-150, -166, 100}, {150, -102, 30},
        {-150, 182, 100}, {150, 75, 100}, {-150, -172, 100}, {150, -86, 65},
        {-150, 176, 100}, {150, 70, 100}, {-150, -176, 100}, {150, -70, 100},
        /*------------------------------------------------------------------*/
        /*------------------- FRONT RIGHT STEP -----------------------------*/
        {-150, 171, 100}, {150, 86, 65}, {-150, -182, 100}, {150, -75, 100},
        {-150, 166, 100}, {150, 102, 30}, {-150, -182, 100}, {150, -80, 100},
        {-150, 160, 100}, {150, 118, 30}, {-150, -192, 100}, {150, -85, 100},
        {-150, 155, 100}, {150, 134, 30}, {-150, -198, 100}, {150, -91, 100},
        {-150, 150, 100}, {150, 150, 30}, {-150, -203, 100}, {150, -96, 100},
        {-150, 144, 100}, {150, 166, 30}, {-150, -208, 100}, {150, -102, 100},
        {-150, 139, 100}, {150, 182, 30}, {-150, -214, 100}, {150, -107, 100},
        {-150, 134, 100}, {150, 198, 30}, {-150, -219, 100}, {150, -112, 100}, /* start shifting right */
        {-150, 128, 100}, {150, 214, 65}, {-150, -224, 100}, {150, -118, 100},
        {-150, 123, 100}, {150, 230, 100}, {-150, -230, 100}, {150, -123, 100},
        /*------------------------------------------------------------------*/
        /*------------------- BACK LEFT STEP -------------------------------*/
        {-150, 118, 100}, {150, 224, 100}, {-150, -214, 65}, {150, -128, 100}, /* end body shift right */
        {-150, 112, 100}, {150, 219, 100}, {-150, -198, 30}, {150, -134, 100},
        {-150, 107, 100}, {150, 214, 100}, {-150, -182, 30}, {150, -139, 100},
        {-150, 102, 100}, {150, 208, 100}, {-150, -166, 30}, {150, -144, 100},
        {-150, 96, 100}, {150, 203, 100}, {-150, -150, 30}, {150, -150, 100},
        {-150, 91, 100}, {150, 198, 100}, {-150, -134, 30}, {150, -155, 100},
        {-150, 86, 100}, {150, 192, 100}, {-150, -118, 30}, {150, -160, 100},
        {-150, 80, 100}, {150, 187, 100}, {-150, -102, 30}, {150, -166, 100},
        {-150, 75, 100}, {150, 182, 100}, {-150, -86, 65}, {150, -171, 100},
        {-150, 70, 100}, {150, 176, 100}, {-150, -70, 100}, {150, -176, 100},
        /*------------------------------------------------------------------*/
        /*------------------- FRONT LEFT STEP ------------------------------*/
        {-150, 86, 65}, {150, 171, 100}, {-150, -75, 100}, {150, -182, 100},
        {-150, 102, 30}, {150, 166, 100}, {-150, -80, 100}, {150, -187, 100},
        {-150, 118, 30}, {150, 160, 100}, {-150, -86, 100}, {150, -192, 100},
        {-150, 134, 30}, {150, 155, 100}, {-150, -91, 100}, {150, -198, 100},
        {-150, 150, 30}, {150, 150, 100}, {-150, -96, 100}, {150, -203, 100},
        {-150, 166, 30}, {150, 144, 100}, {-150, -102, 100}, {150, -208, 100},
        {-150, 182, 30}, {150, 139, 100}, {-150, -107, 100}, {150, -214, 100},
        {-150, 198, 30}, {150, 134, 100}, {-150, -112, 100}, {150, -219, 100}, /* Start shifting left */
        {-150, 214, 65}, {150, 128, 100}, {-150, -118, 100}, {150, -224, 100},
        {-150, 230, 100}, {150, 123, 100}, {-150, -123, 100}, {150, -230, 100}
        /*------------------------------------------------------------------*/

    };
    if (frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        frame = 6;
    }
    return seq[4 * frame + leg_num];
}


point_t seq_stop_and_center(int leg_num, Leg *leg) {
    log_trace("stop_and_center: %d", frame);
    static int NUMBER_OF_FRAMES = 16;
    // first frame just drop all the feet to the ground
    static point_t seq[] = {
        /*---------------- DROP ALL THE FEET -------------------------------*/
        {0, 0, 100},      {0, 0, 100},      {0, 0, 100},       {0, 0, 100},
        /*---------------- CENTER FRONT LEFT -------------------------------*/
        {0, 0, 65},       {0, 0, 100},      {0, 0, 100},       {0, 0, 100},
        {0, 0, 30},       {0, 0, 100},      {0, 0, 100},       {0, 0, 100},
        {-150, 150, 65},  {0, 0, 100},      {0, 0, 100},       {0, 0, 100},
        {-150, 150, 100}, {0, 0, 100},      {0, 0, 100},       {0, 0, 100},
        /*------------------------------------------------------------------*/
        /*---------------- CENTER BACK RIGHT -------------------------------*/
        {-150, 150, 100}, {0, 0, 100},      {0, 0, 65},        {0, 0, 100},
        {-150, 150, 100}, {0, 0, 100},      {0, 0, 30},        {0, 0, 100},
        {-150, 150, 100}, {0, 0, 100},      {-150, -150, 65},  {0, 0, 100},
        {-150, 150, 100}, {0, 0, 100},      {-150, -150, 100}, {0, 0, 100},
        /*------------------------------------------------------------------*/
        /*---------------- CENTER BACK LEFT --------------------------------*/
        {-150, 150, 100}, {0, 0, 65},       {-150, -150, 100}, {0, 0, 100},
        {-150, 150, 100}, {0, 0, 30},       {-150, -150, 100}, {0, 0, 100},
        {-150, 150, 100}, {150, 150, 65},   {-150, -150, 100}, {0, 0, 100},
        {-150, 150, 100}, {150, 150, 100},  {-150, -150, 100}, {0, 0, 100},
        /*------------------------------------------------------------------*/
        /*---------------- CENTER BACK RIGHT -------------------------------*/
        {-150, 150, 100}, {150, 150, 100},  {-150, -150, 100}, {0, 0, 65},
        {-150, 150, 100}, {150, 150, 100},  {-150, -150, 100}, {0, 0, 30},
        {-150, 150, 100}, {150, 150, 100},  {-150, -150, 100}, {150, -150, 65},
        {-150, 150, 100}, {150, 150, 100},  {-150, -150, 100}, {150, -150, 100},
        /*------------------------------------------------------------------*/
    };

    if (frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        frame = NUMBER_OF_FRAMES;
    }

    point_t pt = seq[4 * frame + leg_num];
    if (pt.x == 0) {
        pt.x = gsl_matrix_get(leg->world_end_point, 0, 0);
    }
    if (pt.y == 0) {
        pt.y = gsl_matrix_get(leg->world_end_point, 1, 0);
    }
    if (pt.z == 0) {
        pt.z = gsl_matrix_get(leg->world_end_point, 2, 0);
    }
    return pt;
}
