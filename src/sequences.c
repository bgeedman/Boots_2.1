#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <gsl/gsl_matrix.h>
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

#define UP 30
#define DOWN 70
#define N 120


/******************************************************************************
 *                  PARKING SEQUENCE FUNCTIONS                                *
 *****************************************************************************/

point_t seq_unknown_to_park(int leg_num, Leg *leg) {
    log_trace("unknown_to_park: %d", frame);
    static int number_of_frames = 1;

    static point_t seq[] = {
        {-N, N, UP}, {N, N, UP}, {-N, -N, UP}, {N, -N, UP}
    };

    if (frame >= number_of_frames) {
        log_warn("Frame out of range");
        return (point_t){0, 0, 0};
    }

    return seq[4 * frame + leg_num];
}



point_t seq_park_to_stand(int leg_num, Leg *leg) {
    log_trace("park_to_stand: %d", frame);
    static int last_frame = 1;

    static point_t seq[] ={
        /* FRONT_LEFT     FRONT_RIGHT       BACK_LEFT         BACK_RIGHT */
        /*---------------------------------------------------------------*/
        {-N, N,UP}, {N, N, UP}, {-N, -N, UP}, {N, -N, UP},
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN}
    };

    if (frame > last_frame) {
        log_warn("Frame out of range");
        return (point_t){0, 0, 0};
    }
    return seq[4 * frame + leg_num];
}




point_t seq_stand_to_park(int leg_num, Leg *leg) {
    log_trace("stand_to_park: %d", frame);
    static int last_frame = 1;

    static point_t seq[] ={
        /* FRONT_LEFT     FRONT_RIGHT       BACK_LEFT         BACK_RIGHT */
        /*---------------------------------------------------------------*/
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
        {-N, N, UP}, {N, N, UP}, {-N, -N, UP}, {N, -N, UP},
    };

    if (frame > last_frame) {
        log_warn("Frame out of range");
        return (point_t){0, 0, 0};
    }
    return seq[4 * frame + leg_num];
}






/******************************************************************************
 *                  STRETCH SEQUENCE FUNCTIONS                                *
 *****************************************************************************/

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
        /* FRONT_LEFT     FRONT_RIGHT       BACK_LEFT         BACK_RIGHT */
        /*---------------------------------------------------------------*/
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
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



/******************************************************************************
 *                      TURNING SEQUENCE FUNCTIONS                            *
 *****************************************************************************/

point_t seq_stand_to_turn_left_trot(int leg_num, Leg *leg) {
    log_trace("seq_stand_to_turn_left_trot: %d", frame);
    static int last_frame = 8;

    static point_t seq[] = {
        /* FRONT_LEFT      FRONT_RIGHT      BACK_LEFT       BACK_RIGHT -*/
        /*-----------------------------------------------------------------*/
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
        /* MOVE FRONT RIGHT AND BACK LEFT */
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
        {-N, N, DOWN}, {N, N, UP}, {-N, -N, UP}, {N, -N, DOWN},
        {-N, N, DOWN}, {95, 141, UP}, {-95, -141, UP}, {N, -N, DOWN},
        {-N, N, DOWN}, {95, 141, DOWN}, {-95, -141, DOWN}, {N, -N, DOWN},
        /* LIFT FRONT LEFT AND BACK RIGHT AND ROTATE */
        {-N, N, DOWN}, {95, 141, DOWN}, {-95, -141, DOWN}, {N, -N, DOWN},
        {-N, N, UP}, {95, 141, DOWN}, {-95, -141, DOWN}, {N, -N, UP},
        {-N, N, UP}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, UP},
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
        /*-----------------------------------------------------------------*/
    };

    if (frame > last_frame) {
        frame = 0;
    }
    return seq[4 * frame + leg_num];
}


point_t seq_stand_to_turn_right_trot(int leg_num, Leg *leg) {
    log_trace("seq_stand_to_turn_right_trot: %d", frame);
    static int last_frame = 8;
    static point_t seq[] = {
        /* FRONT_LEFT      FRONT_RIGHT      BACK_LEFT       BACK_RIGHT */
        /*-----------------------------------------------------------------*/
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
        /* LIFT FRONT LEFT AND BACK RIGHT */
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
        {-N, N, UP}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, UP},
        {-95, 141, UP}, {N, N, DOWN}, {-N, -N, DOWN}, {95, -141, UP},
        {-95, 141, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {95, -141, DOWN},
        /* LIFT FRONT RIGHT AND BACK LEFT AND ROTATE */
        {-95, 141, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {95, -141, DOWN},
        {-95, 141, DOWN}, {N, N, UP}, {-N, -N, UP}, {95, -141, DOWN},
        {-N, N, DOWN}, {N, N, UP}, {-N, -N, UP}, {N, -N, DOWN},
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
        /*-----------------------------------------------------------------*/
    };
    if (frame > last_frame) {
        frame = 0;
    }
    return seq[4 * frame + leg_num];
}



point_t seq_stand_to_turn_left_crawl(int leg_num, Leg *leg) {
    log_trace("seq_stand_to_turn_left_crawl: %d", frame);
    static int last_frame = 17;

    static point_t seq[] = {
        /* FRONT_LEFT      FRONT_RIGHT      BACK_LEFT       BACK_RIGHT */
        /*-----------------------------------------------------------------*/
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
        /* Lift back right leg and rotate*/
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, UP},
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {141, -95, UP},
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {141, -95, DOWN},
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {141, -95, DOWN},
        /* Lift front right leg and rotate */
        {-N, N, DOWN}, {N, N, UP}, {-N, -N, DOWN}, {141, -95, DOWN},
        {-N, N, DOWN}, {95, 141, UP}, {-N, -N, DOWN}, {141, -95, DOWN},
        {-N, N, DOWN}, {95, 141, DOWN}, {-N, -N, DOWN}, {141, -95, DOWN},
        {-N, N, DOWN}, {95, 141, DOWN}, {-N, -N, DOWN}, {141, -95, DOWN},
        /* Lift back left leg and rotate */
        {-N, N, DOWN}, {95, 141, DOWN}, {-N, -N, UP}, {141, -95, DOWN},
        {-N, N, DOWN}, {95, 141, DOWN}, {-95, -141, UP}, {141, -95, DOWN},
        {-N, N, DOWN}, {95, 141, DOWN}, {-95, -141, DOWN}, {141, -95, DOWN},
        {-N, N, DOWN}, {95, 141, DOWN}, {-95, -141, DOWN}, {141, -95, DOWN},
        /* Lift front left leg and rotate */
        {-N, N, UP}, {95, 141, DOWN}, {-95, -141, DOWN}, {141, -95, DOWN},
        {-141, 95, UP}, {95, 141, DOWN}, {-95, -141, DOWN}, {141, -95, DOWN},
        {-141, 95, DOWN}, {95, 141, DOWN}, {-95, -141, DOWN}, {141, -95, DOWN},
        {-141, 95, DOWN}, {95, 141, DOWN}, {-95, -141, DOWN}, {141, -95, DOWN},
        /* rotate back */
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
    };

    if (frame > last_frame) {
        frame = 0;
    }
    return seq[4 * frame + leg_num];
}


point_t seq_stand_to_turn_right_crawl(int leg_num, Leg *leg) {
    log_trace("seq_stand_to_turn_right_crawl: %d", frame);
    static int last_frame = 17;
    static point_t seq[] = {
        /* FRONT_LEFT      FRONT_RIGHT      BACK_LEFT       BACK_RIGHT */
        /*-----------------------------------------------------------------*/
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
        /* lift back right and rotate */
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, UP},
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {95, -141, UP},
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {95, -141, DOWN},
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {95, -141, DOWN},
        /* lift front right and rotate */
        {-N, N, DOWN}, {N, N, UP}, {-N, -N, DOWN}, {95, -141, DOWN},
        {-N, N, DOWN}, {141, 95, UP}, {-N, -N, DOWN}, {95, -141, DOWN},
        {-N, N, DOWN}, {141, 95, DOWN}, {-N, -N, DOWN}, {95, -141, DOWN},
        {-N, N, DOWN}, {141, 95, DOWN}, {-N, -N, DOWN}, {95, -141, DOWN},
        /* lift back left and rotate */
        {-N, N, DOWN}, {141, 95, DOWN}, {-N, -N, UP}, {95, -141, DOWN},
        {-N, N, DOWN}, {141, 95, DOWN}, {-141, -95, UP}, {95, -141, DOWN},
        {-N, N, DOWN}, {141, 95, DOWN}, {-141, -95, DOWN}, {95, -141, DOWN},
        {-N, N, DOWN}, {141, 95, DOWN}, {-141, -95, DOWN}, {95, -141, DOWN},
        /* lift front right and rotate */
        {-N, N, UP}, {141, 95, DOWN}, {-141, -95, DOWN}, {95, -141, DOWN},
        {-95, 141, UP}, {141, 95, DOWN}, {-141, -95, DOWN}, {95, -141, DOWN},
        {-95, 141, DOWN}, {141, 95, DOWN}, {-141, -95, DOWN}, {95, -141, DOWN},
        {-95, 141, DOWN}, {141, 95, DOWN}, {-141, -95, DOWN}, {95, -141, DOWN},
        /* rotate back */
        {-N, N, DOWN}, {N, N, DOWN}, {-N, -N, DOWN}, {N, -N, DOWN},
    };
    if (frame > last_frame) {
        frame = 0;
    }
    return seq[4 * frame + leg_num];
}



point_t seq_stand_to_turn_left_creep(int leg_num, Leg *leg) {
    log_trace("seq_stand_to_turn_left_creep: %d", frame);
    static int last_frame = 21;

// going to try a shift of 20mm....pray that is enough
// front shift => decrease y
// back shift => increse y
// left shfit => increase x
// right shift => decrease x

    static point_t seq[] = {
        /* FRONT_LEFT      FRONT_RIGHT      BACK_LEFT       BACK_RIGHT */
        /*-----------------------------------------------------------------*/
        {-120, 120, DOWN}, {120, 120, DOWN}, {-120, -120, DOWN}, {120, -120, DOWN},
        {-100, 100, DOWN}, {140, 100, DOWN}, {-100, -140, DOWN}, {140, -140, DOWN},/* left shift */ /*front shift*/
        /* Step back right, step 22.5 degrees, cover 22.5 in power stroke */
        {-98, 102, DOWN}, {147, 98, DOWN}, {-102, -138, DOWN}, {148, -131, 50},
        {-94, 106, DOWN}, {146, 94, DOWN}, {-106, -134, DOWN}, {157, -120, UP},
        {-91, 108, DOWN}, {148, 91, DOWN}, {-108, -131, DOWN}, {164, -110, UP},
        {-87, 132, DOWN}, {152, 107, DOWN}, {-112, -107, DOWN}, {171, -77, 50}, /* start back left shift */
        {-84, 154, DOWN}, {154, 124, DOWN}, {-114, -84, DOWN}, {176, -46, DOWN}, /* end back left shift */
        /* Step front right */
        {-80, 157, DOWN}, {146, 134, 50}, {-117, -80, DOWN}, {175, -49, DOWN},
        {-77, 160, DOWN}, {138, 142, UP}, {-119, -77, DOWN}, {173, -54, DOWN},
        {-72, 162, DOWN}, {127, 152, UP}, {-122, -72, DOWN}, {171, -57, DOWN},
        {-90, 144, DOWN}, {97, 140, 50}, {-144, -90, DOWN}, {148, -82, DOWN},   /* start shift  front right */
        {-105, 127, DOWN}, {65, 127, DOWN}, {-167, -105, DOWN}, {127, -105, DOWN}, /* end front right shift */
        /* Step back left */
        {-102, 128, DOWN}, {70, 124, DOWN}, {-159, -117, 50}, {124, -110, DOWN},
        {-97, 131, DOWN}, {72, 122, DOWN}, {-152, -127, UP}, {122, -112, DOWN},
        {-94, 133, DOWN}, {77, 119, DOWN}, {-142, -138, UP}, {119, -117, DOWN},
        {-89, 155, DOWN}, {80, 137, DOWN}, {-134, -126, 50}, {117, -100, DOWN}, /* start back right shift */
        {-86, 176, DOWN}, {84, 154, DOWN}, {-124, -114, DOWN}, {114, -84, DOWN}, /* end back right shift */
        /* Step front left */
        {-97, 171, 50}, {86, 152, DOWN}, {-127, -112, DOWN}, {112, -87, DOWN},
        {-110, 164, UP}, {91, 148, DOWN}, {-131, -108, DOWN}, {108, -91, DOWN},
        {-120, 157, UP}, {94, 146, DOWN}, {-134, -106, DOWN}, {106, -94, DOWN},
        {-131, 158, 50}, {98, 142, DOWN}, {-138, -102, DOWN}, {102, -98, DOWN}, /* start center shift */
        {-120, 120, DOWN}, {120, 120, DOWN}, {-120, -120, DOWN}, {120, -120, DOWN}, /* end center shift */
    };

    if (frame > last_frame) {
        frame = 0;
    }
    return seq[4 * frame + leg_num];
}


point_t seq_stand_to_turn_right_creep(int leg_num, Leg *leg) {
    log_trace("seq_stand_to_turn_right_creep: %d", frame);
    static int last_frame = 21;

    static point_t seq[] = {
        /* FRONT_LEFT      FRONT_RIGHT      BACK_LEFT       BACK_RIGHT */
        /*-----------------------------------------------------------------*/
        {-120, 120, DOWN}, {120, 120, DOWN}, {-120, -120, DOWN}, {120, -120, DOWN},
        {-100, 100, DOWN}, {140, 100, DOWN}, {-100, -140, DOWN}, {140, -140, DOWN}, // left shift, front shift
        /* step back right */
        {-103, 97, DOWN}, {137, 103, DOWN}, {-97, -143, DOWN}, {130, -149, 50},
        {-106, 94, DOWN}, {134, 106, DOWN}, {-94, -146, DOWN}, {120, -157, UP},
        {-109, 900, DOWN}, {130, 109, DOWN}, {-90, -149, DOWN}, {109, -145, UP},
        {-112, 107, DOWN}, {127, 132, DOWN}, {-87, -132, DOWN}, {97, -151, 50}, //start back left shift
        {-115, 123, DOWN}, {123, 155, DOWN}, {-83, -115, DOWN}, {85, -137, DOWN}, //end back left shift
        /* Step front right */
        {-117, 120, DOWN}, {134, 146, 50}, {-80, -117, DOWN}, {89, -135, DOWN},
        {-120, 116, DOWN}, {143, 137, UP}, {-76, -120, DOWN}, {93, -133, DOWN},
        {-122, 112, DOWN}, {152, 127, UP}, {-72, -122, DOWN}, {97, -131, DOWN},
        {-145, 89, DOWN}, {140, 96, 50}, {-89, -145, DOWN}, {81, -149, DOWN}, //start front right shift
        {-167, 65, DOWN}, {127, 65, DOWN}, {-105, -167, DOWN}, {105, -167, DOWN}, // end front right shift
        /* step back left */
        {-169, 61, DOWN}, {125, 69, DOWN}, {-116, -160, 50}, {109, -165, DOWN},
        {-171, 57, DOWN}, {122, 72, DOWN}, {-127, -152, UP}, {112, -162, DOWN},
        {-173, 53, DOWN}, {120, 76, DOWN}, {-137, -143, UP}, {116, -160, DOWN},
        {-175, 69, DOWN}, {117, 100, DOWN}, {-146, -114, 50}, {120, -137, DOWN}, //start back right shift
        {-177, 85, DOWN}, {115, 123, DOWN}, {-155, -83, DOWN}, {123, -115, DOWN}, // end back right shift
        /* step front left */
        {-171, 97, 50}, {112, 127, DOWN}, {-152, -87, DOWN}, {127, -112, DOWN},
        {-165, 109, UP}, {109, 130, DOWN}, {-149, -90, DOWN}, {130, -109, DOWN},
        {-157, 120, UP}, {106, 134, DOWN}, {-146, -94, DOWN}, {134, -106, DOWN},
        {-149, 130, 50}, {103, 137, DOWN}, {-143, -97, DOWN}, {137, -103, DOWN},
        {-120, 120, DOWN}, {120, 120, DOWN}, {-120, -120, DOWN}, {120, -120, DOWN} // center shift
    };
    if (frame > last_frame) {
        frame = 0;
    }
    return seq[4 * frame + leg_num];
}





/******************************************************************************
 *                          WALKING SEQUENCE FUNCTIONS                        *
 *****************************************************************************/

point_t seq_stand_to_walk_trot(int leg_num, Leg *leg) {
    log_trace("stand_to_walk_trot: %d", frame);

    static int last_frame = 11;

    static point_t seq[] = {
        /* FRONT_LEFT        FRONT_RIGHT        BACK_LEFT       BACK_RIGHT   */
        /*-------------------------------------------------------------------*/
        {-N, N, DOWN},   {N, N, DOWN},   {-N, -N, DOWN},   {N, -N, DOWN},
        {-N, N, DOWN},   {N, N, UP},     {-N, -N, UP},     {N, -N, DOWN},
        {-N, 110, DOWN},   {N, 140, UP},   {-N, -100, UP},   {N, -130, DOWN},
        {-N, 100, DOWN}, {N, 140, DOWN}, {-N, -100, DOWN}, {N, -140, DOWN},
        /* LIFT FRONT LEFT AND BACK RIGHT MOVE FORWARD */
        {-N, 100, UP},   {N, 140, DOWN}, {-N, -110, DOWN}, {N, -140, UP},
        {-N, N, UP},     {N, N, DOWN},   {-N, -N, DOWN},   {N, -N, UP},
        {-N, 140, UP},   {N, 110, DOWN}, {-N, -130, DOWN}, {N, -100, UP},
        {-N, 140, DOWN}, {N, 100, DOWN}, {-N, -140, DOWN}, {N, -100, DOWN},

        /* LIFT FRONT RIGHT AND BACK LEFT MOVE FOREWARD */
        {-N, 130 , DOWN}, {N, 100, UP},   {-N, -140, UP},   {N, -110, DOWN},
        {-N, N, DOWN},    {N, N, UP},     {-N, -N, UP},     {N, -N, DOWN},
        {-N, 110, DOWN},  {N, 140, UP},   {-N, -100, UP},   {N, -130, DOWN},
        {-N, 100, DOWN},  {N, 140, DOWN}, {-N, -100, DOWN}, {N, -140, DOWN},
        /*-------------------------------------------------------------------*/
    };
    if (frame > last_frame) {
        frame = 4;
    }
    return seq[4 * frame + leg_num];
}



point_t seq_stand_to_walk_crawl(int leg_num, Leg *leg) {
    log_trace("stand_to_walk_crawl: %d", frame);

    static int last_frame = 29;
    static point_t seq[] = {
        /* FRONT_LEFT        FRONT_RIGHT        BACK_LEFT       BACK_RIGHT   */
        /*-------------------------------------------------------------------*/
        {-120, 120, DOWN}, {120, 120, DOWN}, {-120, -120, DOWN}, {120, -120, DOWN},
        // shift body front left
        {-110, 110, DOWN}, {130, 110, DOWN}, {-110, -130, DOWN}, {130, -130, DOWN},
        {-100, 100, DOWN}, {140, 100, DOWN}, {-100, -140, DOWN}, {140, -140, DOWN},

        /* step back right */
        {-100, 100, DOWN}, {140, 100, DOWN}, {-100, -140, DOWN}, {140, -140, UP},
        {-100, 100, DOWN}, {140, 100, DOWN}, {-100, -140, DOWN}, {140, -100, UP},
        {-100, 100, DOWN}, {140, 100, DOWN}, {-100, -140, DOWN}, {140, -100, DOWN},

        // shift body back left
        {-100, 120, DOWN}, {140, 120, DOWN}, {-100, -120, DOWN}, {140, -80, DOWN},
        {-100, 130, DOWN}, {140, 130, DOWN}, {-100, -100, DOWN}, {140, -70, DOWN},
        {-100, 140, DOWN}, {140, 140, DOWN}, {-100, -100, DOWN}, {140, -60, DOWN},

        /* step front right */
        {-100, 140, DOWN}, {140, 140, UP}, {-100, -100, DOWN}, {140, -60, DOWN},
        {-100, 140, DOWN}, {140, 180, UP}, {-100, -100, DOWN}, {140, -60, DOWN},
        {-100, 140, DOWN}, {140, 180, DOWN}, {-100, -100, DOWN}, {140, -60, DOWN},

        // shift body back right
        {-120, 140, DOWN}, {120, 180, DOWN}, {-120, -100, DOWN}, {120, -60, DOWN},
        {-130, 140, DOWN}, {110, 180, DOWN}, {-140, -100, DOWN}, {110, -60, DOWN},
        {-140, 140, DOWN}, {100, 180, DOWN}, {-140, -100, DOWN}, {100, -60, DOWN},

        /* step front left */
        {-140, 140, UP}, {100, 180, DOWN}, {-140, -100, DOWN}, {100, -60, DOWN},
        {-140, 180, UP}, {100, 180, DOWN}, {-140, -100, DOWN}, {100, -60, DOWN},
        {-140, 180, DOWN}, {100, 180, DOWN}, {-140, -100, DOWN}, {100, -60, DOWN},

        // shift body front right
        {-140, 180, DOWN}, {100, 160, DOWN}, {-140, -120, DOWN}, {100, -80, DOWN},
        {-140, 170, DOWN}, {100, 150, DOWN}, {-140, -140, DOWN}, {100, -90, DOWN},
        {-140, 160, DOWN}, {100, 140, DOWN}, {-140, -140, DOWN}, {100, -100, DOWN},

        /* step back left */
        {-140, 140, DOWN}, {100, 140, DOWN}, {-140, -120, UP}, {100, -100, DOWN},
        {-140, 140, DOWN}, {100, 140, DOWN}, {-140, -100, UP}, {100, -100, DOWN},
        {-140, 140, DOWN}, {100, 140, DOWN}, {-140, -100, DOWN}, {100, -100, DOWN},

        // shift center
        {-130, 160, DOWN}, {110, 160, DOWN}, {-130, -80, DOWN}, {110, -80, DOWN},
        {-120, 160, DOWN}, {120, 160, DOWN}, {-120, -80, DOWN}, {120, -80, DOWN},

        /* power stroke */
        {-120, 150, DOWN}, {120, 150, DOWN}, {-120, -90, DOWN}, {120, -90, DOWN},
        {-120, 140, DOWN}, {120, 140, DOWN}, {-120, -100, DOWN}, {120, -100, DOWN},
        {-120, 130, DOWN}, {120, 130, DOWN}, {-120, -110, DOWN}, {120, -110, DOWN},
        {-120, 120, DOWN}, {120, 120, DOWN}, {-120, -120, DOWN}, {120, -120, DOWN},
    };
    if (frame > last_frame) {
        frame = 0;
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
point_t seq_stand_to_walk_creep(int leg_num, Leg *leg) {
    log_trace("stand_to_walk: %d", frame);
    static int NUMBER_OF_FRAMES = 45;
    static point_t seq[] = {
        /* FRONT_LEFT        FRONT_RIGHT        BACK_LEFT       BACK_RIGHT -*/
        /*------------------- INITIAL LEG SETUP ----------------------------*/
        {-120, 120, DOWN}, {120, 120, DOWN}, {-120, -120, DOWN}, {120, -120, DOWN},
        {-120, 166, 65}, {120, 145, DOWN}, {-120, -145, DOWN}, {120, -166, DOWN},
        {-120, 182, UP}, {120, 140, DOWN}, {-120, -140, DOWN}, {120, -182, DOWN},
        {-115, 198, UP}, {125, 134, DOWN}, {-115, -134, DOWN}, {125, -198, DOWN}, /* Start left shift*/
        {-110, 214, 65}, {130, 129, DOWN}, {-110, -129, DOWN}, {120, -214, DOWN}, /* start front shift */
        {-105, 230, DOWN}, {135, 123, DOWN}, {-105, -123, DOWN}, {115, -230, DOWN},
        /*------------------------------------------------------------------*/


        /*------------------- BACK RIGHT STEP ------------------------------*/
        {-100, 224, DOWN}, {140, 118, DOWN}, {-100, -128, DOWN}, {140, -214, 50}, /* end front shift, end left shift */
        {-100, 219, DOWN}, {140, 112, DOWN}, {-100, -134, DOWN}, {140, -198, UP},
        {-100, 214, DOWN}, {140, 107, DOWN}, {-100, -139, DOWN}, {140, -182, UP},
        {-100, 208, DOWN}, {140, 102, DOWN}, {-100, -144, DOWN}, {140, -166, UP},
        {-100, 203, DOWN}, {140, 96, DOWN},  {-100, -150, DOWN}, {140, -150, UP},
        {-100, 198, DOWN}, {140, 91, DOWN},  {-100, -155, DOWN}, {140, -134, UP},
        {-100, 192, DOWN}, {140, 86, DOWN},  {-100, -160, DOWN}, {140, -118, UP},
        {-100, 187, DOWN}, {140, 80, DOWN},  {-100, -166, DOWN}, {140, -102, UP},
        {-100, 182, DOWN}, {140, 75, DOWN},  {-100, -172, DOWN}, {140, -86, 50}, /* start back shift */
        {-100, 176, DOWN}, {140, 70, DOWN},  {-100, -176, DOWN}, {140, -70, DOWN},
        /*------------------------------------------------------------------*/
        /*------------------- FRONT RIGHT STEP -----------------------------*/
        {-100, 171, DOWN}, {140, 86, 50},   {-100, -182, DOWN}, {140, -75, DOWN}, /* end back shift */
        {-100, 166, DOWN}, {140, 102, UP},  {-100, -182, DOWN}, {140, -80, DOWN},
        {-100, 160, DOWN}, {140, 118, UP},  {-100, -192, DOWN}, {140, -85, DOWN},
        {-100, 155, DOWN}, {140, 134, UP},  {-100, -198, DOWN}, {140, -91, DOWN},
        {-100, 150, DOWN}, {140, 150, UP},  {-100, -203, DOWN}, {140, -96, DOWN},
        {-110, 144, DOWN}, {134, 166, UP},  {-110, -208, DOWN}, {134, -102, DOWN}, /* start left shift */
        {-116, 139, DOWN}, {128, 182, UP},  {-116, -214, DOWN}, {128, -107, DOWN},
        {-122, 134, DOWN}, {122, 198, UP},  {-122, -219, DOWN}, {122, -112, DOWN},
        {-128, 128, DOWN}, {116, 214, 50},  {-128, -224, DOWN}, {116, -118, DOWN}, /* start front shift */
        {-134, 123, DOWN}, {110, 230, DOWN},  {-134, -230, DOWN}, {110, -123, DOWN},
        /*------------------------------------------------------------------*/
        /*------------------- BACK LEFT STEP -------------------------------*/
        {-140, 118, DOWN}, {100, 224, DOWN}, {-140, -214, 50},  {100, -128, DOWN},  /* end left shift, end front shift */
        {-140, 112, DOWN}, {100, 219, DOWN}, {-140, -198, UP},  {100, -134, DOWN},
        {-140, 107, DOWN}, {100, 214, DOWN}, {-140, -182, UP},  {100, -139, DOWN},
        {-140, 102, DOWN}, {100, 208, DOWN}, {-140, -166, UP},  {100, -144, DOWN},
        {-140, 96, DOWN},  {100, 203, DOWN}, {-140, -150, UP},  {100, -150, DOWN},
        {-140, 91, DOWN},  {100, 198, DOWN}, {-140, -134, UP},  {100, -155, DOWN},
        {-140, 86, DOWN},  {100, 192, DOWN}, {-140, -118, UP},  {100, -160, DOWN},
        {-140, 80, DOWN},  {100, 187, DOWN}, {-140, -102, UP},  {100, -166, DOWN},
        {-140, 75, DOWN},  {100, 182, DOWN}, {-140, -86, 50},   {100, -171, DOWN}, /* start back shift */
        {-140, 70, DOWN},  {100, 176, DOWN}, {-140, -70, DOWN},   {100, -176, DOWN},
        /*------------------------------------------------------------------*/
        /*------------------- FRONT LEFT STEP ------------------------------*/
        {-140, 86, 50},   {100, 171, DOWN}, {-140, -75, DOWN},  {100, -182, DOWN}, /* end back shift */
        {-140, 102, UP},  {100, 166, DOWN}, {-140, -80, DOWN},  {100, -187, DOWN},
        {-140, 118, UP},  {100, 160, DOWN}, {-140, -86, DOWN},  {100, -192, DOWN},
        {-140, 134, UP},  {100, 155, DOWN}, {-140, -91, DOWN},  {100, -198, DOWN},
        {-140, 150, UP},  {100, 150, DOWN}, {-140, -96, DOWN},  {100, -203, DOWN},
        {-134, 166, UP},  {110, 144, DOWN}, {-134, -102, DOWN}, {110, -208, DOWN}, /* start right shift */
        {-128, 182, UP},  {116, 139, DOWN}, {-128, -107, DOWN}, {116, -214, DOWN},
        {-122, 198, UP},  {122, 134, DOWN}, {-122, -112, DOWN}, {122, -219, DOWN},
        {-116, 214, 50},  {128, 128, DOWN}, {-116, -118, DOWN}, {128, -224, DOWN}, /* start front shift */
        {-110, 230, DOWN},  {134, 123, DOWN}, {-110, -123, DOWN}, {134, -230, DOWN}
        /*------------------------------------------------------------------*/

    };
    if (frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        frame = 6;
    }
    return seq[4 * frame + leg_num];
}





/******************************************************************************
 *                      CENTERING SEQUENCE FUNCTIONS                          *
 *****************************************************************************/
point_t seq_stop_and_center(int leg_num, Leg *leg) {
    log_trace("stop_and_center: %d", frame);
    log_warn("THIS FUNCTION IS FUCKED");

    static int last_frame = 26;

    static point_t seq[] = {
        /* FRONT_LEFT        FRONT_RIGHT        BACK_LEFT       BACK_RIGHT  */
        /*---------------- DROP ALL THE FEET -------------------------------*/
        {0, 0, DOWN},       {0, 0, DOWN},      {0, 0, DOWN},       {0, 0, DOWN},
        {0, 0, DOWN},       {0, 0, DOWN},      {0, 0, DOWN},       {0, 0, DOWN},
        /* STEP BACK RIGHT */
        {0, 0, DOWN},       {0, 0, DOWN},      {0, 0, DOWN},       {0, 0, DOWN},
        {0, 0, DOWN},       {0, 0, DOWN},      {0, 0, DOWN},       {0, 0, 50},
        {0, 0, DOWN},       {0, 0, DOWN},      {0, 0, DOWN},       {0, 0, UP},
        {0, 0, DOWN},       {0, 0, DOWN},      {0, 0, DOWN},       {N, -N, UP},
        {0, 0, DOWN},       {0, 0, DOWN},      {0, 0, DOWN},       {N, -N, UP},
        {0, 0, DOWN},       {0, 0, DOWN},      {0, 0, DOWN},       {N, -N, DOWN},
        /* STEP FRONT RIGHT */
        {0, 0, DOWN},       {0, 0, DOWN},      {0, 0, DOWN},       {N, -N, DOWN},
        {0, 0, DOWN},       {0, 0, 50},      {0, 0, DOWN},       {N, -N, DOWN},
        {0, 0, DOWN},       {0, 0, UP},      {0, 0, DOWN},       {N, -N, DOWN},
        {0, 0, DOWN},       {N, N, UP},      {0, 0, DOWN},       {N, -N, DOWN},
        {0, 0, DOWN},       {N, N, UP},      {0, 0, DOWN},       {N, -N, DOWN},
        {0, 0, DOWN},       {N, N, DOWN},      {0, 0, DOWN},       {N, -N, DOWN},
        /* STEP BACK LEFT */
        {0, 0, DOWN},       {N, N, DOWN},      {0, 0, DOWN},       {N, -N, DOWN},
        {0, 0, DOWN},       {N, N, DOWN},      {0, 0, 50},       {N, -N, DOWN},
        {0, 0, DOWN},       {N, N, DOWN},      {0, 0, UP},       {N, -N, DOWN},
        {0, 0, DOWN},       {N, N, DOWN},      {-N, -N, UP},       {N, -N, DOWN},
        {0, 0, DOWN},       {N, N, DOWN},      {-N, -N, UP},       {N, -N, DOWN},
        {0, 0, DOWN},       {N, N, DOWN},      {-N, -N, DOWN},       {N, -N, DOWN},
        /* STEP FRONT LEFT */
        {0, 0, DOWN},       {N, N, DOWN},      {-N, -N, DOWN},       {N, -N, DOWN},
        {0, 0, 50},       {N, N, DOWN},      {-N, -N, DOWN},       {N, -N, DOWN},
        {0, 0, UP},       {N, N, DOWN},      {-N, -N, DOWN},       {N, -N, DOWN},
        {-N, N, UP},       {N, N, DOWN},      {-N, -N, DOWN},       {N, -N, DOWN},
        {-N, N, UP},       {N, N, DOWN},      {-N, -N, DOWN},       {N, -N, DOWN},
        {-N, N, DOWN},       {N, N, DOWN},      {-N, -N, DOWN},       {N, -N, DOWN},
        /* DONE */
        {-N, N, DOWN},       {N, N, DOWN},      {-N, -N, DOWN},       {N, -N, DOWN},
    };

    if (frame > last_frame) {
        log_warn("Frame out of range");
        return (point_t){0, 0, 0};
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
