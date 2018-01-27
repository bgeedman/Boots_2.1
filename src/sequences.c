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


point_t seq_unknown_to_park(int cur_frame, int leg) {
    log_trace("unknown_to_park: %d", cur_frame);
    static int NUMBER_OF_FRAMES = 0;
    static point_t seq[] = {
        {-150, 40, 30}, {150, 40, 30}, {-150, -50, 30}, {150, -40, 30}
    };

    if (cur_frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        cur_frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * cur_frame + leg];
}




/*
 * TODO: MIGHT CHANGE THIS SEQUENCE SO IT DOES ALL FOUR LEGS AT ONCE TO KEEP
 * BALANCE
 */
point_t seq_park_to_stand(int cur_frame, int leg) {
    log_trace("park_to_stand: %d", cur_frame);
    static int NUMBER_OF_FRAMES = 9;
    static point_t seq[] = {
        {-150, 40, 30}, {150, 40, 30}, {-150, -40, 30}, {150, -40, 30},
        // raise the body a little bit
        {-150, 40, 100}, {150, 40, 100}, {-150, -40, 100}, {150, -40, 100},
        // move the front left leg
        {-150, 150, 0}, {150, 40, 100}, {-150, -40, 100}, {150, -40, 100},
        {-150, 150, 100}, {150, 40, 100}, {-150, -40, 100}, {150, -40, 100},
        // move the front right leg
        {-150, 150, 100}, {150, 150, 0}, {-150, -40, 100}, {150, -40, 100},
        {-150, 150, 100}, {150, 150, 100}, {-150, -40, 100}, {150, -40, 100},
        // move the back left leg
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 0}, {150, -40, 100},
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -40, 100},
        // move the back right leg
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 0},
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100}
    };

    if (cur_frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        cur_frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * cur_frame + leg];
}


/*
 * TODO: MIGHT CHANGE THIS SEQUENCE SO IT DOES ALL FOUR LEGS AT ONCE TO KEEP
 * BALANCE
 */
point_t seq_stand_to_park(int cur_frame, int leg) {
    log_trace("stand_to_park: %d", cur_frame);
    static int NUMBER_OF_FRAMES = 9;
    static point_t seq[] = {
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -150, 100},
        // move the back right leg
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -40, 30},
        {-150, 150, 100}, {150, 150, 100}, {-150, -150, 100}, {150, -40, 100},
        // move the back left leg
        {-150, 150, 100}, {150, 150, 100}, {-150, -40, 30}, {150, -40, 100},
        {-150, 150, 100}, {150, 150, 100}, {-150, -40, 100}, {150, -40, 100},
        // move the front right leg
        {-150, 150, 100}, {150, 40, 30}, {-150, -40, 100}, {150, -40, 100},
        {-150, 150, 100}, {150, 40, 100}, {-150, -40, 100}, {150, -40, 100},
        // move the front left leg
        {-150, 40, 30}, {150, 40, 100}, {-150, -40, 100}, {150, -40, 100},
        {-150, 40, 100}, {150, 40, 100}, {-150, -40, 100}, {150, -40, 100},
        // lower the body a bit
        {-150, 40, 30}, {150, 40, 30}, {-150, -40, 30}, {150, -40, 30},
    };

    if (cur_frame > NUMBER_OF_FRAMES) {
        log_warn("Frame out of range");
        cur_frame = NUMBER_OF_FRAMES;
    }
    return seq[4 * cur_frame + leg];
}





point_t seq_stand_to_stretch(int cur_frame, int leg) {
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
    gsl_matrix_set(original_point, 0, 0, seq[leg].x);
    gsl_matrix_set(original_point, 1, 0, seq[leg].y);
    gsl_matrix_set(original_point, 2, 0, seq[leg].z);
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


point_t seq_stand_to_turn_left(int cur_frame, int leg) {
    return (point_t){0, 0, 0};
}


point_t seq_stand_to_turn_right(int cur_frame, int leg) {
    return (point_t){0, 0, 0};
}


point_t seq_stand_to_walk(int cur_frame, int leg) {
    return (point_t){0, 0, 0};
}


point_t seq_stop_and_center(int cur_frame, int leg) {
    return (point_t){0, 0, 0};
}
