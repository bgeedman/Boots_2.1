#include <errno.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "kinematics.h"
#include "logger.h"
#include "tools.h"

static float law_of_cosines(float a, float b, float c);
static float solve_hypotenuse(float a, float b);


/*
 * Function: kinematics_geometric
 * ==============================
 * Input:
 *  leg - pointer to leg that we need to solve kinematic for
 * Return: Success or failure
 *
 * This function solves the inverse kinematics for a passed in leg.  Solution
 * will be for the leg local end point. The solution will be stored in each of
 * the leg servos by setting the desired angle.
 */
int kinematics_geometric(Leg *leg) {
    float length;
    float HF;
    float theta;
    float alpha1, alpha2, alpha;
    float beta;
    float x;
    float y;
    float z;
    float LL;
    float tmp;

    if (leg->local_end_point == NULL) {
        log_warn("Local end point matrix is NULL");
        return 1;
    }

    x = gsl_matrix_get(leg->local_end_point, 0, 0);
    y = gsl_matrix_get(leg->local_end_point, 1, 0);
    z = gsl_matrix_get(leg->local_end_point, 2, 0);

    if (y != 0) {
        tmp = atan2f(y, x);
        /* if atan2f returns NaN, comparing to self will return false */
        if (tmp != tmp) {
            log_warn("NaN returned when calculating theta");
            return 1;
        }
        theta = radiansToDegrees(tmp);
    } else {
        theta = 0.0;
    }

    if ((length = solve_hypotenuse(x, y)) < 0) {
        log_warn("Failed to calculate length when solving kinematics");
        return 1;
    }

    LL = length - leg->coxa_len;
    tmp = ((LL * LL) + (z * z) - (leg->femur_len * leg->femur_len) - (leg->tibia_len * leg->tibia_len)) /
            (2 * leg->femur_len * leg->tibia_len);

    if (tmp <= -1 || tmp >= 1) {
        log_warn("No solution should exists!");
        return 1;
    }

    beta = radiansToDegrees(acosf(tmp));
    alpha1 = radiansToDegrees(atan2f(LL, z));
    if ((HF = solve_hypotenuse(LL, z)) < 0) {
        log_warn("Failed to calculate HF when solving kinematics");
        return 1;
    }
    alpha2 = radiansToDegrees(law_of_cosines(leg->femur_len, HF, leg->tibia_len));
    alpha = 90.0 - (alpha1 + alpha2);

    leg_set_servo_angle(leg, SHOULDER, theta);
    leg_set_servo_angle(leg, FEMUR, alpha);
    leg_set_servo_angle(leg, TIBIA, beta);

    return 0;
}


static float solve_hypotenuse(float a, float b) {
    int error;

    float x = powf(a, 2);
    if (errno == ERANGE) goto FAIL;

    float y = powf(b, 2);
    if (errno == ERANGE) goto FAIL;

    float z = sqrtf(x + y);
    if (errno == EDOM) goto FAIL;

    return z;

FAIL:
    error = errno;
    log_warn("Failed to solve hypotenuse: %s", strerror(errno));
    errno = error;
    return -1;
}


static float law_of_cosines(float a, float b, float c) {
    float a2, b2, c2;
    float gamma;
    int error;

    a2 = powf(a, 2);
    if (errno == ERANGE) goto FAIL;

    b2 = powf(b, 2);
    if (errno == ERANGE) goto FAIL;

    c2 = powf(c, 2);
    if (errno == ERANGE) goto FAIL;

    gamma = acosf((a2 + b2 - c2) / (2 * a * b));
    if (errno == EDOM) goto FAIL;

    return gamma;

FAIL:
    error = errno;
    log_warn("Failed to solve law_of_cosines: %s", strerror(errno));
    errno = error;
    return -1;
}
