#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include "logger.h"
#include "kinematics.h"
#include "tools.h"

static float solve_hypotenuse(float a, float b);
static float law_of_cosines(float a, float b, float c);


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
        log_warn("local end point matrix is NULL");
        return 1;
    }
    x = gsl_matrix_get(leg->local_end_point, 0, 0);
    y = gsl_matrix_get(leg->local_end_point, 1, 0);
    z = gsl_matrix_get(leg->local_end_point, 2, 0);
    log_debug("(%.2f,%.2f,%.2f)", x, y, z);
    if (y != 0) {
        tmp = atan2f(y, x);
        /* if atan2f returns NaN, comparing to self will return false */
        if (tmp != tmp) {
            log_warn("NaN returned when calculating theta");
            log_warn("X: %g, Y: %d", x, y);
            return 1;
        }
        theta = radiansToDegrees(tmp);
    } else {
        theta = 0.0;
    }

    log_debug("Theta: %.2f", theta);

    length = solve_hypotenuse(x, y);
    LL = length - leg->coxa_len;
    log_debug("Length: %.2f", length);
    log_debug("LL: %.2f", LL); // this is without coxa...aka what we want
    tmp = ((LL * LL) + (z * z) - (leg->femur_len * leg->femur_len) - (leg->tibia_len * leg->tibia_len)) /
            (2 * leg->femur_len * leg->tibia_len);
    log_debug("tmp: %2f", tmp);
    if (tmp <= -1 || tmp >= 1) {
        log_warn("No solution should exists");
        return 1;
    }
    beta = radiansToDegrees(acosf(tmp));
    log_debug("Beta: %.2f", beta);

    alpha1 = radiansToDegrees(atan2f(LL, z));
    log_debug("Alpha1: %.2f", alpha1);
    HF = solve_hypotenuse(LL, z);
    log_debug("HF: %.2f", HF);
    alpha2 = radiansToDegrees(law_of_cosines(leg->femur_len, HF, leg->tibia_len));
    log_debug("Alpha2: %.2f", alpha2);
    alpha = 90.0 - (alpha1 + alpha2);
    log_debug("Alpha: %.2f", alpha);
    leg_set_servo_angle(leg, SHOULDER, theta);
    leg_set_servo_angle(leg, FEMUR, alpha);
    leg_set_servo_angle(leg, TIBIA, beta);
    return 0;
}


static float solve_hypotenuse(float a, float b) {
    // TODO: ALL KINDS OF ERROR CHECKING
    float x = powf(a, 2);
    float y = powf(b, 2);
    float z = sqrtf(x + y);
    return z;
}


static float law_of_cosines(float a, float b, float c) {
    // TODO: ERROR CHECKING
    float a2, b2, c2;
    float gamma;
    a2 = powf(a, 2);
    b2 = powf(b, 2);
    c2 = powf(c, 2);
    /* log_debug("law of cosines -- a: %.2f, a2: %.2f", a, a2); */
    /* log_debug("law of cosines -- b: %.2f, b2: %.2f", b, b2); */
    /* log_debug("law of cosines -- c: %.2f, c2: %.2f", c, c2); */
    gamma = acosf((a2 + b2 - c2) / (2 * a * b));
    return gamma;
}
