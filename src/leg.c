#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_blas.h>
#include "leg.h"
#include "logger.h"
#include "tools.h"


static const char *labels[3] = {"Shoulder", "Femur", "Tibia"};


/*
 * Function: leg_create
 * ====================
 * Input:
 *  label - name of the leg
 * Return: Pointer to a leg structure
 *
 * This funtion allocates a leg structure and initializes all members to NULL
 * or zero where applicable
 */
Leg *leg_create(const char *label) {
    Leg *leg;
    if ((leg = malloc(sizeof(Leg))) == NULL) {
        return NULL;
    }
    leg->label = label;
    if ((leg->servos = calloc(3, sizeof(Servo *))) == NULL) {
        free(leg);
        return NULL;
    }
    leg->rotation_matrix = NULL;
    leg->translation_matrix = NULL;
    leg->world_end_point = NULL;
    leg->local_end_point = NULL;
    return leg;
}



/*
 * Function: leg_set_rotation
 * ==========================
 * Input:
 *  leg - pointer to leg structure
 *  angle - angle of rotation
 * Return: success code
 *
 * This function sets the Z-axis rotation angle for the passed in leg.  If the
 * matrix does not exist, it will create one.
 */
int leg_set_rotation(Leg *leg, float angle) {
    if (leg->rotation_matrix == NULL) {
        if ((leg->rotation_matrix = gsl_matrix_alloc(4, 4)) == NULL) {
            log_error("Failed to allocate space for rotation matrix");
            return 1;
        }
    }
    gsl_matrix_set_identity(leg->rotation_matrix);
    gsl_matrix_set(leg->rotation_matrix, 0, 0, cos(degreesToRadians(angle)));
    gsl_matrix_set(leg->rotation_matrix, 0, 1, -1 * sin(degreesToRadians(angle)));
    gsl_matrix_set(leg->rotation_matrix, 1, 0, sin(degreesToRadians(angle)));
    gsl_matrix_set(leg->rotation_matrix, 1, 1, cos(degreesToRadians(angle)));
    return 0;
}



/*
 * Function: leg_set_translation
 * =============================
 * Input:
 *  leg - pointer to leg structure
 *  delta_x - offset in millimeters along the global X-axis to leg origin
 *  delta_y - offset in millimeters along the global Y-axis to leg origin
 *  delta_z - offset in millimeters along the global Z-axis to leg origin
 * Return: status code
 *
 * This function sets the translation matrix for the passed in leg. Deltas are
 * in units of millimeters and defines the legs local origin from global origin
 */
int leg_set_translation(Leg *leg, int8_t delta_x, int8_t delta_y, int8_t delta_z) {
    if (leg->translation_matrix == NULL) {
        if ((leg->translation_matrix = gsl_matrix_alloc(4, 4)) == NULL) {
            log_error("Failed to allocate space for translation matrix");
            return 1;
        }
    }
    gsl_matrix_set_identity(leg->translation_matrix);
    gsl_matrix_set(leg->translation_matrix, 0, 3, delta_x);
    gsl_matrix_set(leg->translation_matrix, 1, 3, delta_y);
    gsl_matrix_set(leg->translation_matrix, 2, 3, delta_z);
    return 0;
}



/*
 * Function: leg_set_coxa
 * ======================
 * Input:
 *  leg - pointer to leg structure
 *  len - length of coxa in millimeters
 * Return: N/A
 *
 * This function sets the length of the coxa in millimeters for the passed leg
 */
void leg_set_coxa(Leg *leg, uint16_t len) {
    leg->coxa_len = len;
}



/*
 * Function: leg_set_femur
 * =======================
 * Input:
 *   leg - pointer to leg structure
 *   len - length of femur in millimeters
 * Return: N/A
 *
 * This function sets the length of the femur in millimeters for the passed leg
 */
void leg_set_femur(Leg *leg, uint16_t len) {
    leg->femur_len = len;
}



/*
 * Function: leg_set_tibia
 * =======================
 * Input:
 *  leg - pointer to leg structure
 *  len - length of tibia in millimeters
 * Return: N/A
 *
 * This function sets the lenght of the tibia in millimeters for the passed leg
 */
void leg_set_tibia(Leg *leg, uint16_t len) {
    leg->tibia_len = len;
}



/*
 * Function: leg_add_servo
 * =======================
 * Input:
 *  leg - pointer to leg structure
 *  servo - servo to add
 *  pin - servo pin on SSC-32
 * Return: success code
 *
 * This function adds a servo to the leg in the servo position. enum in leg.h
 * provides servo constants.
 */
int leg_add_servo(Leg *leg, int servo, uint8_t pin) {
    if (leg->servos[servo] != NULL) {
        return 1;
    }
    Servo *p_servo = servo_create(labels[servo], pin);
    if (p_servo == NULL) {
        return 1;
    }
    leg->servos[servo] = p_servo;
    return 0;
}



/*
 * Function: leg_delete_servo
 * ==========================
 * Input:
 *  leg - pointer to leg structure
 *  servo - servo to delete
 * Return: N/A
 *
 * This function removes a servo from the leg
 */
void leg_delete_servo(Leg *leg, int servo) {
    free(leg->servos[servo]);
    leg->servos[servo] = NULL;
}



/*
 * Function: leg_set_servo_range
 * =============================
 * Input:
 *  leg - pointer to leg structure
 *  servo - servo to update
 *  min_pos - minimum servo position
 *  max_pos - maximum servo position
 * Return: N/A
 *
 * This function sets a servos range
 */
void leg_set_servo_range(Leg *leg, int servo, uint16_t min_pos, uint16_t max_pos) {
    if (leg->servos[servo] != NULL) {
        servo_set_range(leg->servos[servo], min_pos, max_pos);
    }
}



/*
 * Function: leg_set_servo_zero_position
 * =====================================
 * Input:
 *  leg - pointer to leg structure
 *  servo - servo to update
 *  pos - zero position
 *
 *  This function sets the zero position for a servo.  The zero position is
 *  defined as the position of the servo at 0 degrees
 */
void leg_set_servo_zero_position(Leg *leg, int servo, uint16_t pos) {
    if (leg->servos[servo] != NULL) {
        servo_set_zero_position(leg->servos[servo], pos);
    }
}



/*
 * Function: leg_set_servo_angle
 * =============================
 * Input:
 *  leg - pointer to leg structure
 *  servo - servo to update
 *  angle - desired angle of servo
 * Return: N/A
 *
 * This function sets the desired angle for a servo
 */
void leg_set_servo_angle(Leg *leg, int servo, float angle) {
    if (leg->servos[servo] != NULL) {
        servo_set_desired_angle(leg->servos[servo], angle);
    }
}



/*
 * Function: leg_destroy
 * =====================
 */
void leg_destroy(Leg *leg) {
    log_debug("Destroying leg");
    servo_destroy(leg->servos[SHOULDER]);
    servo_destroy(leg->servos[FEMUR]);
    servo_destroy(leg->servos[TIBIA]);
    free(leg->servos);
    gsl_matrix_free(leg->rotation_matrix);
    gsl_matrix_free(leg->translation_matrix);
    gsl_matrix_free(leg->world_end_point);
    gsl_matrix_free(leg->local_end_point);
    free(leg);
}

void leg_print_details(Leg *leg) {
    char buf[1024];
    snprintf(buf, 1024,
            "\n%s\n"
            "Coxa: %d\n"
            "Femur: %d\n"
            "Tibia: %d",
            leg->label,
            leg->coxa_len,
            leg->femur_len,
            leg->tibia_len);
    log_info(buf);
    tools_matrix_print("Rotation Matrix", leg->rotation_matrix);
    tools_matrix_print("Translation Matrix", leg->translation_matrix);
    tools_matrix_print("World End Point Matrix:", leg->world_end_point);
    tools_matrix_print("Local End Point Matrix:", leg->local_end_point);
    servo_print_details(leg->servos[SHOULDER]);
    servo_print_details(leg->servos[FEMUR]);
    servo_print_details(leg->servos[TIBIA]);
}


int leg_set_end_point(Leg *leg, int16_t x, int16_t y, int16_t z) {
    if (leg->world_end_point == NULL) {
        if ((leg->world_end_point = gsl_matrix_alloc(4, 1)) == NULL) {
            log_error("Failed to allocate world coordinate space");
            return 1;
        }
    }
    if (leg->local_end_point == NULL) {
        if ((leg->local_end_point = gsl_matrix_alloc(4, 1)) == NULL) {
            log_error("Failed to allocate local coordinate space");
            return 1;
        }
    }
    gsl_matrix_set(leg->world_end_point, 0, 0, x);
    gsl_matrix_set(leg->world_end_point, 1, 0, y);
    gsl_matrix_set(leg->world_end_point, 2, 0, z);
    gsl_matrix_set(leg->world_end_point, 3, 0, 1);

    gsl_matrix *tmp;
    if ((tmp = gsl_matrix_alloc(4, 1)) == NULL) {
        log_error("Failed to allocate tmp matrix\n");
        return 1;
    }
    // translate
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,
                    leg->translation_matrix, leg->world_end_point, 0.0,
                    tmp);
    // rotate
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0,
                    leg->rotation_matrix, tmp, 0.0, leg->local_end_point);

    gsl_matrix_free(tmp);
    return 0;
}
