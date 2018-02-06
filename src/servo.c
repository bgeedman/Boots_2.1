#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "servo.h"
#include "logger.h"


/*
 * Function: servo_create
 * ======================
 * Input:
 *  label - name of servo
 *  pin - SSC-32 pin
 * Return: pointer to created servo
 *
 * This function creates a new servo but does not initialize any fields
 */
Servo *servo_create(const char *label, uint8_t pin) {
    Servo *servo;

    if ((servo = malloc(sizeof(Servo))) == NULL) {
        return NULL;
    }

    servo->label = label;
    servo->pin = pin;

    return servo;
}



/*
 * Function: servo_set_range
 * =========================
 * Input:
 *  servo - pointer to servo struct
 *  min_pos - minimum position of the servo
 *  max_pos - maximum position of the servo
 * Return: N/A
 *
 * This function sets the range for a servo. Can use the calibration tool to
 * get these values.
 */
void servo_set_range(Servo *servo, uint16_t min_pos, uint16_t max_pos) {
    servo->min_position = min_pos;
    servo->max_position = max_pos;
}



/*
 * Function: servo_set_zero_position
 * =================================
 * Input:
 *  servo - pointer to servo structure
 *  pos - position that indicates the 0 Degree for a servo
 * Return: N/A
 *
 * This function sets the zero position for a servo. The zero position is
 * defined as the position that represents 0 degrees for the servo. Can find
 * this value using the calibration tool.
 */
void servo_set_zero_position(Servo *servo, uint16_t pos) {
    servo->zero_position = pos;
}



/*
 * Function: servo_set_desired_position
 * ====================================
 * Input:
 *  servo - pointer to servo structure
 *  pos - desired servo position
 * Return: N/A
 *
 * This function sets the servos desired position for the next update call. If
 * the passedin pos is outside the allowed range, it will set to the correct
 * limit and log a warning.
 *
 * NOTE: This might should be static
 */
void servo_set_desired_position(Servo *servo, uint16_t pos) {
    if (pos > servo->max_position || pos < servo->min_position) {
        log_warn("Attempt to set %s servo outside range (%d - %d): %d",
                servo->label, servo->min_position, servo->max_position, pos);
        servo->des_position = (pos < servo->min_position) ?
                                servo->min_position :
                                servo->max_position;
    } else {
        servo->des_position = pos;
    }
}



/*
 * Function: servo_set_desired_angle
 * =================================
 * Input:
 *  servo - pointer to servo structure
 *  angle - desired angle
 * Return: N/A
 *
 * This function sets the desired angle of the servo, and calls the
 * set_desired_position_function.
 */
void servo_set_desired_angle(Servo *servo, float angle) {
    int position;

    servo->des_angle = angle;
    position = servo->zero_position +
                (servo->inverted * angle * SERVO_TICKS_PER_DEGREE);
    servo_set_desired_position(servo, position);
}



/*
 * Function: servo_set_inverted
 * ============================
 * Input:
 *  servo - pointer to servo struct
 *  invert - flag to invert servo. (1 => standard, -1 => inverted)
 * Return: N/A
 *
 * This function sets the servo to be inverted or standard. An inverted servo
 * is defined if the positive rotation for the kinematics is in a counter-
 * clockwise direction.
 */
void servo_set_inverted(Servo *servo, uint8_t invert) {
    servo->inverted = invert;
}



/*
 * Function: servo_get_pin
 * =======================
 * Input:
 *  servo - pointer to servo struct
 * Return:
 *  pin that servo is connected
 */
uint8_t servo_get_pin(Servo *servo) {
    return servo->pin;
}



/*
 * Function: servo_get_position
 * ============================
 * Input:
 *  servo - pointer to servo struct
 * Output:
 *  servo desired position
 */
uint16_t servo_get_position(Servo *servo) {
    return servo->des_position;
}


/*
 * Function: servo_get_angle
 * =========================
 * Input:
 *  servo - pointer to servo structure
 * Return: current angle of servo
 *
 * This function gets the current angle for a servo. May not be needed
 */
float servo_get_angle(Servo *servo) {
    return servo->des_angle;
}


/*
 * Function: servo_destroy
 * =======================
 * Input:
 *  servo - pointer to servo structure
 * Return: N/A
 *
 * This function frees a servo structure
 */
void servo_destroy(Servo *servo) {
    free(servo);
}
