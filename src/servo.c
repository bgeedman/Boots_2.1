#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include "servo.h"
#include "logger.h"


Servo *servo_create(const char *label, uint8_t pin) {
    Servo *servo;
    if ((servo = malloc(sizeof(Servo))) == NULL) {
        return NULL;
    }
    servo->label = label;
    servo->pin = pin;
    return servo;
}

void servo_set_range(Servo *servo, uint16_t min_pos, uint16_t max_pos) {
    servo->min_position = min_pos;
    servo->max_position = max_pos;
}

void servo_set_zero_position(Servo *servo, uint16_t pos) {
    servo->zero_position = pos;
}

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

void servo_set_desired_angle(Servo *servo, float angle) {
    // Check the position is not out of range
    servo->des_angle = angle;
}

float servo_get_angle(Servo *servo) {
    return servo->des_angle;
}

void servo_print_details(Servo *servo) {
    if (servo == NULL) {
        log_info("Null Servo");
        return;
    }
    char buf[1024];

    snprintf(buf, sizeof(buf),
            "\n------------------------------\n"
            "%s servo\n"
            "Pin: %d\n"
            "Range: %d - %d\n"
            "Desired Position: %d\n"
            "Current Position: %d\n"
            "Desired Angle: %.2f\n"
            "Current Angle: %.2f\n"
            "\n------------------------------",
            servo->label,
            servo->pin,
            servo->min_position,
            servo->max_position,
            servo->des_position,
            servo->cur_position,
            servo->des_angle,
            servo->cur_angle);
    log_info(buf);
}

void servo_destroy(Servo *servo) {
    free(servo);
}


