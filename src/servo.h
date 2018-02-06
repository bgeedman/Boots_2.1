#ifndef __SERVO_H
#define __SERVO_H

#define __SERVO_VERSION "0.0.1"

#include <stdint.h>
#include <stdlib.h>

#define SERVO_TICKS_PER_DEGREE  10.0

typedef struct Servo {
    const char *label;
    uint8_t pin;
    int8_t inverted;
    float des_angle;
    uint16_t des_position;
    uint16_t zero_position;
    uint16_t min_position;
    uint16_t max_position;
} Servo;


Servo *servo_create(const char *label, uint8_t pin);

void servo_set_range(Servo *servo, uint16_t min_pos, uint16_t max_pos);
void servo_set_zero_position(Servo *servo, uint16_t pos);
void servo_set_desired_position(Servo *servo, uint16_t pos);
void servo_set_desired_angle(Servo *servo, float angle);
void servo_set_inverted(Servo *servo, uint8_t invert);
uint8_t servo_get_pin(Servo *servo);
uint16_t servo_get_position(Servo *servo);
float servo_get_angle(Servo *servo);
void servo_destroy(Servo *servo);

#endif
