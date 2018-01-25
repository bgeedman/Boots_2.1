#ifndef SERVO_H
#define SERVO_H

#include <stdlib.h>
#include <stdint.h>


#define SERVO_VERSION "0.0.1"

#define SERVO_TICKS_PER_DEGREE                  10.0

typedef struct Servo {
    const char *label;
    int8_t inverted; // when the neg angle has higher pos than 0_pos, set to -1
    uint8_t pin;
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
void servo_print_details(Servo *servo);
void servo_destroy(Servo *servo);

#endif
