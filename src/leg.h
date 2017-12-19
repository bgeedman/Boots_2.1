#ifndef LEG_H
#define LEG_H

#define LEG_H_VERSION "0.0.1"

#include <gsl/gsl_matrix.h>
#include <stdint.h>
#include "servo.h"

enum {SHOULDER, FEMUR, TIBIA};

typedef struct Leg {
    const char *label;
    gsl_matrix *rotation_matrix;
    gsl_matrix *translation_matrix;
    Servo **servos;
    uint8_t coxa_len;
    uint8_t femur_len;
    uint8_t tibia_len;

    gsl_matrix *world_end_point;
    gsl_matrix *local_end_point;
} Leg;


Leg *leg_create(const char *label);
int leg_set_rotation(Leg *leg, float angle);
int leg_set_translation(Leg *leg, int8_t delta_x, int8_t delta_y, int8_t delta_z);

void leg_set_coxa(Leg *leg, uint16_t length);
void leg_set_femur(Leg *leg, uint16_t length);
void leg_set_tibia(Leg *leg, uint16_t length);

int leg_add_servo(Leg *leg, int servo, uint8_t pin);
void leg_delete_servo(Leg *leg, int servo);
void leg_set_servo_range(Leg *leg, int servo, uint16_t min_pos, uint16_t max_pos);
void leg_set_servo_position(Leg *leg, int servo, uint16_t pos);
void leg_set_servo_angle(Leg *leg, int servo, float angle);
void leg_set_servo_zero_position(Leg *leg, int servo, uint16_t pos);
void leg_destroy(Leg *leg);
void leg_print_details(Leg *leg);


int leg_set_end_point(Leg *leg, int16_t x, int16_t y, int16_t z);



void __dev_leg_print(Leg *leg);

#endif
