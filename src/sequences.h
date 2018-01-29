#ifndef SEQUENCES_H
#define SEQUENCES_H

#define SEQUENCES_H_VERSION "0.0.1"

#include "leg.h"

extern unsigned long frame;

typedef struct {
    int x;
    int y;
    int z;
} point_t;


point_t seq_unknown_to_park(int, Leg *leg);
point_t seq_park_to_stand(int, Leg *leg);
point_t seq_stand_to_park(int, Leg *leg);
point_t seq_stand_to_stretch(int, Leg *leg);
point_t seq_stand_to_turn_left(int, Leg *leg);
point_t seq_stand_to_turn_right(int, Leg *leg);
point_t seq_stand_to_walk(int, Leg *leg);

point_t seq_stop_and_center(int, Leg *leg);

#endif
